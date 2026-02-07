#!/usr/bin/env python3
"""
Genesis Simulation Runner — SHM Frame Pipeline + NATS Command/Telemetry

Runs Genesis Go2 environment headless, encodes rendered camera frames,
and writes them to the shared-memory ringbuffer at
/dev/shm/sdr_os_ipc/frames for the Rust transport-server to relay to
browser clients via WebSocket.

Connects to NATS for command reception and telemetry publishing.
Publishes canonical safety state (ARMED/HOLD/ESTOP) as sim authority.

Architecture:
  Genesis scene → camera.render() → encode → ShmRingWriter
  transport-server reads SHM → WebSocket fanout → browser
  NATS command.genesis.> → sim (this process)
  sim → NATS telemetry.* → transport-server → browser
"""

import sys
import os
import json
import time
import signal
import logging
import asyncio
import pickle
import glob
from pathlib import Path

# Ensure project root on path
_project_root = str(Path(__file__).resolve().parent.parent)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)
_bridge_root = os.path.join(_project_root, "ref", "my_fuck_up", "code")
if _bridge_root not in sys.path:
    sys.path.insert(0, _bridge_root)

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

import numpy as np
import cv2
import torch

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("genesis_sim_runner")

# ── SHM ringbuffer writer ─────────────────────────────────────────
from src.sdr_os.ipc.shm_ringbuffer import ShmRingWriter, FrameFlags, Codec

SHM_PATH = os.environ.get("SDR_SHM_PATH", "/dev/shm/sdr_os_ipc/frames")
SHM_SIZE = int(os.environ.get("SDR_SHM_SIZE", 4 * 1024 * 1024))

# ── Genesis + Forge ────────────────────────────────────────────────
import genesis as gs
from genesis_bridge.envs.go2_env import Go2BridgeEnv


class ActorMLP(torch.nn.Module):
    """Simple actor MLP matching rsl_rl ActorCritic checkpoint format."""

    def __init__(self, obs_dim: int, action_dim: int, hidden_dims: list, activation="elu"):
        super().__init__()
        act_fn = torch.nn.ELU() if activation == "elu" else torch.nn.ReLU()
        layers = []
        in_dim = obs_dim
        for h in hidden_dims:
            layers.append(torch.nn.Linear(in_dim, h))
            layers.append(act_fn)
            in_dim = h
        layers.append(torch.nn.Linear(in_dim, action_dim))
        self.actor = torch.nn.Sequential(*layers)
        self.std = torch.nn.Parameter(torch.zeros(action_dim))

    def forward(self, obs):
        return self.actor(obs)

    def act_inference(self, obs):
        return self.actor(obs)


def load_policy(checkpoint_dir: str, model_file: str = None, obs_dim: int = 310):
    """Load a trained locomotion policy from checkpoint directory."""
    if model_file:
        model_path = os.path.join(checkpoint_dir, model_file)
    else:
        model_files = sorted(glob.glob(os.path.join(checkpoint_dir, "model_*.pt")))
        if not model_files:
            logger.error(f"No model files found in {checkpoint_dir}")
            return None
        model_path = model_files[-1]

    logger.info(f"Loading policy from {model_path}")

    cfg_path = os.path.join(checkpoint_dir, "cfgs.pkl")
    policy_cfg = {}
    if os.path.exists(cfg_path):
        with open(cfg_path, "rb") as f:
            cfgs = pickle.load(f)
        if isinstance(cfgs, (list, tuple)) and len(cfgs) >= 1:
            policy_cfg = cfgs[0].get("policy", {})
        elif isinstance(cfgs, dict):
            policy_cfg = cfgs.get("policy", {})

    hidden_dims = policy_cfg.get("actor_hidden_dims", [512, 256, 128])
    activation = policy_cfg.get("activation", "elu")
    num_actions = 12  # Go2 has 12 joints

    policy = ActorMLP(obs_dim, num_actions, hidden_dims, activation).to(gs.device)

    checkpoint = torch.load(model_path, map_location=gs.device, weights_only=False)
    state_dict = checkpoint.get("model_state_dict", checkpoint)

    # Load only the actor weights + std (skip critic)
    actor_state = {k: v for k, v in state_dict.items() if k.startswith("actor.") or k == "std"}
    policy.load_state_dict(actor_state, strict=True)
    policy.eval()

    logger.info(f"Policy loaded: {os.path.basename(model_path)} ({sum(p.numel() for p in policy.parameters())} params)")
    return policy


class JpegEncoder:
    """JPEG fallback encoder."""
    def __init__(self, quality=80):
        self.quality = quality
        self.codec = Codec.JPEG

    def encode(self, frame_rgb, frame_id):
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return None, False
        return buf.tobytes(), True  # JPEG = always keyframe

    def close(self):
        pass


class NvencEncoder:
    """H.264 NVENC hardware encoder producing Annex-B NAL units."""
    def __init__(self, width, height, fps=30, bitrate=3_000_000, gop=30):
        import av
        self.codec = Codec.H264
        self.gop = gop
        self.frame_count = 0

        self.container = av.open(
            "pipe:",
            mode="w",
            format="h264",
            options={
                "preset": "p4",
                "tune": "ull",
                "profile": "baseline",
                "b": str(bitrate),
                "g": str(gop),
                "bf": "0",
            },
        )
        self.stream = self.container.add_stream("h264_nvenc", rate=fps)
        self.stream.width = width
        self.stream.height = height
        self.stream.pix_fmt = "yuv420p"

    def encode(self, frame_rgb, frame_id):
        import av
        frame = av.VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        frame.pts = frame_id
        packets = self.stream.encode(frame)
        if not packets:
            return None, False

        payload = b"".join(bytes(p) for p in packets)
        is_keyframe = self.frame_count % self.gop == 0
        self.frame_count += 1
        return payload, is_keyframe

    def close(self):
        try:
            self.container.close()
        except Exception:
            pass


class GenesisSimRunner:
    """Runs Genesis Go2 env and writes frames to SHM.

    Uses NATS for command reception and telemetry publishing.
    Implements Layer 3 safety: cmd TTL decay and ESTOP.
    """

    def __init__(
        self,
        target_fps: int = 30,
        camera_res: tuple = (1280, 720),
        jpeg_quality: int = 80,
        checkpoint_dir: str = None,
    ):
        self.target_fps = target_fps
        self.camera_res = camera_res
        self.jpeg_quality = jpeg_quality
        self.checkpoint_dir = checkpoint_dir

        self.env = None
        self.policy = None
        self.current_obs = None
        self.shm_writer = None
        self.encoder = None
        self.frame_id = 0
        self.running = False

        # NATS
        self.nc = None
        self.cmd_sub = None
        self.gate_sub = None

        # Safety state (Layer 3 — sim is canonical authority)
        self._safety_mode = "ARMED"  # ARMED | HOLD | ESTOP
        self._safety_state_id = 0
        self._safety_reason = "ok"
        self._last_cmd_seq = 0
        self._last_cmd_vel_time = time.monotonic()
        self._video_gate_active = False
        self.paused = False

    def init_genesis(self):
        """Initialize Genesis scene and Go2 environment."""
        logger.info("Initializing Genesis GPU backend...")
        gs.init(backend=gs.gpu)

        logger.info(f"Creating Go2BridgeEnv (camera: {self.camera_res})...")
        self.env = Go2BridgeEnv(
            num_envs=1,
            dt=1 / 50,
            max_episode_length_s=None,
            headless=True,
            camera_res=self.camera_res,
        )
        self.env.build()

        obs, _ = self.env.reset()
        self.current_obs = obs
        logger.info("Go2BridgeEnv initialized and reset")

        # Load policy if checkpoint provided
        if self.checkpoint_dir and os.path.exists(self.checkpoint_dir):
            obs_dim = obs.shape[-1] if obs is not None else 310
            self.policy = load_policy(self.checkpoint_dir, obs_dim=obs_dim)
            if self.policy:
                logger.info("Policy loaded, robot will be controlled by learned policy")
            else:
                logger.warning("Policy loading failed, using zero actions")
        else:
            logger.info("No checkpoint provided, using zero actions")

        # Initialize encoder (NVENC with JPEG fallback)
        width, height = self.camera_res
        try:
            self.encoder = NvencEncoder(width, height, fps=self.target_fps)
            logger.info(f"NVENC H.264 encoder initialized ({width}x{height})")
        except Exception as e:
            logger.warning(f"NVENC unavailable ({e}), falling back to JPEG")
            self.encoder = JpegEncoder(quality=self.jpeg_quality)

    def init_shm(self):
        """Initialize SHM ringbuffer writer."""
        self.shm_writer = ShmRingWriter(path=SHM_PATH, buffer_size=SHM_SIZE)
        logger.info(f"SHM writer ready at {SHM_PATH} ({SHM_SIZE / 1024 / 1024:.0f}MB)")

    def render_and_write(self):
        """Render camera frame, encode (NVENC or JPEG), write to SHM."""
        render_result = self.env.camera.render()
        img = render_result[0] if isinstance(render_result, tuple) else render_result

        if isinstance(img, torch.Tensor):
            frame_np = img.cpu().numpy()
        else:
            frame_np = np.asarray(img)
        if frame_np.ndim == 4:
            frame_np = frame_np[0]

        payload, is_keyframe = self.encoder.encode(frame_np, self.frame_id)
        if payload is None:
            return

        flags = FrameFlags.KEYFRAME if is_keyframe else FrameFlags.NONE

        self.shm_writer.write(
            payload=payload,
            frame_id=self.frame_id,
            flags=flags,
            codec=self.encoder.codec,
        )
        self.frame_id += 1

    def step_sim(self):
        """Step the simulation with policy or zero actions."""
        if self._safety_mode == "ESTOP":
            # In ESTOP: zero velocity, no policy
            actions = torch.zeros(1, 12, device=gs.device)
        elif self.policy is not None and self.current_obs is not None:
            with torch.no_grad():
                actions = self.policy.act_inference(self.current_obs)
        else:
            actions = torch.zeros(1, 12, device=gs.device)

        obs, _, dones, _, _ = self.env.step(actions)
        self.current_obs = obs

        if dones.any():
            obs, _ = self.env.reset()
            self.current_obs = obs

    # ── NATS ──────────────────────────────────────────────────────

    async def connect_nats(self):
        """Connect to NATS for command/telemetry."""
        import nats as nats_client

        nats_url = os.environ.get("SDR_NATS_URL", "nats://localhost:4222")
        self.nc = await nats_client.connect(nats_url)
        logger.info(f"Connected to NATS at {nats_url}")

        # Subscribe to all genesis commands
        self.cmd_sub = await self.nc.subscribe("command.genesis.>")
        asyncio.create_task(self._handle_commands())

        # Subscribe to video gate from transport
        self.gate_sub = await self.nc.subscribe("telemetry.safety.video_gate")
        asyncio.create_task(self._handle_video_gate())

    async def _handle_commands(self):
        """Process incoming NATS commands."""
        async for msg in self.cmd_sub.messages:
            try:
                action = msg.subject.split(".")[-1]
                data = json.loads(msg.data.decode()) if msg.data else {}
                cmd_data = data.get("data", data)
                cmd_seq = data.get("cmd_seq", 0)

                # Out-of-order protection for velocity commands
                if action == "set_cmd_vel":
                    if cmd_seq <= self._last_cmd_seq:
                        continue
                    self._last_cmd_seq = cmd_seq
                    self._last_cmd_vel_time = time.monotonic()
                    # Recover from HOLD on fresh command
                    if self._safety_mode == "HOLD":
                        self._safety_mode = "ARMED"
                        self._safety_reason = "ok"
                    if self.env:
                        self.env.set_velocity_from_gamepad(cmd_data)
                    continue

                if action == "pause":
                    self.paused = cmd_data.get("paused", True)
                elif action == "reset":
                    if self.env:
                        obs, _ = self.env.reset()
                        self.current_obs = obs
                elif action == "estop":
                    self._safety_mode = "ESTOP"
                    self._safety_reason = cmd_data.get("reason", "operator")
                    logger.warning(f"ESTOP triggered: {self._safety_reason}")
                elif action == "estop_clear":
                    if not self._video_gate_active:
                        self._safety_mode = "ARMED"
                        self._safety_reason = "operator_clear"
                        self._last_cmd_vel_time = time.monotonic()
                        logger.info("ESTOP cleared by operator")
                    else:
                        logger.warning("ESTOP clear rejected — video gate still active")
                elif action == "load_policy":
                    checkpoint_dir = cmd_data.get("checkpoint_dir", "")
                    model_file = cmd_data.get("model_file")
                    if checkpoint_dir and os.path.exists(checkpoint_dir):
                        obs_dim = self.current_obs.shape[-1] if self.current_obs is not None else 310
                        self.policy = load_policy(checkpoint_dir, model_file=model_file, obs_dim=obs_dim)
                elif action == "load_robot":
                    pass  # TODO: implement robot loading
                elif action == "set_mode":
                    pass  # TODO: implement mode switching
                elif action == "camera":
                    pass  # TODO: implement camera control
                elif action == "settings":
                    if "jpeg_quality" in cmd_data:
                        self.jpeg_quality = int(cmd_data["jpeg_quality"])
                        if isinstance(self.encoder, JpegEncoder):
                            self.encoder.quality = self.jpeg_quality
                    if "stream_fps" in cmd_data:
                        self.target_fps = int(cmd_data["stream_fps"])

                # Publish ack
                ack = {"action": action, "cmd_seq": cmd_seq, "status": "ok"}
                await self.nc.publish("telemetry.command.ack", json.dumps(ack).encode())

            except Exception as e:
                logger.error(f"Command handler error: {e}")

    async def _handle_video_gate(self):
        """Process video gate notifications from transport."""
        async for msg in self.gate_sub.messages:
            try:
                data = json.loads(msg.data.decode())
                self._video_gate_active = data.get("gated", False)
                if self._video_gate_active and data.get("mode") == "ESTOP":
                    self._safety_mode = "ESTOP"
                    self._safety_reason = "video_timeout"
            except Exception:
                pass

    # ── Safety (Layer 3) ──────────────────────────────────────────

    def _enforce_cmd_ttl(self):
        """Layer 3: TTL decay and ESTOP on command timeout."""
        if self._safety_mode == "ESTOP":
            return

        elapsed = time.monotonic() - self._last_cmd_vel_time
        if elapsed > 2.0:
            self._safety_mode = "ESTOP"
            self._safety_reason = "cmd_timeout"
            logger.warning("Command timeout >2s — ESTOP")
        elif elapsed > 0.2:
            if self._safety_mode != "HOLD":
                self._safety_mode = "HOLD"
                self._safety_reason = "cmd_timeout"
                logger.warning("Command TTL expired — HOLD")

    # ── Main loop ─────────────────────────────────────────────────

    async def run(self):
        """Main sim loop."""
        self.running = True
        frame_interval = 1.0 / self.target_fps
        step_count = 0
        last_metrics_time = 0
        last_safety_time = 0

        await self.connect_nats()

        logger.info(f"Starting sim loop at {self.target_fps} FPS")

        try:
            while self.running:
                t0 = time.monotonic()

                # Enforce command TTL (Layer 3)
                self._enforce_cmd_ttl()

                # Step physics (unless paused)
                if not self.paused:
                    self.step_sim()
                step_count += 1

                # Render and write to SHM
                self.render_and_write()

                now = time.monotonic()

                # Publish telemetry every ~1s
                if now - last_metrics_time > 1.0 and self.nc and self.nc.is_connected:
                    metrics = {
                        "step": step_count,
                        "fps": self.target_fps,
                        "policy_loaded": self.policy is not None,
                        "paused": self.paused,
                    }
                    if self.env:
                        metrics["velocity_command"] = self.env.get_velocity_command()

                    await self.nc.publish("telemetry.training.metrics", json.dumps(metrics).encode())

                    if self.env:
                        try:
                            reward = self.env.get_reward_breakdown()
                            if reward:
                                await self.nc.publish("telemetry.reward.breakdown", json.dumps(reward).encode())
                        except Exception:
                            pass
                        try:
                            obs = self.env.get_obs_breakdown()
                            if obs:
                                await self.nc.publish("telemetry.obs.breakdown", json.dumps(obs).encode())
                        except Exception:
                            pass

                    last_metrics_time = now

                # Publish canonical safety state at 2 Hz
                if now - last_safety_time > 0.5 and self.nc and self.nc.is_connected:
                    self._safety_state_id += 1
                    state = {
                        "state_id": self._safety_state_id,
                        "mode": self._safety_mode,
                        "reason": self._safety_reason,
                        "since_ms": int((now - self._last_cmd_vel_time) * 1000),
                    }
                    await self.nc.publish("telemetry.safety.state", json.dumps(state).encode())
                    last_safety_time = now

                # Frame pacing
                elapsed = time.monotonic() - t0
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    await asyncio.sleep(sleep_time)

        except KeyboardInterrupt:
            logger.info("Interrupted")
        finally:
            self.running = False
            if self.shm_writer:
                self.shm_writer.close()
            if self.nc and self.nc.is_connected:
                await self.nc.close()
            logger.info("Sim runner stopped")

    def stop(self):
        self.running = False


async def main():
    import argparse

    parser = argparse.ArgumentParser(description="Genesis Simulation Runner (SHM + NATS)")
    parser.add_argument("--fps", type=int, default=30, help="Target render FPS")
    parser.add_argument("--camera-res", type=str, default="1280x720", help="Camera resolution WxH")
    parser.add_argument("--jpeg-quality", type=int, default=80, help="JPEG encode quality (1-100)")
    parser.add_argument("--checkpoint", type=str, default=None, help="Path to policy checkpoint directory")
    args = parser.parse_args()

    w, h = args.camera_res.lower().split("x")
    camera_res = (int(w), int(h))

    # Default checkpoint path
    checkpoint = args.checkpoint
    if checkpoint is None:
        default_ckpt = os.path.join(_project_root, "rl", "checkpoints", "go2-locomotion")
        if os.path.exists(default_ckpt):
            checkpoint = default_ckpt

    runner = GenesisSimRunner(
        target_fps=args.fps,
        camera_res=camera_res,
        jpeg_quality=args.jpeg_quality,
        checkpoint_dir=checkpoint,
    )

    # Handle SIGTERM gracefully
    def on_signal(sig, frame):
        logger.info(f"Received signal {sig}")
        runner.stop()

    signal.signal(signal.SIGTERM, on_signal)
    signal.signal(signal.SIGINT, on_signal)

    runner.init_genesis()
    runner.init_shm()
    await runner.run()


if __name__ == "__main__":
    asyncio.run(main())
