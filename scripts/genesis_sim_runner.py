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
  Main thread:  step_sim() → render() → push frame to queue
  Encoder thread: pop frame → encode → ShmRingWriter
  NATS async:   command.genesis.> → sim state
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
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

# Ensure project root on path
_project_root = str(Path(__file__).resolve().parent.parent)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

os.environ.setdefault("PYOPENGL_PLATFORM", "egl")

# ── GPU device selection (must be set BEFORE importing genesis) ──
# Force PCI bus ordering so CUDA indices match nvidia-smi output.
os.environ.setdefault("CUDA_DEVICE_ORDER", "PCI_BUS_ID")
_gpu_id = os.environ.get("SDR_GPU_ID", "")
if _gpu_id:
    os.environ["CUDA_VISIBLE_DEVICES"] = _gpu_id
    os.environ["TI_VISIBLE_DEVICE"] = _gpu_id
    os.environ["EGL_DEVICE_ID"] = _gpu_id

import numpy as np
import cv2
import torch

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("genesis_sim_runner")

# ── Optional turbojpeg ────────────────────────────────────────────
try:
    from turbojpeg import TurboJPEG, TJPF_RGB
    _turbojpeg = TurboJPEG()
    logger.info("turbojpeg available — using for JPEG encoding")
except ImportError:
    _turbojpeg = None

# ── SHM ringbuffer writer ─────────────────────────────────────────
from src.sdr_os.ipc.shm_ringbuffer import ShmRingWriter, FrameFlags, Codec

SHM_PATH = os.environ.get("SDR_SHM_PATH", "/dev/shm/sdr_os_ipc/frames")
SHM_SIZE = int(os.environ.get("SDR_SHM_SIZE", 4 * 1024 * 1024))

# ── Genesis + Forge ────────────────────────────────────────────────
import genesis as gs


# ── BT.601 color matrix for GPU-side RGB→YUV ──────────────────────
# Standard BT.601 limited-range coefficients (Y 16-235, UV 16-240)
_BT601_MAT = torch.tensor([
    [ 0.257,  0.504,  0.098],
    [-0.148, -0.291,  0.439],
    [ 0.439, -0.368, -0.071],
], dtype=torch.float32)
_BT601_OFFSET = torch.tensor([16.0, 128.0, 128.0], dtype=torch.float32)


def _rgb_to_yuv420p_gpu(tensor):
    """Convert an RGB uint8 tensor (H,W,3) to YUV420p planes on GPU.

    Uses BT.601 limited-range conversion:
      Y  =  0.257*R + 0.504*G + 0.098*B + 16
      Cb = -0.148*R - 0.291*G + 0.439*B + 128
      Cr =  0.439*R - 0.368*G - 0.071*B + 128

    Chroma is subsampled 2x2 via simple averaging.

    Returns (Y, U, V) as contiguous uint8 CPU numpy arrays.
    Transfer is ~1.3 bytes/pixel (YUV420p) vs 3 bytes/pixel (RGB24).
    """
    global _BT601_MAT, _BT601_OFFSET
    device = tensor.device
    _BT601_MAT = _BT601_MAT.to(device)
    _BT601_OFFSET = _BT601_OFFSET.to(device)

    # (H, W, 3) float32 matmul
    rgb_f = tensor.float()
    yuv = rgb_f @ _BT601_MAT.T + _BT601_OFFSET  # (H, W, 3)
    yuv = yuv.clamp(0, 255).to(torch.uint8)

    y_plane = yuv[:, :, 0].contiguous()
    u_full = yuv[:, :, 1]
    v_full = yuv[:, :, 2]

    # Chroma subsample 2x2 — average four pixels
    # Reshape to (H/2, 2, W/2, 2), mean over dims 1 and 3
    h, w = u_full.shape
    u_sub = u_full.reshape(h // 2, 2, w // 2, 2).float().mean(dim=(1, 3)).to(torch.uint8).contiguous()
    v_sub = v_full.reshape(h // 2, 2, w // 2, 2).float().mean(dim=(1, 3)).to(torch.uint8).contiguous()

    return y_plane.cpu().numpy(), u_sub.cpu().numpy(), v_sub.cpu().numpy()


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
    """JPEG encoder — uses turbojpeg if available, else OpenCV fallback."""
    def __init__(self, quality=80):
        self.quality = quality
        self.codec = Codec.JPEG

    def encode(self, frame_rgb, frame_id, force_idr=False):
        if _turbojpeg is not None:
            buf = _turbojpeg.encode(frame_rgb, quality=self.quality, pixel_format=TJPF_RGB)
            return bytes(buf), True
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return None, False
        return buf.tobytes(), True  # JPEG = always keyframe

    def close(self):
        pass


class NvencEncoder:
    """H.264 NVENC hardware encoder producing Annex-B NAL units.

    Uses a standalone codec context (no container/muxer) so that
    encode() returns packets directly with inline SPS/PPS on every IDR.

    Accepts either:
    - frame_rgb (numpy H,W,3 uint8) — legacy path, does from_ndarray + reformat
    - yuv_planes (Y, U, V) numpy arrays — fast path from GPU conversion
    """
    def __init__(self, width, height, fps=30, bitrate=5_000_000,
                 gop=30, preset="p1", intra_refresh=True):
        import av
        from fractions import Fraction

        self.codec = Codec.H264
        self.frame_count = 0
        self._width = width
        self._height = height
        self._intra_refresh = intra_refresh

        self.ctx = av.CodecContext.create("h264_nvenc", "w")
        self.ctx.width = width
        self.ctx.height = height
        self.ctx.pix_fmt = "yuv420p"
        self.ctx.framerate = Fraction(fps, 1)
        self.ctx.time_base = Fraction(1, fps)
        self.ctx.bit_rate = bitrate
        self.ctx.gop_size = gop
        self.ctx.max_b_frames = 0
        opts = {
            "preset": preset,
            "tune": "ull",
            "rc": "cbr",
            "bf": "0",
            "forced-idr": "1",
            "repeat_headers": "1",
            "zerolatency": "1",
            "surfaces": "8",
            "async_depth": "2",
        }
        if intra_refresh:
            opts["intra-refresh"] = "1"
        self.ctx.options = opts
        self.ctx.open()

        # Pre-allocate a reusable VideoFrame for the fast YUV path
        self._yuv_frame = av.VideoFrame(width, height, "yuv420p")

    def encode(self, frame_rgb, frame_id, force_idr=False, yuv_planes=None):
        import av
        if yuv_planes is not None:
            # Fast path: write pre-converted YUV planes into pre-allocated frame
            y, u, v = yuv_planes
            frame = self._yuv_frame
            frame.planes[0].update(y.tobytes())
            frame.planes[1].update(u.tobytes())
            frame.planes[2].update(v.tobytes())
        else:
            # Legacy path: RGB numpy → from_ndarray → reformat
            frame = av.VideoFrame.from_ndarray(frame_rgb, format="rgb24")
            frame = frame.reformat(format="yuv420p", width=self.ctx.width, height=self.ctx.height)
        frame.pts = frame_id
        if force_idr:
            frame.pict_type = av.video.frame.PictureType.I
        packets = self.ctx.encode(frame)
        if not packets:
            return None, False

        payload = b"".join(bytes(p) for p in packets)
        is_keyframe = any(p.is_keyframe for p in packets)
        self.frame_count += 1
        return payload, is_keyframe

    def close(self):
        try:
            self.ctx.encode(None)
        except Exception:
            pass


# ── Threaded encoder pipeline ─────────────────────────────────────

@dataclass
class EncodeRequest:
    """Frame data passed from main loop to encoder thread."""
    frame_np: Optional[np.ndarray]  # RGB numpy (for JPEG or legacy H.264)
    frame_tensor: Optional[object]  # torch.Tensor on GPU (for GPU YUV path)
    frame_id: int
    force_idr: bool


class EncoderThread:
    """Background thread that encodes frames and writes to SHM.

    Uses a single-slot "latest wins" queue — if the encoder is slower
    than the sim, old frames are dropped (not queued up).

    The encoder and SHM writer are owned exclusively by this thread.
    """
    def __init__(self, encoder, shm_writer):
        self.encoder = encoder
        self.shm_writer = shm_writer
        self._request: Optional[EncodeRequest] = None
        self._pending_close = None  # Old encoder awaiting close by thread
        self._event = threading.Event()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="encoder", daemon=True)

        # Instrumentation (written by encoder thread, read by main thread)
        self._encode_times = []
        self._frame_sizes = []
        self._frames_encoded = 0
        self._stats_lock = threading.Lock()

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._event.set()  # Wake up if blocked
        self._thread.join(timeout=2.0)
        # Close any pending-close encoder that the thread didn't get to
        if self._pending_close is not None:
            try:
                self._pending_close.close()
            except Exception:
                pass
            self._pending_close = None

    def submit(self, request: EncodeRequest):
        """Submit a frame for encoding (latest-wins, non-blocking)."""
        with self._lock:
            self._request = request
        self._event.set()

    def swap_encoder(self, new_encoder):
        """Thread-safe encoder swap (for codec switch / param change).

        The old encoder is not closed here — the encoder thread closes
        it on its next iteration to avoid racing with an in-flight encode.
        """
        with self._lock:
            self._pending_close = self.encoder
            self.encoder = new_encoder
            self._request = None  # Discard any pending frame for old encoder
        self._event.set()  # Wake thread to handle swap

    def snapshot_stats(self):
        """Read and reset instrumentation counters (called from main thread)."""
        with self._stats_lock:
            times = list(self._encode_times)
            sizes = list(self._frame_sizes)
            total = self._frames_encoded
            self._encode_times.clear()
            self._frame_sizes.clear()
        return times, sizes, total

    def _run(self):
        logger.info("Encoder thread started")
        while not self._stop.is_set():
            self._event.wait()
            self._event.clear()
            if self._stop.is_set():
                break

            # Grab the latest request and close any swapped-out encoder
            with self._lock:
                to_close = self._pending_close
                self._pending_close = None
                req = self._request
                self._request = None
                encoder = self.encoder

            if to_close is not None:
                try:
                    to_close.close()
                except Exception:
                    pass

            if req is None:
                continue

            try:
                t_enc = time.monotonic()

                # Choose encode path based on encoder type and available data
                yuv_planes = None
                if isinstance(encoder, NvencEncoder) and req.frame_tensor is not None:
                    yuv_planes = _rgb_to_yuv420p_gpu(req.frame_tensor)

                payload, is_keyframe = encoder.encode(
                    req.frame_np, req.frame_id,
                    force_idr=req.force_idr,
                    yuv_planes=yuv_planes,
                ) if isinstance(encoder, NvencEncoder) else encoder.encode(
                    req.frame_np, req.frame_id,
                    force_idr=req.force_idr,
                )

                encode_ms = (time.monotonic() - t_enc) * 1000

                if payload is None:
                    continue

                with self._stats_lock:
                    self._encode_times.append(encode_ms)
                    self._frame_sizes.append(len(payload))
                    self._frames_encoded += 1

                flags = FrameFlags.KEYFRAME if is_keyframe else FrameFlags.NONE
                self.shm_writer.write(
                    payload=payload,
                    frame_id=req.frame_id,
                    flags=flags,
                    codec=encoder.codec,
                )
            except Exception as e:
                logger.error(f"Encoder thread error: {e}")

        logger.info("Encoder thread stopped")


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
        self._encoder_thread: Optional[EncoderThread] = None
        self.frame_id = 0
        self.running = False
        self.h264_bitrate = 5_000_000
        self.h264_preset = "p1"
        self._force_idr = False

        # Encoder instrumentation (legacy — now in EncoderThread)
        self._encode_times = []
        self._frame_sizes = []
        self._frames_encoded = 0
        self._last_encode_stats_time = 0

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
        self._cmd_vel_received = False  # Don't enforce TTL until first command arrives
        self._video_gate_active = False
        self._gait_enabled = False  # L2 held = gait walking, released = position hold
        self._stand_axes = [0.0, 0.0, 0.0, 0.0]  # pitch, roll, yaw, height
        self.paused = False
        self._step_log_counter = 0  # throttle step_sim logging
        self._cmd_log_counter = 0   # throttle cmd_vel logging
        self._gains_mode = "walk"   # "walk" or "stand" — tracks current PD gains

    def init_genesis(self):
        """Initialize Genesis scene and Go2 environment."""
        logger.info("Initializing Genesis GPU backend...")
        gs.init(backend=gs.gpu, performance_mode=True)

        # Import after gs.init() — GaitCommandManager touches genesis.engine at import time
        from src.sdr_os.envs.go2_env import Go2BridgeEnv

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
        logger.info(
            f"Go2BridgeEnv initialized and reset — obs shape: {obs.shape if obs is not None else None}, "
            f"device: {obs.device if obs is not None else None}"
        )

        # Load policy if checkpoint provided
        if self.checkpoint_dir and os.path.exists(self.checkpoint_dir):
            logger.info(f"Attempting policy load from: {self.checkpoint_dir}")
            obs_dim = obs.shape[-1] if obs is not None else 310
            logger.info(f"Policy obs_dim={obs_dim} (obs tensor shape: {obs.shape if obs is not None else 'None'})")
            self.policy = load_policy(self.checkpoint_dir, obs_dim=obs_dim)
            if self.policy:
                logger.info(f"Policy loaded OK — type={type(self.policy).__name__}, device={next(self.policy.parameters()).device}")
                # Smoke test: run one inference to verify shapes match
                try:
                    with torch.no_grad():
                        test_actions = self.policy.act_inference(obs)
                    logger.info(
                        f"Policy smoke test PASSED — input {obs.shape} → output {test_actions.shape}, "
                        f"act_mean={test_actions.abs().mean().item():.4f}, act_max={test_actions.abs().max().item():.4f}"
                    )
                except Exception as e:
                    logger.error(f"Policy smoke test FAILED: {e}")
                    self.policy = None
            else:
                logger.warning("Policy loading returned None, using zero actions")
        else:
            logger.info(f"No checkpoint provided (checkpoint_dir={self.checkpoint_dir!r}), using zero actions")

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
        crc_enabled = os.environ.get("SDR_CRC_ENABLED", "1") != "0"
        self.shm_writer = ShmRingWriter(path=SHM_PATH, buffer_size=SHM_SIZE, crc_enabled=crc_enabled)
        logger.info(f"SHM writer ready at {SHM_PATH} ({SHM_SIZE / 1024 / 1024:.0f}MB, crc={'on' if crc_enabled else 'off'})")

    def _start_encoder_thread(self):
        """Start the background encoder thread."""
        self._encoder_thread = EncoderThread(self.encoder, self.shm_writer)
        self._encoder_thread.start()

    def _stop_encoder_thread(self):
        """Stop the background encoder thread."""
        if self._encoder_thread:
            self._encoder_thread.stop()
            self._encoder_thread = None

    def _scan_checkpoints(self):
        """Scan rl/checkpoints/ for policy directories and standalone .pt files."""
        ckpt_root = os.path.join(_project_root, "rl", "checkpoints")
        if not os.path.isdir(ckpt_root):
            return []

        policies = []
        loaded_dir = os.path.abspath(self.checkpoint_dir) if self.checkpoint_dir else None

        for entry in sorted(os.scandir(ckpt_root), key=lambda e: e.name):
            if entry.is_dir():
                model_files = sorted(glob.glob(os.path.join(entry.path, "model_*.pt")))
                if not model_files:
                    continue

                # Determine algorithm from cfgs.pkl
                algorithm = "PPO"
                cfg_path = os.path.join(entry.path, "cfgs.pkl")
                if os.path.exists(cfg_path):
                    try:
                        with open(cfg_path, "rb") as f:
                            cfgs = pickle.load(f)
                        if isinstance(cfgs, (list, tuple)) and len(cfgs) >= 5:
                            train_cfg = cfgs[4]
                            if isinstance(train_cfg, dict):
                                algorithm = train_cfg.get("algorithm", {}).get("class_name", "PPO")
                    except Exception:
                        pass

                # Extract step numbers and sort numerically
                checkpoints = [os.path.basename(f) for f in model_files]
                steps = []
                for name in checkpoints:
                    try:
                        steps.append(int(name.replace("model_", "").replace(".pt", "")))
                    except ValueError:
                        pass
                # Sort checkpoints by step number (numeric) instead of alphabetic
                checkpoints.sort(key=lambda n: int(n.replace("model_", "").replace(".pt", "")) if n.startswith("model_") else 0)

                total_size = sum(os.path.getsize(f) for f in model_files)
                latest_mtime = max(os.path.getmtime(f) for f in model_files)

                is_loaded = loaded_dir is not None and os.path.abspath(entry.path) == loaded_dir
                loaded_ckpt = None
                if is_loaded and self.policy is not None and hasattr(self, '_loaded_model_file'):
                    loaded_ckpt = self._loaded_model_file

                policies.append({
                    "name": entry.name,
                    "path": entry.path,
                    "type": "directory",
                    "algorithm": algorithm,
                    "checkpoints": checkpoints,
                    "num_checkpoints": len(checkpoints),
                    "latest_step": max(steps) if steps else None,
                    "size_mb": round(total_size / (1024 * 1024), 1),
                    "modified_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(latest_mtime)),
                    "is_loaded": is_loaded,
                    "loaded_checkpoint": loaded_ckpt,
                })

            elif entry.is_file() and entry.name.endswith(".pt"):
                policies.append({
                    "name": entry.name,
                    "path": entry.path,
                    "type": "file",
                    "algorithm": "unknown",
                    "checkpoints": [entry.name],
                    "num_checkpoints": 1,
                    "latest_step": None,
                    "size_mb": round(entry.stat().st_size / (1024 * 1024), 1),
                    "modified_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(entry.stat().st_mtime)),
                    "is_loaded": False,
                    "loaded_checkpoint": None,
                })

        return policies

    def _recreate_encoder(self):
        """Recreate H.264 encoder with current params, fallback to JPEG."""
        width, height = self.camera_res
        try:
            new_encoder = NvencEncoder(
                width, height,
                fps=self.target_fps,
                bitrate=self.h264_bitrate,
                preset=self.h264_preset,
            )
            logger.info(f"NVENC encoder recreated: bitrate={self.h264_bitrate}, preset={self.h264_preset}")
        except Exception as e:
            logger.warning(f"NVENC recreation failed ({e}), falling back to JPEG")
            new_encoder = JpegEncoder(quality=self.jpeg_quality)

        self.encoder = new_encoder
        if self._encoder_thread:
            self._encoder_thread.swap_encoder(new_encoder)

    def render_and_enqueue(self):
        """Render camera frame and submit to encoder thread."""
        render_result = self.env.camera.render(rgb=True, depth=False, segmentation=False, normal=False)
        img = render_result[0] if isinstance(render_result, tuple) else render_result

        frame_tensor = None
        if isinstance(img, torch.Tensor):
            if img.ndim == 4:
                img = img[0]
            # Keep tensor on GPU for H.264 GPU color conversion path
            if isinstance(self.encoder, NvencEncoder):
                frame_tensor = img
            frame_np = img.cpu().numpy()
        else:
            frame_np = np.asarray(img)
            if frame_np.ndim == 4:
                frame_np = frame_np[0]

        force_idr = self._force_idr
        if self._force_idr:
            self._force_idr = False

        req = EncodeRequest(
            frame_np=frame_np,
            frame_tensor=frame_tensor,
            frame_id=self.frame_id,
            force_idr=force_idr,
        )
        self.frame_id += 1

        if self._encoder_thread:
            self._encoder_thread.submit(req)

    def _switch_gains(self, mode):
        """Switch PD gains if mode changed. Returns True if switched."""
        if mode == self._gains_mode or not self.env:
            return False
        if mode == "stand":
            self.env.set_stand_gains()
        else:
            self.env.set_walk_gains()
        logger.info(f"PD gains switched: {self._gains_mode} → {mode}")
        self._gains_mode = mode
        return True

    def step_sim(self):
        """Step the simulation with policy or zero actions."""
        branch = None
        if self._safety_mode == "ESTOP":
            # In ESTOP: hold standing pose with high-stiffness PD
            self._switch_gains("stand")
            if self.env:
                self.env.zero_velocity()
                actions = self.env.compute_stand_actions(0, 0, 0, 0)
            else:
                actions = torch.zeros(1, 12, device=gs.device)
            branch = "ESTOP"
        elif self._gait_enabled and self.policy is not None and self.current_obs is not None:
            # L2 held: gait walking via policy (training kp)
            self._switch_gains("walk")
            with torch.no_grad():
                actions = self.policy.act_inference(self.current_obs)
            branch = "WALK"
        else:
            # L2 released: IK-style body pose with high-stiffness PD
            self._switch_gains("stand")
            if self.env:
                pitch, roll, yaw, height = self._stand_axes
                actions = self.env.compute_stand_actions(pitch, roll, yaw, height)
            else:
                actions = torch.zeros(1, 12, device=gs.device)
            branch = "STAND"

        # Log every 50 steps (~1s at 50Hz)
        self._step_log_counter += 1
        if self._step_log_counter % 50 == 1:
            act_abs = actions.abs().mean().item()
            act_max = actions.abs().max().item()
            obs_shape = self.current_obs.shape if self.current_obs is not None else None
            logger.info(
                f"step_sim: branch={branch} safety={self._safety_mode} "
                f"gains={self._gains_mode} gait={self._gait_enabled} "
                f"policy={'YES' if self.policy else 'NO'} "
                f"obs={obs_shape} act_mean={act_abs:.4f} act_max={act_max:.4f} "
                f"stand_axes={[round(x, 3) for x in self._stand_axes]}"
            )

        obs, _, dones, _, _ = self.env.step(actions)
        self.current_obs = obs

        if dones.any():
            logger.warning("Episode terminated — resetting environment")
            obs, _ = self.env.reset()
            self.current_obs = obs

    # ── NATS ──────────────────────────────────────────────────────

    async def connect_nats(self):
        """Connect to NATS for command/telemetry."""
        import nats as nats_client

        nats_url = os.environ.get("SDR_NATS_URL", "nats://localhost:4222")

        async def on_disconnect():
            logger.warning("NATS disconnected")

        async def on_reconnect():
            logger.info("NATS reconnected")

        async def on_error(e):
            logger.error(f"NATS error: {e}")

        self.nc = await nats_client.connect(
            nats_url,
            disconnected_cb=on_disconnect,
            reconnected_cb=on_reconnect,
            error_cb=on_error,
        )
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
                status = "ok"
                detail = None
                # Out-of-order protection for velocity commands
                if action == "set_cmd_vel":
                    # Out-of-order protection: drop stale commands, but
                    # accept a backwards jump (new browser session reset)
                    if cmd_seq <= self._last_cmd_seq:
                        if cmd_seq > self._last_cmd_seq - 100:
                            continue  # genuinely out-of-order within same session
                        # Large backwards jump → new browser session, accept it
                        logger.info(f"cmd_vel seq reset detected: {self._last_cmd_seq} → {cmd_seq}")
                    self._last_cmd_seq = cmd_seq
                    self._last_cmd_vel_time = time.monotonic()
                    self._cmd_vel_received = True
                    self._gait_enabled = bool(cmd_data.get("gait_enabled", False))
                    self._stand_axes = [
                        cmd_data.get("linear_y", 0.0),    # pitch
                        cmd_data.get("linear_x", 0.0),    # roll
                        cmd_data.get("angular_z", 0.0),   # yaw
                        cmd_data.get("angular_y", 0.0),   # height
                    ]
                    # Log every 30th cmd_vel (~1/sec at 30Hz send rate)
                    self._cmd_log_counter += 1
                    if self._cmd_log_counter % 30 == 1:
                        logger.info(
                            f"cmd_vel: seq={cmd_seq} gait={self._gait_enabled} "
                            f"safety={self._safety_mode} "
                            f"lx={cmd_data.get('linear_x', 0):.3f} "
                            f"ly={cmd_data.get('linear_y', 0):.3f} "
                            f"az={cmd_data.get('angular_z', 0):.3f} "
                            f"ay={cmd_data.get('angular_y', 0):.3f}"
                        )
                    # Recover from HOLD or cmd_timeout ESTOP on fresh command
                    if self._safety_mode in ("HOLD", "ESTOP") and self._safety_reason == "cmd_timeout":
                        logger.info(f"Auto-recovering from {self._safety_mode} on fresh cmd_vel")
                        self._safety_mode = "ARMED"
                        self._safety_reason = "ok"
                    if self.env:
                        self.env.set_velocity_from_gamepad(cmd_data)
                    continue

                try:
                    if action == "pause":
                        self.paused = cmd_data.get("paused", True)
                    elif action == "reset":
                        if self.env:
                            obs, _ = self.env.reset()
                            self.current_obs = obs
                    elif action == "estop":
                        self._safety_mode = "ESTOP"
                        self._safety_reason = cmd_data.get("reason", "operator")
                        if self.env:
                            self.env.zero_velocity()
                        logger.warning(f"ESTOP triggered: {self._safety_reason}")
                    elif action == "estop_clear":
                        if not self._video_gate_active:
                            self._safety_mode = "ARMED"
                            self._safety_reason = "operator_clear"
                            self._last_cmd_vel_time = time.monotonic()
                            logger.info("ESTOP cleared by operator")
                        else:
                            logger.warning("ESTOP clear rejected — video gate still active")
                    elif action == "list_policies":
                        policies = self._scan_checkpoints()
                        await self.nc.publish(
                            "telemetry.policy.list",
                            json.dumps({"policies": policies}).encode(),
                        )
                    elif action == "load_policy":
                        checkpoint_dir = cmd_data.get("checkpoint_dir", "")
                        model_file = cmd_data.get("model_file")
                        if not checkpoint_dir or not os.path.exists(checkpoint_dir):
                            raise FileNotFoundError(f"Checkpoint dir not found: {checkpoint_dir}")
                        if model_file:
                            model_path = os.path.join(checkpoint_dir, model_file)
                            if not os.path.exists(model_path):
                                raise FileNotFoundError(f"Model file not found: {model_path}")
                        obs_dim = self.current_obs.shape[-1] if self.current_obs is not None else 310
                        self.policy = load_policy(checkpoint_dir, model_file=model_file, obs_dim=obs_dim)
                        if not self.policy:
                            raise RuntimeError("Policy load returned None")
                        self.checkpoint_dir = checkpoint_dir
                        self._loaded_model_file = model_file or os.path.basename(
                            sorted(glob.glob(os.path.join(checkpoint_dir, "model_*.pt")))[-1]
                        )
                    elif action == "load_robot":
                        pass  # TODO: implement robot loading
                    elif action == "set_mode":
                        pass  # TODO: implement mode switching
                    elif action == "camera":
                        pass  # TODO: implement camera control
                    elif action == "force_idr":
                        self._force_idr = True
                        logger.info("IDR frame requested")
                    elif action == "settings":
                        if "dt" in cmd_data:
                            new_dt = float(cmd_data["dt"])
                            if self.env and 0.001 <= new_dt <= 0.1:
                                self.env.scene.sim_options.dt = new_dt
                                logger.info(f"Simulation dt updated to {new_dt}")
                        if "jpeg_quality" in cmd_data:
                            self.jpeg_quality = int(cmd_data["jpeg_quality"])
                            if isinstance(self.encoder, JpegEncoder):
                                self.encoder.quality = self.jpeg_quality
                        if "stream_fps" in cmd_data:
                            self.target_fps = int(cmd_data["stream_fps"])
                        # Update H.264 params BEFORE codec switch so _recreate_encoder uses new values
                        h264_params_changed = False
                        if "h264_bitrate" in cmd_data:
                            new_bitrate = int(float(cmd_data["h264_bitrate"]) * 1_000_000)
                            if new_bitrate != self.h264_bitrate:
                                self.h264_bitrate = new_bitrate
                                h264_params_changed = True
                        if "h264_preset" in cmd_data:
                            new_preset = str(cmd_data["h264_preset"])
                            if new_preset != self.h264_preset:
                                self.h264_preset = new_preset
                                h264_params_changed = True
                        # Codec switch: h264 ↔ jpeg
                        if "codec" in cmd_data:
                            requested = cmd_data["codec"]
                            if requested == "jpeg" and isinstance(self.encoder, NvencEncoder):
                                logger.info("Switching encoder to JPEG")
                                new_enc = JpegEncoder(quality=self.jpeg_quality)
                                self.encoder = new_enc
                                if self._encoder_thread:
                                    self._encoder_thread.swap_encoder(new_enc)
                            elif requested == "h264" and isinstance(self.encoder, JpegEncoder):
                                logger.info("Switching encoder to H.264 (NVENC)")
                                self._recreate_encoder()
                                h264_params_changed = False  # Already created with new params
                        # Recreate NVENC only if params changed without a codec switch
                        if h264_params_changed and isinstance(self.encoder, NvencEncoder):
                            self._recreate_encoder()
                except Exception as e:
                    status = "error"
                    detail = str(e)
                    logger.error(f"Command handler error ({action}): {e}")

                # Publish ack
                ack = {"action": action, "cmd_seq": cmd_seq, "status": status}
                if detail:
                    ack["detail"] = detail
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
                    if self.env:
                        self.env.zero_velocity()
            except Exception:
                pass

    # ── Safety (Layer 3) ──────────────────────────────────────────

    def _enforce_cmd_ttl(self):
        """Layer 3: TTL decay and ESTOP on command timeout."""
        if self._safety_mode == "ESTOP":
            return
        if not self._cmd_vel_received:
            return  # Don't enforce TTL until first command arrives

        elapsed = time.monotonic() - self._last_cmd_vel_time
        if elapsed > 2.0:
            self._safety_mode = "ESTOP"
            self._safety_reason = "cmd_timeout"
            self._last_cmd_seq = 0  # Reset so fresh browser sessions are accepted
            if self.env:
                self.env.zero_velocity()
            logger.warning("Command timeout >2s — ESTOP")
        elif elapsed > 0.2:
            if self._safety_mode != "HOLD":
                self._safety_mode = "HOLD"
                self._safety_reason = "cmd_timeout"
                if self.env:
                    self.env.zero_velocity()
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
        self._start_encoder_thread()

        # Reset cmd timer after init (GPU init takes seconds, would trigger ESTOP)
        self._last_cmd_vel_time = time.monotonic()

        logger.info(
            f"Starting sim loop at {self.target_fps} FPS (threaded encoder) — "
            f"policy={'LOADED' if self.policy else 'NONE'}, "
            f"safety={self._safety_mode}, gait={self._gait_enabled}, "
            f"obs={'OK' if self.current_obs is not None else 'NONE'}"
        )

        try:
            while self.running:
                t0 = time.monotonic()

                # Enforce command TTL (Layer 3)
                self._enforce_cmd_ttl()

                # Step physics (unless paused)
                if not self.paused:
                    self.step_sim()
                step_count += 1

                # Render and submit to encoder thread
                self.render_and_enqueue()

                now = time.monotonic()

                # Publish telemetry every ~1s
                if now - last_metrics_time > 1.0 and self.nc and self.nc.is_connected:
                    metrics = {
                        "step": step_count,
                        "fps": self.target_fps,
                        "policy_loaded": self.policy is not None,
                        "policy_checkpoint": getattr(self, '_loaded_model_file', None),
                        "paused": self.paused,
                        "dt": self.env.dt if self.env else 0.02,
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

                # Publish encoder stats every ~1s (from encoder thread)
                if now - self._last_encode_stats_time > 1.0 and self.nc and self.nc.is_connected:
                    if self._encoder_thread:
                        times, sizes, total = self._encoder_thread.snapshot_stats()
                        if times:
                            is_h264 = isinstance(self.encoder, NvencEncoder)
                            enc_stats = {
                                "encode_time_avg_ms": round(sum(times) / len(times), 2),
                                "encode_time_max_ms": round(max(times), 2),
                                "frame_size_avg_bytes": int(sum(sizes) / len(sizes)) if sizes else 0,
                                "frame_size_max_bytes": max(sizes) if sizes else 0,
                                "actual_fps": len(times),
                                "target_fps": self.target_fps,
                                "codec": "h264" if is_h264 else "jpeg",
                                "bitrate": self.h264_bitrate,
                                "frames_encoded": total,
                                "intra_refresh": is_h264 and getattr(self.encoder, '_intra_refresh', False),
                            }
                            await self.nc.publish("telemetry.encoder.stats", json.dumps(enc_stats).encode())
                    self._last_encode_stats_time = now

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

                # Recompute frame interval each iteration (respect runtime FPS changes)
                frame_interval = 1.0 / self.target_fps

                # Frame pacing — minimum 1ms yield so NATS keepalives are processed
                elapsed = time.monotonic() - t0
                sleep_time = frame_interval - elapsed
                await asyncio.sleep(max(sleep_time, 0.001))

        except KeyboardInterrupt:
            logger.info("Interrupted")
        finally:
            self.running = False
            self._stop_encoder_thread()
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
    parser.add_argument("--gpu", type=str, default=None,
                        help="GPU device index (e.g. 0, 1). Overrides SDR_GPU_ID env var.")
    args = parser.parse_args()

    # CLI --gpu overrides SDR_GPU_ID env var
    if args.gpu is not None:
        os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu
        os.environ["TI_VISIBLE_DEVICE"] = args.gpu
        os.environ["EGL_DEVICE_ID"] = args.gpu

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
