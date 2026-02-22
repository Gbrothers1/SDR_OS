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

# Steam Deck AMD APU (gfx1033, Van Gogh RDNA2) — AMDGPU backend setup:
#   Uses gs.cuda backend remapped to ti.amdgpu via TI_ARCH monkey-patch.
#   This gives: AMDGPU physics kernels + HIP tensors on GPU (gs.device=cuda:0).
#   HIP_LAUNCH_BLOCKING=1: Required — gstaichi AMDGPU has async dispatch race where
#     SNode init isn't flushed before user kernels (nil pointer crash without it).
#   HSA_ENABLE_SDMA=0: Disables DMA engine to prevent hipMemcpy deadlock when HIP and
#     AMDGPU coexist on the APU. Set in process-compose.yml.
#   GS_ENABLE_ZEROCOPY: Default OFF because gstaichi DLPack only whitelists
#     CPU/Metal/CUDA. Enabled at runtime by _init_genesis_amdgpu() after applying
#     binary patches (src/sdr_os/amdgpu_dlpack_patch.py).
os.environ.setdefault("TORCHDYNAMO_DISABLE", "1")
os.environ.setdefault("GS_ENABLE_ZEROCOPY", "0")

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

# EGL for headless offscreen rendering; viewer mode uses GLX for windowed context
if "--viewer" not in sys.argv:
    os.environ.setdefault("PYOPENGL_PLATFORM", "egl")
else:
    os.environ["PYOPENGL_PLATFORM"] = "x11"

# ── GPU device selection (must be set BEFORE importing genesis) ──
# Force PCI bus ordering so CUDA indices match nvidia-smi output.
os.environ.setdefault("CUDA_DEVICE_ORDER", "PCI_BUS_ID")
_gpu_id = os.environ.get("SDR_GPU_ID", "")
if _gpu_id:
    os.environ["CUDA_VISIBLE_DEVICES"] = _gpu_id
    os.environ["TI_VISIBLE_DEVICE"] = _gpu_id
    os.environ["EGL_DEVICE_ID"] = _gpu_id

# ── AMDGPU setup (must happen BEFORE importing gstaichi/genesis) ──
if os.environ.get("HSA_OVERRIDE_GFX_VERSION"):
    os.environ.setdefault("HSA_ENABLE_SDMA", "0")

_is_amd_gpu = False
try:
    import torch as _torch_probe
    if _torch_probe.cuda.is_available():
        _dev_name = _torch_probe.cuda.get_device_name(0)
        _is_amd_gpu = "AMD" in _dev_name or "Radeon" in _dev_name
    del _torch_probe
except Exception:
    pass

if _is_amd_gpu:
    os.environ.setdefault("HIP_LAUNCH_BLOCKING", "1")
    _rocm_lld = "/opt/rocm-6.3.0/llvm/bin/ld.lld"
    if os.path.exists(_rocm_lld):
        _lld_dir = "/tmp/lld_only"
        os.makedirs(_lld_dir, exist_ok=True)
        _lld_link = os.path.join(_lld_dir, "ld.lld")
        if not os.path.exists(_lld_link):
            os.symlink(_rocm_lld, _lld_link)
        os.environ["PATH"] = f"{_lld_dir}:{os.environ['PATH']}"

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
from tensordict import TensorDict
from genesis.utils.geom import inv_quat


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


class SoftH264Encoder:
    """Software H.264 encoder (libx264) producing Annex-B NAL units.

    Same interface as NvencEncoder but uses CPU-based libx264.
    Tuned for low-latency streaming (zerolatency, no B-frames).
    """
    def __init__(self, width, height, fps=30, bitrate=5_000_000,
                 gop=30, preset="ultrafast"):
        import av
        from fractions import Fraction

        self.codec = Codec.H264
        self.frame_count = 0
        self._width = width
        self._height = height

        self.ctx = av.CodecContext.create("libx264", "w")
        self.ctx.width = width
        self.ctx.height = height
        self.ctx.pix_fmt = "yuv420p"
        self.ctx.framerate = Fraction(fps, 1)
        self.ctx.time_base = Fraction(1, fps)
        self.ctx.bit_rate = bitrate
        self.ctx.gop_size = gop
        self.ctx.max_b_frames = 0
        self.ctx.options = {
            "preset": preset,
            "tune": "zerolatency",
            "repeat-headers": "1",
        }
        self.ctx.open()

        self._yuv_frame = av.VideoFrame(width, height, "yuv420p")

    def encode(self, frame_rgb, frame_id, force_idr=False, yuv_planes=None):
        import av
        if yuv_planes is not None:
            y, u, v = yuv_planes
            frame = self._yuv_frame
            frame.planes[0].update(y.tobytes())
            frame.planes[1].update(u.tobytes())
            frame.planes[2].update(v.tobytes())
        else:
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
    frame_np: Optional[np.ndarray]  # RGB numpy for JPEG encoding
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

                payload, is_keyframe = encoder.encode(
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
        headless: bool = True,
    ):
        self.target_fps = target_fps
        self.camera_res = camera_res
        self.jpeg_quality = jpeg_quality
        self.checkpoint_dir = checkpoint_dir
        self.headless = headless

        self.env = None
        self.policy = None
        self.current_obs = None
        self.shm_writer = None
        self.encoder = None
        self._encoder_thread: Optional[EncoderThread] = None
        self.frame_id = 0
        self.running = False
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
        self._last_stand_axes = None  # Cache to skip redundant action manager calls
        self._actions_dirty = True    # Force action manager update on first step

    def init_genesis(self):
        """Initialize Genesis scene and Go2 environment.

        On AMD GPUs: remaps gs.cuda → ti.amdgpu so Genesis enables the cuda
        code path (gs.device=cuda:0, HIP tensors on GPU) while Taichi compiles
        AMDGPU kernels for gfx1033.  Applies DLPack binary patches to enable
        zero-copy between gstaichi fields and PyTorch HIP tensors.
        """
        if _is_amd_gpu:
            self._init_genesis_amdgpu()
        else:
            logger.info("Initializing Genesis GPU backend...")
            gs.init(backend=gs.gpu, performance_mode=True)

        # Import after gs.init() — GaitCommandManager touches genesis.engine at import time
        from src.sdr_os.envs.go2_env import Go2BridgeEnv

        logger.info(f"Creating Go2BridgeEnv (camera: {self.camera_res})...")
        self.env = Go2BridgeEnv(
            num_envs=1,
            dt=1 / 50,
            max_episode_length_s=None,
            headless=self.headless,
            camera_res=self.camera_res,
        )
        self.env.build()

        # gs.device is already cuda:0 (HIP) — no separate inference device needed
        self._infer_device = gs.device
        logger.info(
            f"Inference device: {self._infer_device} "
            f"({torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu'})"
        )

        obs, _ = self.env.reset()
        self.current_obs = obs
        logger.info(
            f"Go2BridgeEnv initialized — obs: {obs.shape if obs is not None else None}, "
            f"device: {obs.device if obs is not None else None}, infer: {self._infer_device}"
        )

        # Load policy and move to HIP (if available)
        if self.checkpoint_dir and os.path.exists(self.checkpoint_dir):
            logger.info(f"Loading policy from: {self.checkpoint_dir}")
            obs_dim = obs.shape[-1] if obs is not None else 310
            self.policy = load_policy(self.checkpoint_dir, obs_dim=obs_dim)
            if self.policy:
                if self._infer_device != gs.device:
                    self.policy = self.policy.to(self._infer_device)
                    torch.cuda.synchronize()
                    logger.info(f"Policy moved to {self._infer_device}")
                logger.info(f"Policy loaded — {type(self.policy).__name__}, device={next(self.policy.parameters()).device}")
                try:
                    with torch.no_grad():
                        test_actions = self.policy.act_inference(self.current_obs)
                    logger.info(
                        f"Policy smoke test PASSED — {self.current_obs.shape} → {test_actions.shape}, "
                        f"act_mean={test_actions.abs().mean().item():.4f}"
                    )
                except Exception as e:
                    logger.error(f"Policy smoke test FAILED: {e}")
                    self.policy = None
            else:
                logger.warning("Policy loading returned None")
        else:
            logger.info(f"No checkpoint (checkpoint_dir={self.checkpoint_dir!r})")

        # Initialize encoder (JPEG only — H.264 disabled)
        width, height = self.camera_res
        self.encoder = JpegEncoder(quality=self.jpeg_quality)
        logger.info(f"JPEG encoder initialized ({width}x{height}, quality={self.jpeg_quality})")

    def _init_genesis_amdgpu(self):
        """Initialize Genesis with AMDGPU backend + DLPack zero-copy on AMD APU."""
        import gstaichi as _ti
        from genesis.constants import backend as gs_backend, TI_ARCH

        logger.info("AMD GPU detected — routing gs.cuda → ti.amdgpu")

        # Route gs.cuda → ti.amdgpu so Genesis enables zero-copy path
        # (zero-copy requires backend == gs_backend.cuda && device.type == "cuda")
        TI_ARCH['Linux'][gs_backend.cuda] = _ti.amdgpu

        # Binary-patch gstaichi DLPack for AMDGPU support on APU unified memory
        try:
            from src.sdr_os.amdgpu_dlpack_patch import patch_gstaichi_dlpack
            if patch_gstaichi_dlpack():
                os.environ["GS_ENABLE_ZEROCOPY"] = "1"
                logger.info("DLPack patches applied — enabling zero-copy")
            else:
                logger.warning("DLPack patches failed — zero-copy disabled")
        except ImportError:
            logger.warning("amdgpu_dlpack_patch not found — zero-copy disabled")

        gs.init(backend=gs.cuda, performance_mode=True)
        logger.info(
            f"Genesis AMDGPU init: device={gs.device}, backend={gs.backend}, "
            f"zerocopy={gs.use_zerocopy}"
        )

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



    def render_and_enqueue(self):
        """Render camera frame and submit to encoder thread."""
        # AMDGPU headless: flush Taichi + force camera follow update before
        # pyrender reads the camera pose matrix. Without this, get_pos() may
        # return stale/zero data → singular camera transform → LinAlgError.
        # In viewer mode the viewer thread handles visual state updates, so
        # we only need a sync fence (no manual update_following).
        if _is_amd_gpu:
            import gstaichi as _ti
            _ti.sync()
            if self.headless and self.env.camera._followed_entity is not None:
                self.env.camera.update_following()
        render_result = self.env.camera.render(rgb=True, depth=False, segmentation=False, normal=False)
        img = render_result[0] if isinstance(render_result, tuple) else render_result

        if isinstance(img, torch.Tensor):
            if img.ndim == 4:
                img = img[0]
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

    # Policy runs every WALK_POLICY_INTERVAL physics steps in WALK mode.
    # Each policy step costs ~45ms (GPU readbacks) while physics alone costs ~5ms.
    # With interval=4: avg = (45 + 5*3)/4 = 15ms → 66 FPS.
    # PD control maintains joint targets between policy updates.
    WALK_POLICY_INTERVAL = 4

    def step_sim(self):
        """Step the simulation with policy or zero actions.

        In viewer mode, decouples physics from the visualizer: skips
        visualizer sync on most steps (letting the viewer thread render
        independently at max_FPS) and only syncs every Nth step.
        """
        branch = None
        if self._safety_mode == "ESTOP":
            self._switch_gains("stand")
            if self.env:
                self.env.zero_velocity()
                actions = self.env.compute_stand_actions(0, 0, 0, 0)
            else:
                actions = torch.zeros(1, 12, device=gs.device)
            # Only update actions when entering ESTOP (axes become [0,0,0,0])
            if self._last_stand_axes != [0, 0, 0, 0]:
                self._actions_dirty = True
                self._last_stand_axes = [0, 0, 0, 0]
            branch = "ESTOP"
        elif self._gait_enabled and self.policy is not None and self.current_obs is not None:
            self._switch_gains("walk")
            # Policy decimation: only run inference + obs readback every
            # Nth step.  Between updates, PD controllers hold joint targets.
            # At 50Hz physics with N=4, policy runs at 12.5Hz — well within
            # the Go2 gait policy's stability margin.
            self._policy_update = (self._step_log_counter % self.WALK_POLICY_INTERVAL == 0)
            if self._policy_update:
                with torch.no_grad():
                    actions = self.policy.act_inference(self.current_obs)
                # Policy runs on _infer_device (HIP); Genesis needs CPU tensors
                if actions.device != gs.device:
                    actions = actions.to(gs.device)
                self._actions_dirty = True
            else:
                # Reuse last actions — PD controller maintains targets
                actions = self.env._actions if self.env._actions is not None else torch.zeros(1, 12, device=gs.device)
            branch = "WALK"
        else:
            self._switch_gains("stand")
            if self.env:
                pitch, roll, yaw, height = self._stand_axes
                actions = self.env.compute_stand_actions(pitch, roll, yaw, height)
            else:
                actions = torch.zeros(1, 12, device=gs.device)
            # Only update when stick axes change
            if self._last_stand_axes != self._stand_axes:
                self._actions_dirty = True
                self._last_stand_axes = list(self._stand_axes)
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

        # In viewer mode, skip the visualizer sync on most steps so the
        # viewer thread renders independently at 60 FPS.  Physics runs
        # as fast as it can; we sync the visualizer every 3rd step so it
        # picks up the latest state.
        if not self.headless:
            # Replicate GenesisEnv.step() bookkeeping — clear extras so
            # get_observations() computes fresh obs instead of returning cache.
            self.env._extras = {}
            self.env._extras[self.env.extras_logging_key] = {}
            self.env.extras["observations"] = TensorDict({}, device=gs.device)
            self.env.step_count += 1
            self.env.episode_length += 1
            if self.env._actions is None:
                self.env._actions = actions.detach().clone()
                self.env._last_actions = torch.zeros_like(actions, device=gs.device)
            else:
                self.env._last_actions[:] = self.env._actions[:]
                self.env._actions[:] = actions[:]
            # Apply actions — skip when PD targets unchanged (static stand pose).
            # The physics solver continues applying PD control with the last target.
            _t_act = time.monotonic()
            if self._actions_dirty:
                if self.env.managers["action"] is not None:
                    self.env.managers["action"].step(actions)
                self._actions_dirty = False
            _act_ms = (time.monotonic() - _t_act) * 1000
            # Physics — sync visualizer every Nth step.  The viewer thread
            # renders at max_FPS independently; we push state updates at a
            # lower rate.  Each sync costs ~25ms (lock contention) so we
            # offset by 2 to avoid overlapping with policy readback steps
            # (which fire at step % INTERVAL == 0).
            _viz_period = self.WALK_POLICY_INTERVAL * 3  # every 12th step
            do_viz = (self._step_log_counter % _viz_period == 2)
            _t_phys = time.monotonic()
            self.env.scene.step(update_visualizer=do_viz)
            _phys_ms = (time.monotonic() - _t_phys) * 1000
            # Entity + command + obs managers — only run when needed.
            # WALK: every Nth step to amortize GPU→CPU readback cost.
            #   The readback (mgr ~13ms + obs ~30ms) dominates step time.
            #   PD controllers hold joint targets between policy updates,
            #   so skipping obs on intermediate steps is safe.
            # STAND/ESTOP: never (no state needed for static pose).
            need_full_state = (branch == "WALK" and
                               getattr(self, '_policy_update', False))
            _t_mgr = time.monotonic()
            if need_full_state:
                # Flush all pending Vulkan compute before CPU reads —
                # single fence wait instead of per-field stalls
                import gstaichi as _ti
                _ti.sync()
                # Minimal entity update: only quat → inv_quat (skip pos read)
                robot_mgr = self.env.robot_manager
                quat = self.env.robot.get_quat()
                robot_mgr._base_quat[:] = quat
                robot_mgr._inv_base_quat = inv_quat(quat)
                for cmd in self.env.managers["command"]:
                    cmd.step()
            _mgr_ms = (time.monotonic() - _t_mgr) * 1000
            # Observations: only on policy-update steps
            _t_obs = time.monotonic()
            if need_full_state:
                obs = self.env.get_observations()
                # Move obs to inference device (HIP) for next policy step.
                # Dual fence required: Vulkan (Taichi) + HIP must both be
                # quiesced before crossing GPU compute domains on gfx1033.
                if obs is not None and obs.device != self._infer_device:
                    torch.cuda.synchronize()
                    obs = obs.to(self._infer_device)
            else:
                obs = self.current_obs
            _obs_ms = (time.monotonic() - _t_obs) * 1000
            # Termination check every 50 steps in ALL modes (STAND/WALK/ESTOP).
            # Fall detection doesn't need per-step precision, and entity reads
            # are expensive.  In viewer mode we bypass env.step() so there is
            # no auto-reset — we must detect and handle it ourselves.
            dones = self.env._terminated_buf
            if self._step_log_counter % 50 == 0:
                import gstaichi as _ti
                _ti.sync()
                for em in self.env.managers["entity"]:
                    em.step()
                if self.env.managers["termination"] is not None:
                    dones, _ = self.env.managers["termination"].step()
            # Record sub-timings for periodic logging
            if not hasattr(self, '_phys_times'):
                self._phys_times = []
                self._obs_times = []
                self._act_times = []
                self._mgr_times = []
            self._phys_times.append(_phys_ms)
            self._obs_times.append(_obs_ms)
            self._act_times.append(_act_ms)
            self._mgr_times.append(_mgr_ms)
        else:
            obs, _, dones, _, _ = self.env.step(actions)
            # Dual fence: Vulkan + HIP must be quiesced before cross-domain transfer
            if obs is not None and obs.device != self._infer_device:
                import gstaichi as _ti
                _ti.sync()
                torch.cuda.synchronize()
                obs = obs.to(self._infer_device)

        self.current_obs = obs

        if dones is not None and dones.any():
            if self.headless:
                # Headless: ManagedEnvironment.step() already resets internally.
                logger.warning("Episode terminated (bad_orientation) — env auto-reset")
            else:
                # Viewer mode: we bypass env.step(), so reset manually.
                logger.warning("Episode terminated (bad_orientation) — manual reset")
                reset_obs, _ = self.env.reset()
                if reset_obs is not None:
                    if reset_obs.device != self._infer_device:
                        import gstaichi as _ti
                        _ti.sync()
                        torch.cuda.synchronize()
                        reset_obs = reset_obs.to(self._infer_device)
                    self.current_obs = reset_obs
                self._actions_dirty = True

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
                            if obs is not None and obs.device != self._infer_device:
                                import gstaichi as _ti
                                _ti.sync()
                                torch.cuda.synchronize()
                                obs = obs.to(self._infer_device)
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
                        if self._infer_device != gs.device:
                            self.policy = self.policy.to(self._infer_device)
                            torch.cuda.synchronize()
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
        last_metrics_step = 0
        last_safety_time = 0
        last_timing_log = 0
        next_frame_time = 0  # absolute timeline for catch-up pacing
        step_times = []
        loop_times = []

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
                    t_step = time.monotonic()
                    self.step_sim()
                    step_ms = (time.monotonic() - t_step) * 1000
                    step_times.append(step_ms)
                step_count += 1

                # Render and submit to encoder thread for web UI stream.
                # In viewer mode, the Genesis viewer thread already renders
                # the 3D scene.  We do a separate offscreen render for the
                # web UI stream but only every 3rd step (~20 FPS) to avoid
                # doubling the GPU rendering load.
                if self.headless or step_count % 3 == 0:
                    try:
                        self.render_and_enqueue()
                    except Exception as e:
                        if step_count % 100 == 0:
                            logger.warning(f"render_and_enqueue failed: {e}")

                now = time.monotonic()

                # Log timing every ~5s
                loop_ms = (now - t0) * 1000
                loop_times.append(loop_ms)
                if now - last_timing_log > 5.0 and step_times:
                    avg_step = sum(step_times) / len(step_times)
                    max_step = max(step_times)
                    avg_loop = sum(loop_times) / len(loop_times)
                    actual_fps = len(loop_times) / (now - last_timing_log) if last_timing_log > 0 else 0
                    # Sub-component timings (viewer mode only)
                    phys_str = ""
                    if hasattr(self, '_phys_times') and self._phys_times:
                        avg_phys = sum(self._phys_times) / len(self._phys_times)
                        avg_obs = sum(self._obs_times) / len(self._obs_times)
                        avg_act = sum(self._act_times) / len(self._act_times) if hasattr(self, '_act_times') and self._act_times else 0
                        avg_mgr = sum(self._mgr_times) / len(self._mgr_times) if hasattr(self, '_mgr_times') and self._mgr_times else 0
                        phys_str = f" act={avg_act:.1f}ms phys={avg_phys:.1f}ms mgr={avg_mgr:.1f}ms obs={avg_obs:.1f}ms"
                        self._phys_times.clear()
                        self._obs_times.clear()
                        if hasattr(self, '_act_times'):
                            self._act_times.clear()
                        if hasattr(self, '_mgr_times'):
                            self._mgr_times.clear()
                    logger.info(
                        f"TIMING: step_avg={avg_step:.1f}ms step_max={max_step:.1f}ms "
                        f"loop_avg={avg_loop:.1f}ms actual_fps={actual_fps:.1f}"
                        f"{phys_str} "
                        f"({len(step_times)} steps in {now - last_timing_log:.1f}s)"
                    )
                    step_times.clear()
                    loop_times.clear()
                    last_timing_log = now

                # Publish telemetry every ~1s
                if now - last_metrics_time > 1.0 and self.nc and self.nc.is_connected:
                    # Compute actual FPS from step count delta (independent of timing log clears)
                    elapsed_since_metrics = now - last_metrics_time if last_metrics_time > 0 else 1.0
                    steps_since_metrics = step_count - last_metrics_step
                    measured_fps = steps_since_metrics / elapsed_since_metrics if elapsed_since_metrics > 0 else 0
                    avg_step_ms = sum(step_times) / len(step_times) if step_times else 0
                    metrics = {
                        "step": step_count,
                        "fps": self.target_fps,
                        "actual_fps": round(measured_fps, 1),
                        "step_ms": round(avg_step_ms, 2),
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
                    last_metrics_step = step_count

                # Publish encoder stats every ~1s (from encoder thread)
                if self.headless and now - self._last_encode_stats_time > 1.0 and self.nc and self.nc.is_connected:
                    if self._encoder_thread:
                        times, sizes, total = self._encoder_thread.snapshot_stats()
                        if times:
                            enc_stats = {
                                "encode_time_avg_ms": round(sum(times) / len(times), 2),
                                "encode_time_max_ms": round(max(times), 2),
                                "frame_size_avg_bytes": int(sum(sizes) / len(sizes)) if sizes else 0,
                                "frame_size_max_bytes": max(sizes) if sizes else 0,
                                "actual_fps": len(times),
                                "target_fps": self.target_fps,
                                "codec": "jpeg",
                                "frames_encoded": total,
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

                if self.headless:
                    # Frame pacing — minimum 1ms yield so NATS keepalives are processed
                    elapsed = time.monotonic() - t0
                    sleep_time = frame_interval - elapsed
                    await asyncio.sleep(max(sleep_time, 0.001))
                else:
                    # Viewer mode — absolute-timeline catch-up pacing.
                    # After a spike (e.g. 25ms physics), the next frames run
                    # back-to-back until we're caught up, averaging 60 FPS.
                    frame_interval = 1.0 / self.target_fps
                    if next_frame_time == 0:
                        next_frame_time = t0 + frame_interval
                    else:
                        next_frame_time += frame_interval
                    # Don't accumulate more than 3 frames of debt
                    now = time.monotonic()
                    if next_frame_time < now - frame_interval * 3:
                        next_frame_time = now
                    remaining = next_frame_time - now
                    # Use asyncio.sleep for bulk wait — this yields to the
                    # event loop so NATS can process incoming commands and
                    # publish telemetry every frame.  asyncio.sleep has ~1-3ms
                    # jitter; leave 3ms buffer for busy-wait correction.
                    if remaining > 0.004:
                        await asyncio.sleep(remaining - 0.003)
                    elif remaining <= 0:
                        # Behind schedule (catch-up) — still yield briefly
                        # so NATS doesn't starve during burst recovery.
                        await asyncio.sleep(0)
                    # Busy-wait the last few ms for precise timing
                    while time.monotonic() < next_frame_time:
                        pass

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
    parser.add_argument("--viewer", action="store_true",
                        help="Open Genesis viewer on DISPLAY (skip SHM encode pipeline)")
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
        headless=not args.viewer,
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
