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
import re
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

# ── Raw command control (MCP/UI arbitration + direct joint mode) ──
from src.sdr_os.control.cmd_arbiter import CmdVelArbiter
from src.sdr_os.control.joint_command import (
    merge_joint_targets,
    targets_to_actions,
)

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


def _resolve_model_path(checkpoint_dir: str, model_file: str | None = None) -> str | None:
    if not checkpoint_dir:
        return None
    if os.path.isfile(checkpoint_dir):
        return checkpoint_dir
    if model_file:
        candidate = model_file if os.path.isabs(model_file) else os.path.join(checkpoint_dir, model_file)
        return candidate if os.path.exists(candidate) else None
    # Numeric sort — lexicographic picks model_900 over model_1600 (the known
    # get_latest_model bug class).
    def _step_num(p):
        m = re.search(r"model_(\d+)", os.path.basename(p))
        return int(m.group(1)) if m else 0

    model_files = sorted(glob.glob(os.path.join(checkpoint_dir, "model_*.pt")), key=_step_num)
    if not model_files:
        return None
    return model_files[-1]


def _extract_actor_state_dict(checkpoint: dict) -> dict:
    if isinstance(checkpoint, dict) and "actor_state_dict" in checkpoint:
        state = checkpoint["actor_state_dict"]
    elif isinstance(checkpoint, dict):
        state = checkpoint.get("model_state_dict", checkpoint)
    else:
        state = checkpoint
    if not isinstance(state, dict):
        return {}
    if any(k.startswith("actor.") for k in state.keys()):
        return {k: v for k, v in state.items() if k.startswith("actor.") or k in ("std", "log_std")}
    # rsl-rl 3.x MLPModel actors store "mlp.*" weights with either "std"
    # (noise_std_type=scalar, e.g. v44/v45 checkpoints) or "log_std".
    if any(k.startswith("mlp.") for k in state.keys()) or "log_std" in state:
        return {k: v for k, v in state.items() if k.startswith("mlp.") or k in ("log_std", "std")}
    return state


def _normalize_actor_state(actor_state: dict) -> dict:
    if any(k.startswith("mlp.") for k in actor_state.keys()):
        mapped = {}
        for k, v in actor_state.items():
            if k.startswith("mlp."):
                mapped[f"actor.{k[len('mlp.'):]}"] = v
            elif k == "log_std":
                mapped["std"] = v.exp()
            elif k == "std":
                mapped["std"] = v
        return mapped
    if "std" not in actor_state and "log_std" in actor_state:
        mapped = dict(actor_state)
        mapped["std"] = actor_state["log_std"].exp()
        return mapped
    return actor_state


def _infer_obs_dim_from_state(actor_state: dict) -> int | None:
    for key in ("actor.0.weight", "mlp.0.weight"):
        if key in actor_state and getattr(actor_state[key], "ndim", 0) == 2:
            return int(actor_state[key].shape[1])
    for _, value in actor_state.items():
        if getattr(value, "ndim", 0) == 2:
            return int(value.shape[1])
    return None


def infer_policy_obs_dim(checkpoint_dir: str, model_file: str | None = None) -> int | None:
    model_path = _resolve_model_path(checkpoint_dir, model_file)
    if not model_path or not os.path.exists(model_path):
        return None
    checkpoint = torch.load(model_path, map_location="cpu", weights_only=False)
    actor_state = _extract_actor_state_dict(checkpoint)
    return _infer_obs_dim_from_state(actor_state)


# Skill obs dims (per-frame x 5-frame history) are mutually distinct, so the
# env mode is inferable from the checkpoint's input layer alone:
#   v45 crawl: 45x5=225, v44 launch: 48x5=240, v46 hurdle: 50x5=250,
#   walk: 62/63x5 = 310/315, v48 park: 79x5 = 395.
CRAWL_OBS_DIM = 225
LAUNCH_OBS_DIM = 240
DIRECTED_OBS_DIM = 245
HURDLE_OBS_DIM = 250
PARK_OBS_DIM = 395


def _env_mode_for_obs_dim(obs_dim: int | None) -> str:
    """Infer which bridge env a checkpoint was trained against."""
    if obs_dim == CRAWL_OBS_DIM:
        return "crawl"
    if obs_dim == LAUNCH_OBS_DIM:
        return "launch"
    if obs_dim == DIRECTED_OBS_DIM:
        return "directed"
    if obs_dim == HURDLE_OBS_DIM:
        return "hurdle"
    if obs_dim == PARK_OBS_DIM:
        return "park"
    return "walk"


def load_policy(checkpoint_dir: str, model_file: str | None = None, obs_dim: int | None = None):
    """Load a trained policy from checkpoint directory or .pt file (old or new format)."""
    model_path = _resolve_model_path(checkpoint_dir, model_file)
    if not model_path:
        logger.error(f"No model files found in {checkpoint_dir}")
        return None

    logger.info(f"Loading policy from {model_path}")

    cfg_path = os.path.join(os.path.dirname(model_path), "cfgs.pkl")
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

    checkpoint = torch.load(model_path, map_location="cpu", weights_only=False)
    actor_state_raw = _extract_actor_state_dict(checkpoint)
    inferred_obs_dim = _infer_obs_dim_from_state(actor_state_raw)
    if obs_dim is None:
        obs_dim = inferred_obs_dim or 310
    actor_state = _normalize_actor_state(actor_state_raw)

    policy = ActorMLP(obs_dim, num_actions, hidden_dims, activation).to(gs.device)
    policy.load_state_dict(actor_state, strict=True)
    policy.eval()

    logger.info(
        f"Policy loaded: {os.path.basename(model_path)} "
        f"(obs_dim={obs_dim}, params={sum(p.numel() for p in policy.parameters())})"
    )
    return policy


# ── Policy library metadata (multi-root scan, skill/version/validation tags) ──

# Skill is inferred from the run-name family. Authoritative skill is the policy
# obs dim (crawl=225, launch=240, walk=310/315) but loading 100s of checkpoints
# to read it would make every list_policies call slow, and this project's run
# names map cleanly onto skills. obs_dim enrichment is added only for the
# already-loaded policy (free).
_SKILL_OBS_DIM = {225: "crawl", 240: "launch", 310: "walk", 315: "walk", 395: "park"}


def _derive_skill(name: str, obs_dim: int | None = None) -> str:
    """Classify a policy's skill from its obs dim (authoritative) or run name."""
    if obs_dim is not None and obs_dim in _SKILL_OBS_DIM:
        return _SKILL_OBS_DIM[obs_dim]
    n = name.lower()
    if "crawl" in n:
        return "crawl"
    if "launch" in n:
        return "launch"
    if "jump" in n or "walk" in n or "tracking" in n:
        # Older jump-power / terrain-jump / tracking lines are walk-obs policies.
        return "walk"
    if n.startswith("bc") or "warmstart" in n:
        return "bc"
    return "unknown"


def _parse_version(name: str) -> dict:
    """Extract version (vNN.N.N), major family (vNN), and run kind from a name."""
    version = None
    family = None
    m = re.search(r"v(\d+(?:\.\d+)*)", name)
    if m:
        version = "v" + m.group(1)
        family = "v" + m.group(1).split(".")[0]
    kind = "other"
    n = name.lower()
    if "smoke" in n:
        kind = "smoke"
    elif "long" in n:
        kind = "long"
    elif n.endswith("-g9") or "-g9" in n:
        kind = "long"
    return {"version": version, "version_family": family, "run_kind": kind}


# Registry statuses/visuals that mean "an evaluation actually ran" (vs a run
# that was merely launched/planned and never assessed).
_EVAL_STATUSES = {"eval_passed", "smoke_passed", "smoke_failed", "smoke_partial"}
_EVAL_VISUALS = {"passed", "failed", "exploit"}


def _load_validation_map(project_root: str) -> dict:
    """Build {run_id: status-object} from artifacts/run_registry.jsonl.

    Each object carries:
      - status: 'validated' (eval_passed or visual passed) > 'smoke_pass' >
                'failed' (smoke_failed / visual failed / killed) > 'untested'
      - evaluated: whether ANY eval/smoke verdict exists for the run (the
        "evaluated vs not-yet-evaluated" axis the library filters on)
      - eval_checkpoint / grid_pass: the specific model_*.pt that was eval'd
      - checkpoint_evals: {model_file: {status, grid_pass, visual}} so the
        checkpoint picker can mark individual checkpoints
    """
    path = os.path.join(project_root, "artifacts", "run_registry.jsonl")
    runs: dict[str, dict] = {}
    if not os.path.isfile(path):
        return {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                rid = d.get("run_id")
                if not rid:
                    continue
                r = runs.setdefault(rid, {
                    "statuses": set(), "visual": set(),
                    "eval_checkpoint": None, "grid_pass": None, "checkpoint_evals": {},
                })
                status = d.get("status")
                visual = d.get("visual_status")
                if status:
                    r["statuses"].add(status)
                if visual:
                    r["visual"].add(visual)
                ckpt = d.get("checkpoint") or d.get("model_file")
                if ckpt:
                    ckpt = os.path.basename(ckpt)
                    rec = r["checkpoint_evals"].setdefault(ckpt, {})
                    if status:
                        rec["status"] = status
                    if d.get("grid_pass"):
                        rec["grid_pass"] = d["grid_pass"]
                    if visual and visual != "pending":
                        rec["visual"] = visual
                    if status == "eval_passed" or visual == "passed":
                        r["eval_checkpoint"] = ckpt
                        if d.get("grid_pass"):
                            r["grid_pass"] = d["grid_pass"]
    except OSError:
        return {}

    result = {}
    for rid, r in runs.items():
        st, vis = r["statuses"], r["visual"]
        if "eval_passed" in st or "passed" in vis:
            status = "validated"
        elif "smoke_passed" in st or "smoke_partial" in st:
            # A smoke test ran (fully or partially) — an evaluated result, kept
            # distinct from a validated eval and from an outright failure so the
            # 'Evaluated' filter (validated+smoke+failed) == the evaluated count.
            status = "smoke_pass"
        elif "killed" in st or "smoke_failed" in st or "failed" in vis or "exploit" in vis:
            status = "failed"
        else:
            status = "untested"
        evaluated = bool(st & _EVAL_STATUSES) or bool(vis & _EVAL_VISUALS) or bool(r["checkpoint_evals"])
        result[rid] = {
            "status": status,
            "evaluated": evaluated,
            "visual_passed": "passed" in vis,
            "eval_passed": "eval_passed" in st,
            "eval_checkpoint": r["eval_checkpoint"],
            "grid_pass": r["grid_pass"],
            "checkpoint_evals": r["checkpoint_evals"],
        }
    return result


def parse_policy_roots(project_root: str) -> list[tuple[str, str]]:
    """Parse SDR_POLICY_ROOTS ('label=path,label=path') into [(label, path)].

    Falls back to a single 'workspace' root at <project_root>/rl/checkpoints.
    """
    raw = os.environ.get("SDR_POLICY_ROOTS", "").strip()
    roots: list[tuple[str, str]] = []
    if raw:
        for chunk in raw.split(","):
            chunk = chunk.strip()
            if not chunk:
                continue
            if "=" in chunk:
                label, path = chunk.split("=", 1)
                roots.append((label.strip(), path.strip()))
            else:
                roots.append((os.path.basename(chunk.rstrip("/")) or chunk, chunk))
    if not roots:
        roots.append(("workspace", os.path.join(project_root, "rl", "checkpoints")))
    return roots


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
        self._cmd_arbiter = CmdVelArbiter(operator_grace_s=1.0, owner_ttl_s=0.2)
        # Direct-joint mode: latched 12-dim target list (None = inactive)
        self._joint_targets: list | None = None
        self._last_joint_cmd_time = 0.0
        self._joint_cmd_count = 0
        self._last_cmd_vel_time = time.monotonic()
        self._cmd_vel_received = False  # Don't enforce TTL until first command arrives
        self._video_gate_active = False
        self._gait_enabled = False  # L2 held = gait walking, released = position hold
        # Active gait state for D-pad switching (walk env only). Re-applied
        # EVERY step so the manager's resample timer cannot overwrite it
        # (Gate A-0 quirk #3: one-shot set_fixed_gait drifts within 3-5s).
        self._active_gait = {"name": "trot", "period": 0.45,
                             "clearance": 0.08, "mode": "walk"}
        self._last_perception_pub = 0.0
        self._stand_axes = [0.0, 0.0, 0.0, 0.0]  # pitch, roll, yaw, height
        self.paused = False
        self._step_log_counter = 0  # throttle step_sim logging
        self._cmd_log_counter = 0   # throttle cmd_vel logging
        self._gains_mode = "walk"   # "walk" or "stand" — tracks current PD gains
        self._include_jump_power = False
        self._env_mode = "walk"     # "walk" (Go2BridgeEnv) or "crawl" (Go2CrawlBridgeEnv)


    def _gait_vx_ceiling(self):
        # Operator-gated speed mode (D-pad up/down). Pronk envelope binds in
        # either mode (Gate A-0: fails at vx 1.2 / T 0.35).
        if self._active_gait["name"] == "pronk":
            return 1.0
        return 2.0 if self._active_gait["mode"] == "run" else 1.0

    def _collect_perception(self):
        """Operator HUD truth: telemetry.policy.perception @10Hz."""
        msg = {"obstacles": [], "available": {}, "skill_active": {}}
        env = self.env
        try:
            # Park env uses 'gait_command'; walk/hurdle/crawl use 'gait_command_manager'.
            _has_gcm = (env is not None and (
                (hasattr(env, "gait_command_manager") and env.gait_command_manager is not None)
                or (hasattr(env, "gait_command") and env.gait_command is not None)))
            if _has_gcm:
                msg["gait"] = {"name": self._active_gait["name"],
                               "period": self._active_gait["period"],
                               "mode": self._active_gait["mode"]}
            mode = getattr(self, "_env_mode", "")

            if mode == "park" and env is not None:
                # Park mode: read from the analytic slot table (_slot_x, _slot_low,
                # _slot_high, _slot_active, _slot_kind, _slot_depth).
                # skill_command.command[0] = [jump, crouch, climb].
                try:
                    x = float(env.robot.get_pos()[0, 0])
                    skill_cmd = env.skill_command.command[0].tolist() \
                        if env.skill_command.command is not None else [0, 0, 0]
                    obstacles = []
                    if env._slot_active is not None:
                        slot_x = env._slot_x[0].tolist()
                        slot_low = env._slot_low[0].tolist()
                        slot_high = env._slot_high[0].tolist()
                        slot_active = env._slot_active[0].tolist()
                        slot_kind = env._slot_kind[0].tolist()
                        slot_depth = env._slot_depth[0].tolist()
                        _KIND_NAMES = {0: "hurdle", 1: "crawl", 2: "box", 3: "gate"}
                        ahead_slots = [
                            (slot_x[j] - x, j) for j in range(len(slot_active))
                            if slot_active[j] and (slot_x[j] - x) > -0.1
                               and slot_kind[j] >= 0
                        ]
                        ahead_slots.sort(key=lambda t: t[0])
                        for dx, j in ahead_slots[:2]:
                            kind_name = _KIND_NAMES.get(int(slot_kind[j]), "unknown")
                            obstacles.append({
                                "kind": kind_name,
                                "dx": round(dx, 3),
                                "z_low": round(float(slot_low[j]), 3),
                                "z_high": round(float(slot_high[j]), 3),
                                "depth": round(float(slot_depth[j]), 3),
                            })
                    msg["obstacles"] = obstacles
                    # Availability: range thresholds from plan section 10 /
                    # park_skill_commands constants.
                    nearest_dx = ahead_slots[0][0] if ahead_slots else float("inf")
                    nearest_kind = int(slot_kind[ahead_slots[0][1]]) \
                        if ahead_slots else -1
                    BOX_APPROACH_DIST = 0.5
                    JUMP_CMD_WINDOW = 0.55
                    msg["available"]["jump"] = (
                        nearest_kind == 0 and 0.0 < nearest_dx <= JUMP_CMD_WINDOW)
                    msg["available"]["crouch"] = (nearest_kind == 1)
                    msg["available"]["climb"] = (
                        nearest_kind == 2 and 0.0 < nearest_dx <= BOX_APPROACH_DIST)
                    jump_on = bool(skill_cmd[0] > 0.5)
                    crouch_on = bool(skill_cmd[1] > 0.5)
                    climb_on = bool(skill_cmd[2] > 0.5)
                    msg["skill_active"] = {
                        "jump": jump_on,
                        "crouch": crouch_on,
                        "climb": climb_on,
                    }
                    # Zone/course progress for HUD strip.
                    msg["zone"] = getattr(env, "_zone_key", "FULL")
                    msg["course_progress"] = round(float(max(x, 0.0) / 30.0), 3)
                except Exception as _e:
                    logger.debug(f"park perception read failed: {_e}")

            elif env is not None and hasattr(env, "_compute_bar_obs"):
                bo = env._compute_bar_obs()
                dx = float(bo[0, 0]) * 1.5
                top = float(bo[0, 2]) if bo.shape[1] > 2 else float(bo[0, 1])
                kind = "crawl" if mode == "crawl" else "hurdle"
                msg["obstacles"].append(
                    {"kind": kind, "dx": round(dx, 3), "z_low": 0.0,
                     "z_high": round(top, 3)})
                msg["available"]["jump"] = (
                    mode in ("hurdle", "directed", "launch") and 0.0 < dx <= 0.8)
                msg["available"]["crouch"] = (mode == "crawl")
                msg["available"]["climb"] = False
                msg["skill_active"] = {
                    "jump": bool(getattr(self, "_gait_enabled", False)
                                 and mode in ("hurdle", "directed")),
                    "crouch": bool(mode == "crawl"
                                   and getattr(self, "_gait_enabled", False)),
                }
        except Exception:
            pass
        return msg

    def init_genesis(self):
        """Initialize Genesis scene and Go2 environment."""
        logger.info("Initializing Genesis GPU backend...")
        gs.init(backend=gs.gpu, performance_mode=True)

        expected_obs_dim = None
        if self.checkpoint_dir and os.path.exists(self.checkpoint_dir):
            expected_obs_dim = infer_policy_obs_dim(self.checkpoint_dir)
        self._include_jump_power = expected_obs_dim == 315
        obs = self._create_env(
            include_jump_power=self._include_jump_power,
            env_mode=_env_mode_for_obs_dim(expected_obs_dim),
        )

        # Load policy if checkpoint provided
        if self.checkpoint_dir and os.path.exists(self.checkpoint_dir):
            logger.info(f"Attempting policy load from: {self.checkpoint_dir}")
            obs_dim = obs.shape[-1] if obs is not None else 310
            logger.info(f"Policy obs_dim={obs_dim} (obs tensor shape: {obs.shape if obs is not None else 'None'})")
            self.policy = load_policy(self.checkpoint_dir, obs_dim=obs_dim)
            if self.policy:
                model_path = _resolve_model_path(self.checkpoint_dir)
                self._loaded_model_file = os.path.basename(model_path) if model_path else None
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

    def _create_env(self, include_jump_power: bool, env_mode: str = "walk"):
        """Create or recreate the bridge env with the desired observation layout."""
        if self.env is not None:
            try:
                self.env.close()
            except Exception:
                pass

        if env_mode == "crawl":
            # v45 LL-Crawl primitive — height-commanded locomotion
            from src.sdr_os.envs.go2_crawl_bridge_env import Go2CrawlBridgeEnv

            logger.info(f"Creating Go2CrawlBridgeEnv (camera: {self.camera_res})...")
            self.env = Go2CrawlBridgeEnv(
                num_envs=1,
                dt=1 / 50,
                headless=True,
                camera_res=self.camera_res,
            )
        elif env_mode == "launch":
            # v44 LL-Launch primitive — apex-commanded ballistic jump
            from src.sdr_os.envs.go2_launch_bridge_env import Go2LaunchBridgeEnv

            logger.info(f"Creating Go2LaunchBridgeEnv (camera: {self.camera_res})...")
            self.env = Go2LaunchBridgeEnv(
                num_envs=1,
                dt=1 / 50,
                headless=True,
                camera_res=self.camera_res,
            )
        elif env_mode == "directed":
            # v44.1 directed launch — stand until commanded, jump on X (obs 245)
            from src.sdr_os.envs.go2_directed_launch_bridge_env import Go2DirectedLaunchBridgeEnv

            logger.info(f"Creating Go2DirectedLaunchBridgeEnv (camera: {self.camera_res})...")
            self.env = Go2DirectedLaunchBridgeEnv(
                num_envs=1,
                dt=1 / 50,
                headless=True,
                camera_res=self.camera_res,
            )
        elif env_mode == "hurdle":
            # v46 LL-Hurdle primitive — run up and clear the physical bar
            from src.sdr_os.envs.go2_hurdle_bridge_env import Go2HurdleBridgeEnv

            logger.info(f"Creating Go2HurdleBridgeEnv (camera: {self.camera_res})...")
            self.env = Go2HurdleBridgeEnv(
                num_envs=1,
                dt=1 / 50,
                headless=True,
                camera_res=self.camera_res,
            )
        elif env_mode == "park":
            # v48 unified park — gait + velocity + skill commands, 395-dim obs
            from src.sdr_os.envs.go2_park_bridge_env import Go2ParkBridgeEnv

            logger.info(f"Creating Go2ParkBridgeEnv (camera: {self.camera_res})...")
            self.env = Go2ParkBridgeEnv(
                num_envs=1,
                dt=1 / 50,
                headless=True,
                camera_res=self.camera_res,
            )
        else:
            # Import after gs.init() — GaitCommandManager touches genesis.engine at import time
            from src.sdr_os.envs.go2_env import Go2BridgeEnv

            logger.info(
                f"Creating Go2BridgeEnv (camera: {self.camera_res}, "
                f"jump_power={'on' if include_jump_power else 'off'})..."
            )
            self.env = Go2BridgeEnv(
                num_envs=1,
                dt=1 / 50,
                max_episode_length_s=None,
                headless=True,
                camera_res=self.camera_res,
                include_jump_power=include_jump_power,
            )
        self._env_mode = env_mode
        self.env.build()

        obs, _ = self.env.reset()
        self.current_obs = obs
        logger.info(
            f"{type(self.env).__name__} initialized and reset — obs shape: {obs.shape if obs is not None else None}, "
            f"device: {obs.device if obs is not None else None}"
        )
        return obs
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

    def _checkpoint_step(self, name: str) -> int:
        m = re.search(r"model_(\d+)", name)
        return int(m.group(1)) if m else 0

    def _scan_one_root(self, root_label, root_path, loaded_dir, validation, seen):
        """Scan a single policy root, returning enriched policy dicts."""
        policies = []
        for entry in sorted(os.scandir(root_path), key=lambda e: e.name):
            real = os.path.realpath(entry.path)
            if real in seen:
                continue  # same dir reachable from two roots (e.g. bind mounts)

            if entry.is_dir():
                model_files = glob.glob(os.path.join(entry.path, "model_*.pt"))
                if not model_files:
                    continue
                seen.add(real)

                # Algorithm from cfgs.pkl (best-effort).
                algorithm = "PPO"
                cfg_path = os.path.join(entry.path, "cfgs.pkl")
                if os.path.exists(cfg_path):
                    try:
                        with open(cfg_path, "rb") as f:
                            cfgs = pickle.load(f)
                        if isinstance(cfgs, (list, tuple)) and len(cfgs) >= 5 and isinstance(cfgs[4], dict):
                            algorithm = cfgs[4].get("algorithm", {}).get("class_name", "PPO")
                    except Exception:
                        pass

                checkpoints = sorted((os.path.basename(f) for f in model_files), key=self._checkpoint_step)
                steps = [self._checkpoint_step(c) for c in checkpoints if c.startswith("model_")]
                total_size = sum(os.path.getsize(f) for f in model_files)
                latest_mtime = max(os.path.getmtime(f) for f in model_files)
                is_loaded = loaded_dir is not None and real == os.path.realpath(loaded_dir)
                loaded_ckpt = None
                obs_dim = None
                if is_loaded and self.policy is not None:
                    loaded_ckpt = getattr(self, "_loaded_model_file", None)
                    obs_dim = self.current_obs.shape[-1] if self.current_obs is not None else None

                meta = _parse_version(entry.name)
                policies.append({
                    "name": entry.name,
                    "path": entry.path,
                    "type": "directory",
                    "root_label": root_label,
                    "algorithm": algorithm,
                    "skill": _derive_skill(entry.name, obs_dim),
                    "version": meta["version"],
                    "version_family": meta["version_family"],
                    "run_kind": meta["run_kind"],
                    "validation": validation.get(entry.name, {"status": "untested", "evaluated": False}),
                    "checkpoints": checkpoints,
                    "num_checkpoints": len(checkpoints),
                    "latest_step": max(steps) if steps else None,
                    "size_mb": round(total_size / (1024 * 1024), 1),
                    "modified_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(latest_mtime)),
                    "is_loaded": is_loaded,
                    "loaded_checkpoint": loaded_ckpt,
                })

            elif entry.is_file() and entry.name.endswith(".pt"):
                seen.add(real)
                meta = _parse_version(entry.name)
                policies.append({
                    "name": entry.name,
                    "path": entry.path,
                    "type": "file",
                    "root_label": root_label,
                    "algorithm": "unknown",
                    "skill": _derive_skill(entry.name),
                    "version": meta["version"],
                    "version_family": meta["version_family"],
                    "run_kind": meta["run_kind"],
                    "validation": validation.get(entry.name, {"status": "untested", "evaluated": False}),
                    "checkpoints": [entry.name],
                    "num_checkpoints": 1,
                    "latest_step": None,
                    "size_mb": round(entry.stat().st_size / (1024 * 1024), 1),
                    "modified_iso": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(entry.stat().st_mtime)),
                    "is_loaded": False,
                    "loaded_checkpoint": None,
                })
        return policies

    def _scan_checkpoints(self):
        """Scan every configured policy root (SDR_POLICY_ROOTS) for policies.

        Each policy is tagged with its source root, skill (crawl/launch/walk),
        version family, run kind, and validation status from the run registry.
        Dirs reachable from more than one root are de-duplicated by realpath.
        """
        loaded_dir = self.checkpoint_dir if self.checkpoint_dir else None
        validation = _load_validation_map(_project_root)
        seen: set[str] = set()
        policies = []
        for root_label, root_path in parse_policy_roots(_project_root):
            if not os.path.isdir(root_path):
                continue
            try:
                policies.extend(self._scan_one_root(root_label, root_path, loaded_dir, validation, seen))
            except OSError as e:
                logger.warning(f"Policy root scan failed for {root_path}: {e}")
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
        """Step the simulation with policy, direct joint targets, or zero actions."""
        if self._joint_targets is not None and not self._direct_joint_active():
            logger.info("Joint targets stale — exiting DIRECT_JOINT mode")
            self._joint_targets = None

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
        elif self._direct_joint_active():
            # Raw joint targets from MCP (Claude) — stand gains give scale=1.0
            # so actions are radian offsets from default; PositionActionManager
            # clamps to URDF limits.
            self._switch_gains("stand")
            # _offset_values is (n_envs, num_dofs) at runtime (the manager
            # expands the 1-D actuator buffer) — take env 0's row.
            offsets_t = self.env.action_manager._offset_values
            if offsets_t.dim() == 2:
                offsets_t = offsets_t[0]
            offsets = offsets_t.cpu().tolist()
            action_list = targets_to_actions(self._joint_targets, offsets)
            actions = torch.tensor(
                [action_list], dtype=torch.float32, device=gs.device
            )
            branch = "DIRECT_JOINT"
        elif self._gait_enabled and self.policy is not None and self.current_obs is not None:
            # L2 held: gait walking via policy (training kp)
            self._switch_gains("walk")
            # Park env (Go2ParkEnv) uses attr 'gait_command'; walk/hurdle/crawl
            # envs use 'gait_command_manager'.  Check both so the runner works
            # with all bridge env types without any env-specific branches here.
            gcm = getattr(self.env, "gait_command_manager", None) \
                or getattr(self.env, "gait_command", None)
            if gcm is not None:
                gcm.set_fixed_gait(self._active_gait["name"],
                                   self._active_gait["period"],
                                   self._active_gait["clearance"])
                if hasattr(self.env, "_cmd_buf"):
                    self.env._cmd_buf[0, 0] = torch.clamp(
                        self.env._cmd_buf[0, 0],
                        min=float(self.env.velocity_command.range["lin_vel_x"][0]),
                        max=self._gait_vx_ceiling())
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
            # ManagedEnvironment.step() already reset the terminated envs
            # internally (and bridge envs re-apply the gamepad command in their
            # reset override) — an explicit reset here would double-push the
            # obs history and contaminate the policy's first post-reset frames.
            logger.warning("Episode terminated — env auto-reset")

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
                # Velocity commands: per-source seq + ownership arbitration
                if action == "set_cmd_vel":
                    source = data.get("source", "ui")  # back-compat: untagged = ui
                    is_zero = all(
                        abs(float(cmd_data.get(k, 0.0))) < 1e-3
                        for k in ("linear_x", "linear_y", "angular_z", "angular_y")
                    )
                    decision = self._cmd_arbiter.evaluate(source, cmd_seq, is_zero)
                    if not decision.accepted:
                        continue  # out-of-order within the same session
                    if decision.refresh_ttl:
                        self._last_cmd_vel_time = time.monotonic()
                        self._cmd_vel_received = True
                    if not decision.apply_velocity:
                        continue  # non-owner (e.g. UI idle zeros while MCP drives)

                    # Non-zero operator input aborts direct-joint mode instantly
                    if source == "ui" and not is_zero and self._joint_targets is not None:
                        logger.info("Operator input — aborting direct-joint mode")
                        self._joint_targets = None

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
                            f"cmd_vel: seq={cmd_seq} src={source} "
                            f"owner={self._cmd_arbiter.owner} "
                            f"gait={self._gait_enabled} safety={self._safety_mode} "
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
                        if hasattr(self.env, "_cmd_buf"):
                            self.env._cmd_buf[0, 0] = torch.clamp(
                                self.env._cmd_buf[0, 0],
                                min=float(self.env.velocity_command.range["lin_vel_x"][0]),
                                max=self._gait_vx_ceiling())
                    continue

                if action == "set_joint_targets":
                    # Fresh joint stream auto-recovers from cmd_timeout, same
                    # as cmd_vel; operator ESTOP still requires re-arm.
                    if (
                        self._safety_mode in ("HOLD", "ESTOP")
                        and self._safety_reason == "cmd_timeout"
                    ):
                        logger.info(
                            f"Auto-recovering from {self._safety_mode} on fresh joint targets"
                        )
                        self._safety_mode = "ARMED"
                        self._safety_reason = "ok"
                    if self._safety_mode == "ESTOP":
                        continue  # operator ESTOP requires re-arm first
                    if not self.env:
                        continue
                    try:
                        names = cmd_data.get("names", [])
                        positions = cmd_data.get("positions", [])
                        if self._joint_targets is None:
                            # Enter mode: latch current joint positions so
                            # unspecified joints hold where they are.
                            dofs_idx = self.env.actuator_manager.dofs_idx
                            current = self.env.robot.get_dofs_position(dofs_idx)
                            if current.dim() == 2:  # batched scene: (n_envs, 12)
                                current = current[0]
                            self._joint_targets = current.cpu().tolist()
                            logger.info("Entering DIRECT_JOINT mode")
                        self._joint_targets = merge_joint_targets(
                            self._joint_targets,
                            names,
                            positions,
                            layout=list(self.env.actuator_manager.join_names),
                        )
                        self._last_joint_cmd_time = time.monotonic()
                        # Joint stream is the liveness signal — keep the TTL
                        # clock fresh so mode-exit doesn't instantly HOLD.
                        self._last_cmd_vel_time = self._last_joint_cmd_time
                        self._cmd_vel_received = True
                        self._joint_cmd_count += 1
                        if self._joint_cmd_count % 20 == 1:
                            logger.info(
                                f"joint_targets #{self._joint_cmd_count}: "
                                f"{dict(zip(names, positions))}"
                            )
                    except ValueError as e:
                        logger.warning(f"set_joint_targets rejected: {e}")
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
                        self._joint_targets = None  # exit direct-joint mode
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
                        # Scanning many checkpoint roots reads 100s of cfgs.pkl;
                        # run it off the event loop so the video stream / command
                        # handling never stalls during a rescan.
                        policies = await asyncio.get_event_loop().run_in_executor(
                            None, self._scan_checkpoints
                        )
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
                        expected_obs_dim = infer_policy_obs_dim(checkpoint_dir, model_file)
                        if (
                            expected_obs_dim is not None
                            and self.current_obs is not None
                            and expected_obs_dim != self.current_obs.shape[-1]
                        ):
                            include_jump_power = expected_obs_dim == 315
                            env_mode = _env_mode_for_obs_dim(expected_obs_dim)
                            logger.info(
                                "Policy obs_dim mismatch (current=%s, expected=%s) — recreating env mode=%s jump_power=%s",
                                self.current_obs.shape[-1],
                                expected_obs_dim,
                                env_mode,
                                "on" if include_jump_power else "off",
                            )
                            self._include_jump_power = include_jump_power
                            obs = self._create_env(
                                include_jump_power=include_jump_power, env_mode=env_mode
                            )
                            obs_dim = obs.shape[-1] if obs is not None else expected_obs_dim
                        else:
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
                    elif action == "set_gait":
                        _VALID_GAITS = {"walk", "trot", "pace", "bound", "pronk"}
                        gait_name = str(cmd_data.get("gait", "trot")).lower()
                        # Resolve gait manager: park env uses 'gait_command',
                        # walk/hurdle/crawl envs use 'gait_command_manager'.
                        _gcm = (getattr(self.env, "gait_command_manager", None)
                                or getattr(self.env, "gait_command", None)) \
                            if self.env else None
                        if gait_name not in _VALID_GAITS:
                            status = "error"
                            detail = f"unknown gait {gait_name!r}"
                        elif _gcm is None:
                            status = "unsupported"
                            detail = "active policy has no gait command manager"
                        else:
                            mode = str(cmd_data.get("mode", self._active_gait["mode"])).lower()
                            if mode not in ("walk", "run"):
                                mode = self._active_gait["mode"]
                            self._active_gait["mode"] = mode
                            _mode_default = 0.40 if mode == "run" else 0.50
                            raw_period = float(cmd_data.get("period", _mode_default))
                            if gait_name == "pronk":
                                raw_period = max(0.45, raw_period)
                            period = max(0.35, min(0.8, raw_period))
                            clearance = float(cmd_data.get("clearance",
                                              self._active_gait["clearance"]))
                            # Persist — step_sim re-applies every step.
                            self._active_gait.update(
                                name=gait_name, period=period, clearance=clearance)
                            _gcm.set_fixed_gait(gait_name, period, clearance)
                            # Park bridge: also update via set_gait_by_name so
                            # the env's internal gait index stays in sync.
                            if hasattr(self.env, "set_gait_by_name"):
                                self.env.set_gait_by_name(gait_name, period, clearance)
                            logger.info(
                                f"set_gait: gait={gait_name} period={period:.3f} "
                                f"clearance={clearance:.3f} mode={mode}")
                            detail = f"gait={gait_name} mode={mode}"
                    elif action == "set_crouch":
                        on = bool(cmd_data.get("on", False))
                        if self.env and hasattr(self.env, "set_crouch_intent"):
                            self.env.set_crouch_intent(on)
                        # No error when unsupported — B is harmless on other policies.
                    elif action == "trigger_jump":
                        intensity = float(cmd_data.get("intensity", 1.0))
                        if self.env and hasattr(self.env, "set_jump_intent"):
                            self.env.set_jump_intent(intensity)
                            logger.info(f"trigger_jump: intensity={intensity}")
                            detail = f"jump_intent={intensity}"
                        else:
                            status = "error"
                            detail = "env has no set_jump_intent"
                    elif action == "set_climb":
                        # Y button: climb pulse (park mode) — follows set_crouch pattern.
                        # Harmless on non-park policies (no set_climb_intent attr).
                        intensity = float(cmd_data.get("intensity", 1.0))
                        if self.env and hasattr(self.env, "set_climb_intent"):
                            self.env.set_climb_intent(intensity)
                            logger.info(f"set_climb: intensity={intensity}")
                            detail = f"climb_intent={intensity}"
                        # No error when unsupported — Y is harmless on other policies.
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

    # ── Robot state telemetry (ROS republisher feed) ─────────────

    def _robot_state_snapshot(self) -> dict | None:
        """Payload for telemetry.robot.state, consumed by
        scripts/ros/sim_state_republisher.py → /odom, /imu/data, /joint_states.

        Schema (republisher is the contract): pos [x,y,z], quat [w,x,y,z],
        lin_vel/ang_vel in body frame, projected_gravity, joint_names/pos/vel.
        Reconstructed 2026-06-12 from the consumer after the original
        publisher (uncommitted in the live worktree) was lost in a deploy.
        """
        env = self.env
        if env is None or getattr(env, "robot", None) is None:
            return None

        def _row(t):
            t = t[0] if t.dim() == 2 else t
            return [float(v) for v in t.cpu().tolist()]

        dofs_idx = env.actuator_manager.dofs_idx
        return {
            "pos": _row(env.robot.get_pos()),
            "quat": _row(env.robot.get_quat()),  # Genesis: [w, x, y, z]
            "lin_vel": _row(env.robot_manager.get_linear_velocity()),
            "ang_vel": _row(env.robot_manager.get_angular_velocity()),
            "projected_gravity": _row(env.robot_manager.get_projected_gravity()),
            "joint_names": list(env.actuator_manager.join_names),
            "joint_pos": _row(env.robot.get_dofs_position(dofs_idx)),
            "joint_vel": _row(env.robot.get_dofs_velocity(dofs_idx)),
        }

    # ── Safety (Layer 3) ──────────────────────────────────────────

    JOINT_CMD_TTL_S = 0.5

    def _direct_joint_active(self) -> bool:
        return (
            self._joint_targets is not None
            and time.monotonic() - self._last_joint_cmd_time < self.JOINT_CMD_TTL_S
        )

    def _enforce_cmd_ttl(self):
        """Layer 3: TTL decay and ESTOP on command timeout."""
        if self._safety_mode == "ESTOP":
            return
        if self._direct_joint_active():
            return  # the joint stream is the liveness signal
        if not self._cmd_vel_received:
            return  # Don't enforce TTL until first command arrives

        elapsed = time.monotonic() - self._last_cmd_vel_time
        if elapsed > 2.0:
            self._safety_mode = "ESTOP"
            self._safety_reason = "cmd_timeout"
            self._cmd_arbiter.release()  # fresh sessions accepted after ESTOP
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
        last_robot_state_time = 0

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

                # Release a silent velocity owner (e.g. MCP stream died while
                # the UI tab's idle zeros keep the TTL fresh) — zero immediately.
                if self._cmd_arbiter.expire_owner():
                    logger.info("cmd_vel owner expired — zeroing velocity")
                    if self.env:
                        self.env.zero_velocity()
                    self._gait_enabled = False
                    self._stand_axes = [0.0, 0.0, 0.0, 0.0]

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

                # Publish robot state for the ROS republisher at ~20 Hz
                if now - last_robot_state_time > 0.05 and self.nc and self.nc.is_connected:
                    try:
                        state = self._robot_state_snapshot()
                        if state:
                            await self.nc.publish(
                                "telemetry.robot.state", json.dumps(state).encode()
                            )
                    except Exception as e:
                        if self._step_log_counter % 100 == 1:
                            logger.warning(f"robot state publish failed: {e}")
                    last_robot_state_time = now

                # Publish canonical safety state at 2 Hz
                if now - last_safety_time > 0.5 and self.nc and self.nc.is_connected:
                    self._safety_state_id += 1
                    state = {
                        "state_id": self._safety_state_id,
                        "mode": self._safety_mode,
                        "reason": self._safety_reason,
                        "since_ms": int((now - self._last_cmd_vel_time) * 1000),
                        "cmd_owner": self._cmd_arbiter.owner,
                        "direct_joint": self._direct_joint_active(),
                    }
                    await self.nc.publish("telemetry.safety.state", json.dumps(state).encode())
                    last_safety_time = now

                if now - self._last_perception_pub >= 0.1:
                    try:
                        await self.nc.publish("telemetry.policy.perception",
                                              json.dumps(self._collect_perception()).encode())
                    except Exception:
                        pass
                    self._last_perception_pub = now

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
