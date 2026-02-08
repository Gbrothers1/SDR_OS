#!/usr/bin/env python3
"""
Genesis Bridge Server

Runs Genesis simulation headless, streams rendered frames via WebSocket,
and receives gamepad input via Socket.io from SDR_OS.

Architecture:
- WebSocket server (port 9091): Binary JPEG frames + JSON metrics
- Socket.io client: Connects to SDR_OS server (port 3000) for commands
"""

# Fix import path BEFORE any relative imports
import sys
from pathlib import Path as _PathFix
_project_root = str(_PathFix(__file__).resolve().parent.parent)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

import os
import asyncio
import json
import time
import logging
import threading
from pathlib import Path
from typing import Optional, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
from copy import deepcopy
import fractions

import yaml

import numpy as np
import cv2
import torch

from genesis_bridge.gpu_pipeline import GpuFramePipeline
from genesis_bridge.stream_fanout import StreamFanout
from genesis_bridge.frame_protocol import pack_frame_header, FrameType

# #region agent log
import sys
log_path = Path(__file__).parent.parent / ".cursor" / "debug.log"
try:
    with open(log_path, "a") as f:
        f.write(json.dumps({"id": "log_genesis_import_start", "timestamp": time.time() * 1000, "location": "bridge_server.py:25", "message": "Attempting Genesis import", "data": {"python_path": sys.path[:5], "python_version": sys.version}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "A"}) + "\n")
except: pass
# #endregion

# Genesis import status tracking
_genesis_import_error = None
_genesis_import_error_type = None

def _import_genesis_with_timeout(timeout_sec: float = 5.0):
    """Import genesis with a timeout to avoid startup hangs.

    If the import blocks longer than timeout_sec, fall back to mock mode (gs=None).
    """
    result = {"module": None, "error": None, "error_type": None}

    def _worker():
        try:
            import genesis as _gs
            result["module"] = _gs
        except Exception as exc:  # broad to capture hangs / import errors
            result["error"] = str(exc)
            result["error_type"] = type(exc).__name__

    thread = threading.Thread(target=_worker, name="genesis-import", daemon=True)
    thread.start()
    thread.join(timeout=timeout_sec)

    if thread.is_alive():
        result["error"] = f"Import timed out after {timeout_sec}s"
        result["error_type"] = "Timeout"
        return result
    return result


_import_log_common = {"sessionId": "debug-session", "runId": "run1", "hypothesisId": "A"}

# Attempt Genesis import with timeout guard
_import_result = _import_genesis_with_timeout(timeout_sec=30.0)
gs = _import_result.get("module")
_genesis_import_error = _import_result.get("error")
_genesis_import_error_type = _import_result.get("error_type")

if gs is not None:
    try:
        with open(log_path, "a") as f:
            f.write(json.dumps({"id": "log_genesis_import_success", "timestamp": time.time() * 1000, "location": "bridge_server.py:import", "message": "Genesis import successful", "data": {"genesis_version": getattr(gs, "__version__", "unknown"), "genesis_path": getattr(gs, "__file__", "unknown")}, **_import_log_common}) + "\n")
    except:  # noqa: E722
        pass
    logging.info(f"Genesis v{getattr(gs, '__version__', 'unknown')} imported successfully")
elif _genesis_import_error is not None:
    try:
        with open(log_path, "a") as f:
            f.write(json.dumps({"id": "log_genesis_import_failed", "timestamp": time.time() * 1000, "location": "bridge_server.py:import", "message": "Genesis import failed", "data": {"error_type": _genesis_import_error_type, "error_msg": _genesis_import_error}, **_import_log_common}) + "\n")
    except:  # noqa: E722
        pass
    gs = None
    logging.warning("=" * 60)
    logging.warning("GENESIS IMPORT FAILED OR TIMED OUT - RUNNING IN MOCK MODE")
    logging.warning(f"Error: {_genesis_import_error}")
    logging.warning("")
    logging.warning("Common causes:")
    logging.warning("  1. matplotlib version incompatibility (docstring module removed in newer versions)")
    logging.warning("  2. Not running from the project virtual environment")
    logging.warning("  3. Genesis not installed or import hanging")
    logging.warning("")
    logging.warning("To fix, try:")
    logging.warning("  source .venv/bin/activate")
    logging.warning("  pip install matplotlib<3.8")
    logging.warning("  or investigate genesis import hang (gstaichi/taichi GPU init)")
    logging.warning("=" * 60)

# genesis-forge environment (optional — graceful fallback if not available)
_genesis_forge_available = False
try:
    from genesis_bridge.envs import Go2BridgeEnv
    _genesis_forge_available = True
    logging.info("genesis-forge Go2BridgeEnv available")
except ImportError as _forge_err:
    logging.warning(f"genesis-forge Go2BridgeEnv not available: {_forge_err}")

try:
    import websockets
    from websockets.asyncio.server import serve as ws_serve
    # websockets 16.0+ uses websockets.serve() directly
except ImportError:
    websockets = None
    ws_serve = None
    logging.warning("websockets not installed")

try:
    import socketio
except ImportError:
    socketio = None
    logging.warning("python-socketio not installed")

# Optional: aiohttp and aiortc for WebRTC
_aiohttp = None
_aiortc = None
try:
    import aiohttp
    from aiohttp import web
    _aiohttp = aiohttp
except ImportError:
    logging.warning("aiohttp not installed - WebRTC server disabled")
try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate, VideoStreamTrack as AiortcVideoStreamTrack
    from aiortc.codecs import get_capabilities as _get_rtc_capabilities
    from av import VideoFrame
    _aiortc = True
except ImportError:
    AiortcVideoStreamTrack = None
    VideoFrame = None
    _aiortc = False
    logging.warning("aiortc/av not installed - WebRTC server disabled")

if _aiortc:

    class GenesisWebRTCVideoTrack(AiortcVideoStreamTrack):
        """Video track that pushes frames from the bridge's render loop."""
        
        kind = "video"
        
        def __init__(self, server: "GenesisBridgeServer"):
            super().__init__()
            self._server = server
            self._pts = 0
            self._time_base = fractions.Fraction(1, server.config.target_fps)
        
        async def recv(self):
            if self._server._webrtc_new_frame_event is None:
                raise asyncio.CancelledError()
            await self._server._webrtc_new_frame_event.wait()
            self._server._webrtc_new_frame_event.clear()
            # Prefer GPU tensor when available, fall back to numpy frame
            if self._server._webrtc_latest_tensor is not None:
                frame_np = self._server._webrtc_latest_tensor.cpu().numpy()
                frame_np = np.ascontiguousarray(frame_np)
            elif self._server._webrtc_latest_frame is not None:
                frame_np = self._server._webrtc_latest_frame
            else:
                raise asyncio.CancelledError()
            self._pts += 1
            av_frame = VideoFrame.from_ndarray(frame_np, format="rgb24")
            av_frame.pts = self._pts
            av_frame.time_base = self._time_base
            return av_frame
else:
    GenesisWebRTCVideoTrack = None


# Import our action pipeline
import sys
# Add Genesis to Python path if it exists
genesis_path = Path("/home/ethan/dev/Genesis")
if genesis_path.exists() and str(genesis_path) not in sys.path:
    sys.path.insert(0, str(genesis_path))
sys.path.insert(0, str(Path(__file__).parent.parent / "rl"))
from actions import ActionSpec, ActionRouter, TeleopBridge

logging.basicConfig(level=logging.INFO, force=True)
logger = logging.getLogger(__name__)
# Prevent genesis/twisted from double-logging through root handler
logging.getLogger('genesis').propagate = False
logging.getLogger('twisted').propagate = False


class ScriptStatus(Enum):
    """Script execution status states."""
    IDLE = "idle"
    LOADING = "loading"
    RUNNING = "running"
    PAUSED = "paused"
    ERROR = "error"


class AnomalyDetector:
    """
    Detects anomalies in actions and states for safety.
    
    Monitors for:
    - Sudden large actions (action jerk)
    - Actions exceeding magnitude limits
    - Unreasonable velocities
    - Prolonged bad rewards
    """
    
    def __init__(self, config: 'AnomalyConfig'):
        self.config = config
        self.prev_action = None
        self.consecutive_anomalies = 0
        self.anomaly_history = deque(maxlen=1000)  # Bounded history prevents memory leak
        self.total_anomaly_count = 0  # Total anomalies detected (persists across resets)
        self.reset_count = 0  # Number of auto-stop triggered resets
    
    def check_action(self, action: np.ndarray, dt: float) -> Dict[str, bool]:
        """Check action for anomalies."""
        anomalies = {}

        if not self.config.enabled:
            return anomalies

        # Check action magnitude
        action_norm = np.linalg.norm(action)
        if action_norm > self.config.max_action_magnitude:
            anomalies['action_magnitude'] = True
            logger.warning(f"Action magnitude anomaly: {action_norm:.2f} > {self.config.max_action_magnitude}")

        # Check action jerk (rate of change)
        if self.prev_action is not None:
            jerk = np.linalg.norm(action - self.prev_action) / dt
            if jerk > self.config.max_action_jerk:
                anomalies['action_jerk'] = True
                logger.warning(f"Action jerk anomaly: {jerk:.2f} > {self.config.max_action_jerk}")

        self.prev_action = action.copy()
        return anomalies

    def check_state(self, velocity: Optional[np.ndarray] = None,
                    angular_velocity: Optional[np.ndarray] = None) -> Dict[str, bool]:
        """Check state for anomalies."""
        anomalies = {}

        if not self.config.enabled:
            return anomalies

        if velocity is not None:
            vel_norm = np.linalg.norm(velocity)
            if vel_norm > self.config.max_velocity:
                anomalies['velocity'] = True
                logger.warning(f"Velocity anomaly: {vel_norm:.2f} > {self.config.max_velocity}")

        if angular_velocity is not None:
            ang_vel_norm = np.linalg.norm(angular_velocity)
            if ang_vel_norm > self.config.max_angular_velocity:
                anomalies['angular_velocity'] = True
                logger.warning(f"Angular velocity anomaly: {ang_vel_norm:.2f} > {self.config.max_angular_velocity}")

        return anomalies
    
    def check_reward(self, episode_reward: float) -> Dict[str, bool]:
        """Check reward for anomalies."""
        anomalies = {}
        
        if not self.config.enabled:
            return anomalies
        
        if episode_reward < self.config.min_episode_reward:
            anomalies['reward'] = True
            logger.warning(f"Reward anomaly: {episode_reward:.2f} < {self.config.min_episode_reward}")
        
        return anomalies
    
    def update(self, anomalies: Dict[str, bool]) -> bool:
        """
        Update anomaly tracking and determine if auto-stop should trigger.

        Returns True if auto-stop should trigger.
        """
        if anomalies:
            self.consecutive_anomalies += 1
            self.total_anomaly_count += 1
            self.anomaly_history.append({
                'time': time.time(),
                'anomalies': anomalies
            })

            if (self.config.auto_stop and
                self.consecutive_anomalies >= self.config.consecutive_anomalies_to_stop):
                logger.error(f"Auto-stop triggered after {self.consecutive_anomalies} consecutive anomalies")
                self.reset_count += 1
                return True
        else:
            self.consecutive_anomalies = 0

        return False
    
    def reset(self):
        """Reset detector state."""
        self.prev_action = None
        self.consecutive_anomalies = 0
        self.anomaly_history.clear()  # Clear old anomalies
    
    def get_summary(self) -> Dict:
        """Get anomaly summary."""
        return {
            'consecutive_anomalies': self.consecutive_anomalies,
            'total_anomalies': self.total_anomaly_count,
            'reset_count': self.reset_count,
            'recent_anomalies': list(self.anomaly_history)[-10:]  # Last 10
        }


@dataclass
class AnomalyConfig:
    """Configuration for anomaly detection."""
    # Action anomaly thresholds
    max_action_magnitude: float = 10.0  # Max L2 norm of action (12-joint position offsets)
    max_action_jerk: float = 200.0  # Max change in action per step

    # State anomaly thresholds
    max_velocity: float = 5.0  # m/s
    max_angular_velocity: float = 10.0  # rad/s

    # Reward anomaly
    min_episode_reward: float = -100.0
    
    # Auto-stop settings
    consecutive_anomalies_to_stop: int = 3
    
    # Enable flags
    enabled: bool = True
    auto_stop: bool = True


@dataclass
class BridgeConfig:
    """Configuration for the bridge server."""
    # WebSocket settings
    ws_host: str = "0.0.0.0"
    ws_port: int = 9091
    
    # WebRTC settings
    webrtc_port: int = 9092
    webrtc_bitrate_kbps: int = 6000
    
    # Socket.io settings
    sdr_os_url: str = "http://localhost:3000"
    
    # Rendering settings
    camera_res: tuple = (1280, 720)
    jpeg_quality: int = 80
    target_fps: int = 60
    frame_queue_size: int = 2
    # Stream tuning
    timing_report_interval: float = 5.0  # seconds between coarse timing logs
    ft_log_interval: int = 200  # steps between fine timing logs
    fast_stream: bool = False  # optional fast profile (lower res/quality)
    
    # Metrics streaming
    metrics_rate: int = 5  # Hz
    
    # Simulation settings
    dt: float = 0.02  # 50 Hz simulation
    num_envs: int = 1
    sim_substeps: int = 1  # reduce substeps to cut physics cost
    
    # Data logging
    enable_logging: bool = False
    log_dir: str = "rl/datasets"
    
    # Anomaly detection
    anomaly: AnomalyConfig = field(default_factory=AnomalyConfig)


def _load_bridge_config_file(path: Optional[str]) -> Dict[str, Any]:
    """Load bridge settings from a YAML file.

    Returns a plain dict that can be unpacked into BridgeConfig. Missing files
    or parse errors fall back to an empty dict so defaults still apply.
    """
    if path is None:
        return {}
    cfg_path = Path(path)
    if not cfg_path.exists():
        logging.warning(f"[CONFIG] Bridge config file not found: {cfg_path}")
        return {}
    try:
        with open(cfg_path, "r") as f:
            data = yaml.safe_load(f) or {}
    except Exception as e:
        logging.warning(f"[CONFIG] Failed to read {cfg_path}: {e}")
        return {}

    # Allow either top-level or nested under 'bridge'
    bridge_cfg = data.get("bridge", data)

    # Normalize camera_res if provided as list or string
    cam_res = bridge_cfg.get("camera_res")
    if isinstance(cam_res, str) and "x" in cam_res.lower():
        try:
            w, h = cam_res.lower().split("x")
            bridge_cfg["camera_res"] = (int(w), int(h))
        except Exception:
            logging.warning(f"[CONFIG] Invalid camera_res string '{cam_res}', ignoring")
            bridge_cfg.pop("camera_res", None)
    elif isinstance(cam_res, (list, tuple)) and len(cam_res) == 2:
        bridge_cfg["camera_res"] = (int(cam_res[0]), int(cam_res[1]))

    return bridge_cfg


def _filter_bridge_kwargs(cfg: Dict[str, Any]) -> Dict[str, Any]:
    """Drop unknown keys before constructing BridgeConfig."""
    return {k: v for k, v in cfg.items() if k in BridgeConfig.__dataclass_fields__}


class GenesisBridgeServer:
    """
    Main bridge server that:
    - Runs Genesis simulation
    - Streams rendered frames via WebSocket
    - Receives commands via Socket.io
    - Applies blended teleop/policy actions
    """
    
    def __init__(self, config: Optional[BridgeConfig] = None):
        self.config = config or BridgeConfig()

        # Preserve baseline stream settings for easy revert
        self._baseline_camera_res = self.config.camera_res
        self._baseline_jpeg_quality = self.config.jpeg_quality
        self._baseline_target_fps = self.config.target_fps

        # Optional fast-stream profile (env or config)
        _fast_env = os.getenv("GENESIS_BRIDGE_FAST_STREAM", "").lower() in ("1", "true", "yes", "on")
        if self.config.fast_stream or _fast_env:
            self._apply_fast_stream_profile(reason="env" if _fast_env else "config")
        
        # Action pipeline
        self.action_spec = ActionSpec()
        self.router = ActionRouter()
        self.teleop_bridge = TeleopBridge()
        
        # Anomaly detector
        self.anomaly_detector = AnomalyDetector(self.config.anomaly)
        self._last_anomaly_vibration_time = 0.0
        self.auto_stopped = False
        
        # Genesis scene (initialized later)
        self.scene = None
        self.camera = None
        self.robot = None
        self.env = None
        self.forge_env = None  # genesis-forge ManagedEnvironment
        self.scene_built = False  # Track if scene has been built
        self.current_robot_name = None  # Track currently loaded robot name
        
        # State
        self.running = False
        self.paused = False
        self.mode = "teleop_record"  # Default mode
        self.current_obs = None
        self.current_goal = None
        
        # Policy (loaded separately)
        self.policy = None
        self.policy_checkpoint_path = None  # Path to loaded checkpoint
        self.skill_policies = {}  # skill -> policy
        self.skill_policy_checkpoint_paths = {}
        self.skill_policy_dirs = {
            "stand": "go2-stand",
            "sit": "go2-sit",
            "freeze": "go2-freeze",
        }
        self._last_active_skill = None

        # Command source switching
        self.command_source = "gamepad"  # "gamepad" or "ros"
        self.deadman_active = True  # Set by button handler (L4 on Steam Deck, always True on Xbox)
        self._pending_joystick = None  # Latest joystick data for sync processing
        self._pending_joystick_time = 0.0  # Timestamp of latest joystick sample
        self._decouple_command_updates = True
        self._command_update_hz = 120.0
        self._command_update_thread = None
        self._command_update_lock = threading.Lock()
        self._command_update_zeroed = False
        self._command_stale_seconds = 0.2
        self._cached_action_np = None  # Previous step's actions (numpy, for anomaly detection)
        self._cached_velocity_cmd = None  # Cached velocity command (CPU) for anomaly checks

        # Gamepad button mapping state
        self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
        self.active_skill = None       # None | "stand" | "sit" | "freeze"
        self.gait_preset = "walk"      # "walk" | "trot" | "run"
        self.speed_scale = 0.6         # 0.3 | 0.6 | 1.0
        self.mapping_type = "xbox"     # "steam_deck" | "xbox"
        self._prev_buttons = {}        # for rising-edge detection
        self._r2_tap_time = 0.0        # for double-tap detection
        self._l4_press_time = 0.0      # for hold vs tap detection
        self._l4_held = False          # tracks L4 held state for deadman

        # ROS node (initialized in _initialize_forge_env if available)
        self.ros_node = None

        # WebSocket clients
        self.ws_clients = set()

        # Stream fanout for codec-based client grouping (H.264 vs JPEG)
        self.stream_fanout = StreamFanout()

        # WebRTC: latest frame and event for track (when webrtc enabled)
        self._webrtc_latest_frame = None
        self._webrtc_new_frame_event = None  # Created in run() when webrtc enabled

        # GPU encoding pipeline (initialized after Genesis gs.init())
        self.gpu_pipeline: Optional[GpuFramePipeline] = None
        self._webrtc_latest_tensor: Optional[torch.Tensor] = None
        
        # Socket.io client
        self.sio = None
        
        # Timing
        self.last_frame_time = 0
        self.last_metrics_time = 0
        self.frame_interval = 1.0 / self.config.target_fps
        self.metrics_interval = 1.0 / self.config.metrics_rate
        # Offload encode/broadcast so render loop isn’t blocked
        self._frame_queue = None
        self._frame_sender_task = None
        self._frame_queue_size = max(1, int(getattr(self.config, "frame_queue_size", 2)))
        self.frames_dropped_encode = 0
        self._frame_id_counter = 0

        # Data logging
        self.episode_data = []
        self.episode_count = 0
        
        # Metrics
        self.step_count = 0
        self.total_reward = 0.0
        self.episode_rewards = []
        self.fps_counter = 0
        self.fps = 0
        self.fps_time = time.time()

        # Timing accumulators (coarse)
        self._timing_acc = {
            "step": 0.0,
            "step_count": 0,
            "render": 0.0,
            "render_count": 0,
            "encode": 0.0,
            "encode_count": 0,
            "broadcast": 0.0,
            "broadcast_count": 0,
        }

    def _reset_timing_acc(self):
        self._timing_acc = {
            "step": 0.0,
            "step_count": 0,
            "render": 0.0,
            "render_count": 0,
            "encode": 0.0,
            "encode_count": 0,
            "broadcast": 0.0,
            "broadcast_count": 0,
        }
        
        # Script status tracking
        self.script_status = ScriptStatus.IDLE
        self.current_script_name = None
        self.script_error = None
        
        # Frame statistics
        self.frames_sent = 0
        self.frames_failed = 0
        self.total_frame_bytes = 0
        self.frame_stats_time = time.time()
        self.frame_stats_interval = 5.0  # Emit stats every 5 seconds

    def _apply_fast_stream_profile(self, reason: str = "manual"):
        """Lower stream cost to chase higher FPS; keeps baseline for revert."""
        # Only apply once
        if getattr(self, "_fast_stream_applied", False):
            return
        self._fast_stream_applied = True

        # Set aggressive but reasonable defaults
        self.config.camera_res = (720, 405)
        self.config.jpeg_quality = 60
        # Ensure target fps is not capped low
        self.config.target_fps = max(self.config.target_fps, 60)
        self.frame_interval = 1.0 / self.config.target_fps

        logger.info(
            f"[FAST-STREAM] enabled ({reason}) "
            f"res {self._baseline_camera_res} -> {self.config.camera_res}, "
            f"quality {self._baseline_jpeg_quality} -> {self.config.jpeg_quality}, "
            f"target_fps {self._baseline_target_fps} -> {self.config.target_fps}"
        )

    def restore_stream_defaults(self):
        """Revert to baseline stream settings (for A/B testing)."""
        self.config.camera_res = self._baseline_camera_res
        self.config.jpeg_quality = self._baseline_jpeg_quality
        self.config.target_fps = self._baseline_target_fps
        self.frame_interval = 1.0 / self.config.target_fps
        self._fast_stream_applied = False
        logger.info(
            "[FAST-STREAM] reverted to baseline "
            f"res {self.config.camera_res}, quality {self.config.jpeg_quality}, "
            f"target_fps {self.config.target_fps}"
        )
    
    async def initialize_genesis(self):
        """Initialize Genesis scene with camera."""
        # #region agent log
        try:
            with open(log_path, "a") as f:
                f.write(json.dumps({"id": "log_init_genesis_start", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "initialize_genesis called", "data": {"gs_is_none": gs is None}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
        except: pass
        # #endregion
        if gs is None:
            logger.warning("Genesis module not available - falling back to mock mode")
            logger.warning("To use Genesis, ensure you're running from the project venv and Genesis is installed")
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_gs_none", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "gs is None, returning False - MOCK MODE", "data": {"reason": "Genesis module failed to import at startup"}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            return False

        # Try genesis-forge first
        if _genesis_forge_available:
            success = await self._initialize_forge_env()
            if success:
                return True
            logger.warning("genesis-forge init failed, falling back to manual scene setup")

        try:
            logger.info("Initializing Genesis engine...")
            
            # #region agent log - before gs.init()
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_before_init", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "About to call gs.init()", "data": {"gs_version": getattr(gs, "__version__", "unknown")}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            # CRITICAL: Genesis requires gs.init() before creating any Scene
            # Use GPU backend if available, fallback to CPU
            try:
                gs.init(backend=gs.gpu)
                logger.info("Genesis initialized with GPU backend")
                backend_used = "gpu"
            except Exception as gpu_err:
                logger.warning(f"GPU backend failed ({gpu_err}), falling back to CPU")
                # #region agent log
                try:
                    with open(log_path, "a") as f:
                        f.write(json.dumps({"id": "log_init_genesis_gpu_failed", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "GPU backend failed, trying CPU", "data": {"error": str(gpu_err), "error_type": type(gpu_err).__name__}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
                except: pass
                # #endregion
                gs.init(backend=gs.cpu)
                logger.info("Genesis initialized with CPU backend")
                backend_used = "cpu"
            
            # #region agent log - after gs.init()
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_after_init", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "gs.init() completed successfully", "data": {"backend": backend_used}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion

            # Initialize GPU pipeline after Genesis (CUDA context ready)
            try:
                self.gpu_pipeline = GpuFramePipeline()
                logger.info(f"GPU pipeline: nvjpeg={self.gpu_pipeline.has_nvjpeg}, nvenc={self.gpu_pipeline.has_nvenc}")
            except Exception as e:
                logger.warning(f"GPU pipeline init failed, using CPU encoding: {e}")
                self.gpu_pipeline = None

            logger.info("Creating Genesis scene...")
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_before_scene", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "Before creating Scene", "data": {"gs_available": True, "gs_type": str(type(gs)), "backend": backend_used}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            # Create headless scene
            self.scene = gs.Scene(show_viewer=False)
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_scene_created", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "Scene created successfully", "data": {"scene_type": str(type(self.scene))}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            logger.info("Adding ground plane...")
            # Add ground plane
            self.scene.add_entity(gs.morphs.Plane())
            
            logger.info(f"Adding camera (resolution: {self.config.camera_res})...")
            # Add camera for rendering
            self.camera = self.scene.add_camera(
                res=self.config.camera_res,
                pos=[2.0, 2.0, 1.5],
                lookat=[0.0, 0.0, 0.5],
                fov=40,
                GUI=False
            )
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_camera_created", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "Camera created", "data": {"camera_type": str(type(self.camera)), "resolution": list(self.config.camera_res)}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            # NOTE: Do NOT build scene here - it will be built after robot is loaded
            # Building the scene locks it, preventing entities from being added later
            self.scene_built = False
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_build_deferred", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "Scene created but NOT built yet - will build after robot load", "data": {"scene_built": False, "camera_ready": self.camera is not None}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            logger.info("Genesis scene initialized (scene will be built after robot is loaded)")
            return True
            
        except Exception as e:
            import traceback
            tb_str = traceback.format_exc()
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_init_genesis_exception", "timestamp": time.time() * 1000, "location": "bridge_server.py:initialize_genesis", "message": "Exception during Genesis init - falling back to mock mode", "data": {"error_type": type(e).__name__, "error_msg": str(e), "traceback": tb_str}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "E"}) + "\n")
            except: pass
            # #endregion
            logger.error(f"Failed to initialize Genesis: {type(e).__name__}: {e}")
            logger.error(f"Traceback:\n{tb_str}")
            logger.warning("Falling back to mock mode due to Genesis initialization failure")
            return False

    async def _initialize_forge_env(self):
        """Initialize a genesis-forge ManagedEnvironment."""
        if not _genesis_forge_available or gs is None:
            return False

        try:
            logger.info("Creating Go2BridgeEnv (genesis-forge)...")

            # CRITICAL: Genesis requires gs.init() before creating any Scene
            try:
                gs.init(backend=gs.gpu)
                logger.info("Genesis initialized with GPU backend (forge path)")
            except Exception as gpu_err:
                logger.warning(f"GPU backend failed ({gpu_err}), falling back to CPU")
                gs.init(backend=gs.cpu)
                logger.info("Genesis initialized with CPU backend (forge path)")

            self.forge_env = Go2BridgeEnv(
                num_envs=self.config.num_envs,
                dt=self.config.dt,
                max_episode_length_s=None,
                headless=True,
                camera_res=self.config.camera_res,
                sim_substeps=self.config.sim_substeps,
            )
            self.forge_env.build()

            # Backward compat: expose scene/camera/robot
            self.scene = self.forge_env.scene
            self.camera = self.forge_env.camera
            self.robot = self.forge_env.robot
            self.scene_built = True

            # Initial reset
            obs, _ = self.forge_env.reset()
            self.current_obs = obs

            logger.info("Go2BridgeEnv initialized successfully")

            # Try to auto-load locomotion policy
            default_checkpoint = os.path.join(
                str(Path(__file__).parent.parent), "rl", "checkpoints", "go2-locomotion"
            )
            if os.path.exists(default_checkpoint):
                logger.info(f"[DIAG] Found checkpoint dir: {default_checkpoint}")
                load_ok = self.load_policy(default_checkpoint)
                logger.info(f"[DIAG] load_policy returned: {load_ok}, self.policy is None: {self.policy is None}")
                if self.policy is not None:
                    logger.info(f"[DIAG] Policy type: {type(self.policy)}")
            else:
                logger.info(f"No locomotion policy found at {default_checkpoint}, using zero actions")
                logger.info(f"[DIAG] Checkpoint dir does NOT exist: {default_checkpoint}")

            # Start ROS node if available (rclpy or roslibpy fallback)
            try:
                from genesis_bridge.ros_bridge_node import GenesisBridgeROSNode, ROS_AVAILABLE
                if ROS_AVAILABLE:
                    self.ros_node = GenesisBridgeROSNode(self)
                    started = self.ros_node.start()
                    logger.info(f"ROS node started: {started} (backend: {self.ros_node._backend})")
                else:
                    logger.warning("ROS integration not available (no rclpy or roslibpy)")
            except Exception as e:
                logger.warning(f"ROS bridge node init failed: {e}")

            return True
        except Exception as e:
            import traceback
            logger.error(f"Failed to initialize Go2BridgeEnv: {e}")
            logger.error(f"Traceback:\n{traceback.format_exc()}")
            self.forge_env = None
            return False

    def load_policy(self, checkpoint_dir: str, model_file: str = None) -> bool:
        """Load a trained locomotion policy from checkpoint directory.

        Args:
            checkpoint_dir: Path to directory containing model_*.pt and cfgs.pkl
            model_file: Optional specific model filename (e.g. 'model_900.pt').
                         If None, the latest checkpoint is used.

        Returns:
            True if policy loaded successfully
        """
        try:
            import torch
            import glob
            import pickle
            logger.info(f"[DIAG] load_policy: importing rsl_rl and genesis_forge...")
            from rsl_rl.runners import OnPolicyRunner
            from genesis_forge.wrappers import RslRlWrapper
            logger.info(f"[DIAG] load_policy: imports OK")

            if model_file:
                # Use the explicitly requested checkpoint
                model_path = os.path.join(checkpoint_dir, model_file)
                if not os.path.exists(model_path):
                    logger.error(f"Requested model file not found: {model_path}")
                    return False
            else:
                # Find latest checkpoint
                model_files = glob.glob(os.path.join(checkpoint_dir, "model_*.pt"))
                if not model_files:
                    logger.error(f"No model files found in {checkpoint_dir}")
                    return False
                model_files.sort()
                model_path = model_files[-1]
            logger.info(f"[DIAG] load_policy: using checkpoint {model_path}")

            # Load config
            cfg_path = os.path.join(checkpoint_dir, "cfgs.pkl")
            if not os.path.exists(cfg_path):
                logger.error(f"No cfgs.pkl found in {checkpoint_dir}")
                return False
            [cfg] = pickle.load(open(cfg_path, "rb"))
            logger.info(f"[DIAG] load_policy: cfg loaded, obs_groups={cfg.get('obs_groups', 'NOT SET')}")

            # We need a dummy env for OnPolicyRunner to extract policy dims.
            # Use the forge_env wrapped for rsl_rl compatibility.
            if self.forge_env is None:
                logger.error("Cannot load policy: forge_env not initialized")
                return False

            wrapped_env = RslRlWrapper(self.forge_env)
            logger.info(f"[DIAG] load_policy: RslRlWrapper created, num_obs={wrapped_env.num_observations}, num_actions={wrapped_env.num_actions}")

            runner = OnPolicyRunner(wrapped_env, cfg, checkpoint_dir, device=gs.device)
            runner.load(model_path)
            self.policy = runner.get_inference_policy(device=gs.device)
            self.policy_checkpoint_path = model_path

            logger.info(f"[DIAG] load_policy: SUCCESS - policy type={type(self.policy)}")
            logger.info(f"Policy loaded from {model_path}")
            return True
        except Exception as e:
            import traceback
            logger.error(f"Failed to load policy: {e}\n{traceback.format_exc()}")
            self.policy = None
            self.policy_checkpoint_path = None
            return False

    def load_skill_policy(self, skill: str, checkpoint_dir: str, model_file: str = None) -> bool:
        """Load a trained skill policy (stand/sit/freeze) from checkpoint directory."""
        if skill is None:
            return False
        try:
            import glob
            import pickle
            logger.info(f"[DIAG] load_skill_policy: {skill} from {checkpoint_dir}")
            from rsl_rl.runners import OnPolicyRunner
            from genesis_forge.wrappers import RslRlWrapper

            if model_file:
                model_path = os.path.join(checkpoint_dir, model_file)
                if not os.path.exists(model_path):
                    logger.error(f"Requested skill model file not found: {model_path}")
                    return False
            else:
                model_files = glob.glob(os.path.join(checkpoint_dir, "model_*.pt"))
                if not model_files:
                    logger.error(f"No skill model files found in {checkpoint_dir}")
                    return False
                model_files.sort()
                model_path = model_files[-1]

            cfg_path = os.path.join(checkpoint_dir, "cfgs.pkl")
            if not os.path.exists(cfg_path):
                logger.error(f"No cfgs.pkl found in {checkpoint_dir}")
                return False
            [cfg] = pickle.load(open(cfg_path, "rb"))

            if self.forge_env is None:
                logger.error("Cannot load skill policy: forge_env not initialized")
                return False

            wrapped_env = RslRlWrapper(self.forge_env)
            runner = OnPolicyRunner(wrapped_env, cfg, checkpoint_dir, device=gs.device)
            runner.load(model_path)
            policy = runner.get_inference_policy(device=gs.device)

            self.skill_policies[skill] = policy
            self.skill_policy_checkpoint_paths[skill] = model_path
            logger.info(f"Skill policy loaded: {skill} from {model_path}")
            return True
        except Exception as e:
            import traceback
            logger.error(f"Failed to load skill policy {skill}: {e}\n{traceback.format_exc()}")
            return False

    def _ensure_skill_policy_loaded(self, skill: str) -> bool:
        """Ensure a skill policy is loaded (lazy-loads from default checkpoint dirs)."""
        if skill in self.skill_policies:
            return True
        skill_dir = self.skill_policy_dirs.get(skill)
        if not skill_dir:
            return False
        checkpoint_dir = os.path.join(str(Path(__file__).parent.parent), "rl", "checkpoints", skill_dir)
        if not os.path.exists(checkpoint_dir):
            logger.info(f"No skill checkpoint dir found for {skill}: {checkpoint_dir}")
            return False
        return self.load_skill_policy(skill, checkpoint_dir)

    def scan_policies(self):
        """Scan rl/checkpoints/ for available policy checkpoints.

        Returns list of dicts with metadata for each discovered policy.
        """
        import glob as _glob
        import pickle

        checkpoints_dir = Path(__file__).parent.parent / "rl" / "checkpoints"
        if not checkpoints_dir.exists():
            return []

        policies = []

        for entry in sorted(checkpoints_dir.iterdir()):
            if entry.is_dir():
                model_files = sorted(_glob.glob(str(entry / "model_*.pt")))
                if not model_files:
                    continue
                # Parse latest step from filename like model_1999.pt
                latest_step = None
                try:
                    stem = Path(model_files[-1]).stem  # e.g. "model_1999"
                    latest_step = int(stem.split("_", 1)[1])
                except (ValueError, IndexError):
                    pass

                # Determine algorithm from cfgs.pkl
                algorithm = "unknown"
                cfg_path = entry / "cfgs.pkl"
                if cfg_path.exists():
                    try:
                        with open(cfg_path, "rb") as f:
                            [cfg] = pickle.load(f)
                        algo_class = cfg.get("algorithm", {}).get("class_name", "")
                        if "PPO" in algo_class or "ppo" in str(entry.name).lower():
                            algorithm = "PPO"
                        elif "BC" in algo_class or "bc" in str(entry.name).lower():
                            algorithm = "BC"
                    except Exception:
                        # Fallback: guess from directory name
                        name_lower = entry.name.lower()
                        if "ppo" in name_lower:
                            algorithm = "PPO"
                        elif "bc" in name_lower:
                            algorithm = "BC"

                total_size = sum(f.stat().st_size for f in entry.rglob("*") if f.is_file())

                # Build list of individual checkpoint filenames (newest first)
                checkpoint_names = [Path(f).name for f in reversed(model_files)]

                policies.append({
                    "name": entry.name,
                    "path": str(entry.resolve()),
                    "type": "directory",
                    "algorithm": algorithm,
                    "num_checkpoints": len(model_files),
                    "latest_step": latest_step,
                    "size_mb": round(total_size / (1024 * 1024), 1),
                    "modified_iso": time.strftime(
                        "%Y-%m-%dT%H:%M:%S",
                        time.localtime(entry.stat().st_mtime),
                    ),
                    "is_loaded": (
                        self.policy_checkpoint_path is not None
                        and str(entry.resolve()) in str(Path(self.policy_checkpoint_path).resolve().parent)
                    ),
                    "loaded_checkpoint": (
                        os.path.basename(self.policy_checkpoint_path)
                        if self.policy_checkpoint_path
                        and str(entry.resolve()) in str(Path(self.policy_checkpoint_path).resolve().parent)
                        else None
                    ),
                    "checkpoints": checkpoint_names,
                })

            elif entry.is_file() and entry.suffix == ".pt":
                name_lower = entry.stem.lower()
                if "ppo" in name_lower:
                    algorithm = "PPO"
                elif "bc" in name_lower:
                    algorithm = "BC"
                else:
                    algorithm = "unknown"

                policies.append({
                    "name": entry.stem,
                    "path": str(entry.resolve()),
                    "type": "file",
                    "algorithm": algorithm,
                    "num_checkpoints": 1,
                    "latest_step": None,
                    "size_mb": round(entry.stat().st_size / (1024 * 1024), 1),
                    "modified_iso": time.strftime(
                        "%Y-%m-%dT%H:%M:%S",
                        time.localtime(entry.stat().st_mtime),
                    ),
                    "is_loaded": (
                        self.policy_checkpoint_path is not None
                        and str(entry.resolve()) == str(Path(self.policy_checkpoint_path).resolve())
                    ),
                })

        logger.info(f"Scanned {len(policies)} policies in {checkpoints_dir}")
        return policies

    def load_robot(self, urdf_path: str, position: list = None, orientation: list = None):
        """Load a robot from URDF file."""
        # #region agent log
        log_path = Path(__file__).parent.parent / ".cursor" / "debug.log"
        try:
            with open(log_path, "a") as f:
                f.write(json.dumps({"id": "log_load_robot_method_entry", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:entry", "message": "load_robot method called", "data": {"urdf_path": urdf_path, "position": position, "orientation": orientation, "gs_is_none": gs is None, "scene_is_none": self.scene is None}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
        except: pass
        # #endregion
        
        if gs is None or self.scene is None:
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_no_genesis", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:no_genesis", "message": "Cannot load robot - Genesis not initialized", "data": {"gs_is_none": gs is None, "scene_is_none": self.scene is None}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            logger.warning("Cannot load robot - Genesis not initialized")
            return False
        
        try:
            position = position or [0.0, 0.0, 0.5]
            orientation = orientation or [1.0, 0.0, 0.0, 0.0]
            
            # Check if scene is already built - if so, recreate it to allow adding entities
            if self.scene_built:
                logger.info("Scene already built - recreating to allow robot loading")
                # Save camera config
                camera_res = self.config.camera_res
                camera_pos = [2.0, 2.0, 1.5]
                camera_lookat = [0.0, 0.0, 0.5]
                # Recreate scene
                self.scene = gs.Scene(show_viewer=False)
                self.scene.add_entity(gs.morphs.Plane())
                self.camera = self.scene.add_camera(
                    res=camera_res,
                    pos=camera_pos,
                    lookat=camera_lookat,
                    fov=40,
                    GUI=False
                )
                self.scene_built = False
                self.robot = None  # Clear previous robot reference
            
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_before_add", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:before_add", "message": "About to add URDF entity", "data": {"urdf_path": urdf_path, "position": position, "orientation": orientation, "scene_built": self.scene_built}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            # Add robot from URDF
            self.robot = self.scene.add_entity(
                gs.morphs.URDF(
                    file=urdf_path,
                    pos=position,
                    quat=orientation
                )
            )
            
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_after_add", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:after_add", "message": "URDF entity added", "data": {"robot_type": str(type(self.robot)), "robot_is_none": self.robot is None}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            # Set up camera to follow robot
            if self.camera and self.robot:
                self.camera.follow_entity(self.robot, smoothing=0.9)
            
            # Build the scene NOW (after robot is added)
            logger.info("Building Genesis scene with robot...")
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_before_build", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:before_build", "message": "About to call scene.build()", "data": {"urdf_path": urdf_path, "robot_added": self.robot is not None, "camera_set": self.camera is not None}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            try:
                self.scene.build()
                self.scene_built = True
                # #region agent log
                try:
                    with open(log_path, "a") as f:
                        f.write(json.dumps({"id": "log_load_robot_scene_built", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:scene_built", "message": "Scene built successfully with robot", "data": {"urdf_path": urdf_path, "scene_built": True}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
                except: pass
                # #endregion
            except Exception as build_err:
                import traceback
                tb_str = traceback.format_exc()
                # #region agent log
                try:
                    with open(log_path, "a") as f:
                        f.write(json.dumps({"id": "log_load_robot_build_exception", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:build_exception", "message": "Exception during scene.build()", "data": {"urdf_path": urdf_path, "error": str(build_err), "error_type": type(build_err).__name__, "traceback": tb_str}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
                except: pass
                # #endregion
                logger.error(f"Failed to build scene: {build_err}")
                raise
            
            logger.info(f"Loaded robot from {urdf_path}")
            
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_success", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:success", "message": "Robot loaded successfully", "data": {"urdf_path": urdf_path}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            return True
            
        except Exception as e:
            import traceback
            tb_str = traceback.format_exc()
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_exception", "timestamp": time.time() * 1000, "location": "bridge_server.py:load_robot:exception", "message": "Exception during robot load", "data": {"urdf_path": urdf_path, "error": str(e), "error_type": type(e).__name__, "traceback": tb_str}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            logger.error(f"Failed to load robot: {e}")
            return False
    
    def unload_robot(self):
        """Unload the current robot and reset to empty scene."""
        if gs is None or self.scene is None:
            logger.warning("Cannot unload robot - Genesis not initialized")
            return False
        
        try:
            logger.info("Unloading robot - recreating empty scene")
            
            # Save camera config
            camera_res = self.config.camera_res
            camera_pos = [2.0, 2.0, 1.5]
            camera_lookat = [0.0, 0.0, 0.5]
            
            # Recreate scene without robot
            self.scene = gs.Scene(show_viewer=False)
            self.scene.add_entity(gs.morphs.Plane())
            self.camera = self.scene.add_camera(
                res=camera_res,
                pos=camera_pos,
                lookat=camera_lookat,
                fov=40,
                GUI=False
            )
            
            # Clear robot reference
            self.robot = None
            self.current_robot_name = None
            self.scene_built = False
            
            logger.info("Robot unloaded - scene reset to empty state")
            return True
            
        except Exception as e:
            logger.error(f"Failed to unload robot: {e}")
            return False
    
    def render_frame(self, encode: bool = True):
        """
        Render current scene.
        
        Args:
            encode: when True, return JPEG bytes; when False, return BGR ndarray ready for encoding.
        """
        if self.camera is None or not self.scene_built:
            logger.debug("render_frame: Camera is None or scene not built, generating diagnostic frame")
            # Return placeholder frame in mock mode
            frame = np.zeros((self.config.camera_res[1], self.config.camera_res[0], 3), dtype=np.uint8)
            frame[:] = (30, 30, 30)  # Dark gray
            
            # Add warning banner at top
            cv2.rectangle(frame, (0, 0), (self.config.camera_res[0], 60), (0, 50, 100), -1)
            cv2.putText(
                frame, "MOCK MODE - Genesis Not Initialized",
                (50, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 150, 255), 2
            )
            
            # Add text overlay with status info
            y_offset = 120
            cv2.putText(
                frame, "Genesis Bridge Server",
                (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (100, 100, 100), 3
            )
            y_offset += 60
            
            # Show reason for mock mode
            if gs is None:
                reason = "Genesis module failed to import"
                if _genesis_import_error:
                    # Truncate long error messages
                    err_preview = _genesis_import_error[:60] + "..." if len(_genesis_import_error) > 60 else _genesis_import_error
                    cv2.putText(
                        frame, f"Reason: {err_preview}",
                        (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 200), 2
                    )
                    y_offset += 35
            else:
                cv2.putText(
                    frame, "Reason: Scene/Camera not created",
                    (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 200), 2
                )
                y_offset += 35
            
            y_offset += 20
            cv2.putText(
                frame, f"Step: {self.step_count}",
                (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (150, 150, 150), 2
            )
            y_offset += 40
            cv2.putText(
                frame, f"Mode: {self.mode}",
                (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (150, 150, 150), 2
            )
            y_offset += 40
            cv2.putText(
                frame, f"Status: {self.script_status.value.upper()}",
                (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 1, (150, 150, 150), 2
            )
            y_offset += 40
            if self.current_script_name:
                cv2.putText(
                    frame, f"Script: {self.current_script_name}",
                    (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (150, 150, 150), 2
                )
                y_offset += 35
            if self.script_error:
                cv2.putText(
                    frame, f"Error: {self.script_error[:50]}",
                    (50, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2
                )
                y_offset += 35
            
            # Add helpful hint at bottom
            cv2.putText(
                frame, "Check terminal logs for details. Ensure venv is activated.",
                (50, self.config.camera_res[1] - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 80, 80), 1
            )
            
            # Feed WebRTC track when available (mock frame: BGR -> RGB)
            if self._webrtc_new_frame_event is not None:
                self._webrtc_latest_frame = np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
                self._webrtc_new_frame_event.set()
            if not encode:
                return frame
            encode_start = time.time()
            _, jpeg_buf = cv2.imencode(
                '.jpg', frame,
                [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
            )
            self._timing_acc["encode"] += time.time() - encode_start
            self._timing_acc["encode_count"] += 1
            return jpeg_buf.tobytes()
        
        try:
            # Update camera tracking only if following a robot
            if self.robot is not None:
                try:
                    self.camera.update_following()
                except Exception as follow_err:
                    # Only log once per session to avoid spam
                    if not hasattr(self, '_follow_error_logged'):
                        logger.warning(f"Camera follow update failed (will not repeat): {follow_err}")
                        self._follow_error_logged = True
            
            # Render frame - returns tuple (rgb, depth, segmentation, normal)
            render_start = time.time()
            render_result = self.camera.render(rgb=True)
            self._timing_acc["render"] += time.time() - render_start
            self._timing_acc["render_count"] += 1
            
            # Extract RGB array/tensor from tuple (first element)
            if isinstance(render_result, tuple):
                rgb_data = render_result[0]
            else:
                # Fallback for older API that might return tensor directly
                rgb_data = render_result

            # Keep as tensor if possible for GPU pipeline
            rgb_tensor = None
            if isinstance(rgb_data, torch.Tensor):
                rgb_tensor = rgb_data
                rgb_array = rgb_data.cpu().numpy() if rgb_data.is_cuda else rgb_data.numpy()
            else:
                rgb_array = rgb_data

            # OpenCV expects BGR
            bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)

            # Feed WebRTC track when available (prefer tensor for GPU pipeline)
            if self._webrtc_new_frame_event is not None:
                if rgb_tensor is not None:
                    self._webrtc_latest_tensor = rgb_tensor
                    self._webrtc_latest_frame = None
                else:
                    self._webrtc_latest_tensor = None
                    self._webrtc_latest_frame = np.ascontiguousarray(rgb_array)
                self._webrtc_new_frame_event.set()
            if not encode:
                return bgr_array

            # Encode as JPEG (use GPU pipeline when available)
            logger.debug(f"render_frame: Rendering frame from camera, shape: {bgr_array.shape}")
            encode_start = time.time()
            if self.gpu_pipeline is not None and rgb_tensor is not None:
                jpeg_bytes = self.gpu_pipeline.encode_jpeg(rgb_tensor, quality=self.config.jpeg_quality)
            elif self.gpu_pipeline is not None:
                jpeg_bytes = self.gpu_pipeline.encode_jpeg_numpy(rgb_array, quality=self.config.jpeg_quality)
            else:
                _, jpeg_buf = cv2.imencode(
                    '.jpg', bgr_array,
                    [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
                )
                jpeg_bytes = jpeg_buf.tobytes()
            self._timing_acc["encode"] += time.time() - encode_start
            self._timing_acc["encode_count"] += 1

            return jpeg_bytes
            
        except Exception as e:
            # Reduce log spam - only log render errors periodically
            if not hasattr(self, '_last_render_error_time') or time.time() - self._last_render_error_time > 5.0:
                logger.error(f"Render error: {e}")
                self._last_render_error_time = time.time()
            return None
    
    def get_script_status(self) -> Dict[str, Any]:
        """Get current script execution status."""
        return {
            'status': self.script_status.value,
            'script_name': self.current_script_name,
            'error': self.script_error,
            'timestamp': time.time()
        }
    
    def get_frame_statistics(self) -> Dict[str, Any]:
        """Get frame streaming statistics."""
        avg_frame_size = 0
        if self.frames_sent > 0:
            avg_frame_size = self.total_frame_bytes / self.frames_sent

        # Calculate frame rate over last interval
        time_elapsed = time.time() - self.frame_stats_time
        frame_rate = 0
        bitrate_mibps = 0.0
        if time_elapsed > 0:
            frame_rate = self.frames_sent / time_elapsed if time_elapsed > 0 else 0
            # Bitrate in Mibps: (bytes * 8 bits/byte) / (seconds * 1024 * 1024 bits/Mibit)
            bitrate_mibps = (self.total_frame_bytes * 8) / (time_elapsed * 1024 * 1024)

        return {
            'frames_sent': self.frames_sent,
            'frames_failed': self.frames_failed,
            'avg_frame_size': avg_frame_size,
            'total_frame_bytes': self.total_frame_bytes,
            'camera_available': self.camera is not None,
            'frame_rate': frame_rate,
            'bitrate_mibps': round(bitrate_mibps, 2),
            'clients_connected': len(self.ws_clients),
            'frames_dropped_encode': self.frames_dropped_encode
        }
    
    def get_initialization_status(self) -> Dict[str, Any]:
        """Get detailed initialization status for diagnostics."""
        return {
            "genesis_module_available": gs is not None,
            "genesis_version": getattr(gs, "__version__", None) if gs else None,
            "genesis_import_error": _genesis_import_error,
            "genesis_import_error_type": _genesis_import_error_type,
            "scene_created": self.scene is not None,
            "camera_created": self.camera is not None,
            "is_mock_mode": self.camera is None,
            "python_executable": sys.executable,
            "python_version": sys.version,
        }
    
    def get_training_metrics(self) -> Dict[str, Any]:
        """Get current training metrics for streaming."""
        router_state = self.router.get_state()
        init_status = self.get_initialization_status()
        
        # Try to get memory stats (optional, fails gracefully if psutil not available)
        memory_stats = {}
        try:
            import psutil
            process = psutil.Process()
            memory_info = process.memory_info()
            memory_stats = {
                "memory_mb": memory_info.rss / 1024 / 1024,
                "episode_buffer_size": len(self.episode_data)
            }
        except (ImportError, Exception):
            # psutil not available or error getting memory - skip gracefully
            pass
        
        return {
            "type": "training_metrics",
            "step_count": self.step_count,
            "num_envs": self.config.num_envs,
            "total_reward": self.total_reward,
            "reward_components": {},  # Would come from RewardManager
            "episode_lengths": {
                "mean": np.mean(self.episode_rewards) if self.episode_rewards else 0,
                "min": min(self.episode_rewards) if self.episode_rewards else 0,
                "max": max(self.episode_rewards) if self.episode_rewards else 0
            },
            "fps": self.fps,
            "blend_alpha": router_state["alpha"],
            "deadman_active": router_state["deadman_active"],
            "actor_tag": router_state["actor_tag"],
            "safety_flags": self.action_spec.last_clamp_flags,
            "confidence": router_state["confidence"],
            "confidence_gate_active": router_state["confidence_gate_active"],
            "mode": self.mode,
            "command_source": self.command_source,
            "active_skill": self.active_skill,
            "gait_preset": self.gait_preset,
            "posture_offset": self.posture_offset.copy(),
            "speed_scale": self.speed_scale,
            "mapping_type": self.mapping_type,
            "deadman_active_forge": self.deadman_active,
            "policy_loaded": self.policy is not None,
            "policy_checkpoint": os.path.basename(self.policy_checkpoint_path) if self.policy_checkpoint_path else None,
            "script_status": self.script_status.value,
            "script_name": self.current_script_name,
            "script_error": self.script_error,
            # Add mock mode info
            "is_mock_mode": init_status["is_mock_mode"],
            "genesis_available": init_status["genesis_module_available"],
            "genesis_version": init_status["genesis_version"],
            # Anomaly detection counters
            "anomaly_count": self.anomaly_detector.total_anomaly_count,
            "anomaly_reset_count": self.anomaly_detector.reset_count,
            "consecutive_anomalies": self.anomaly_detector.consecutive_anomalies,
            # Memory monitoring (optional, only if psutil available)
            **memory_stats,
            # Forge env structured data
            **(self._get_forge_metrics() if self.forge_env is not None else {}),
        }
    
    def _get_forge_metrics(self) -> dict:
        """Get structured telemetry from the genesis-forge environment."""
        data = {}
        try:
            data["obs_breakdown"] = self.forge_env.get_obs_breakdown()
            data["reward_breakdown"] = self.forge_env.get_reward_breakdown()
            data["velocity_command"] = self.forge_env.get_velocity_command()
            data["num_envs"] = self.forge_env.num_envs
            # Robot base pose (env 0) — wxyz quaternion, Z-up
            if self.forge_env.robot_manager is not None:
                quat = self.forge_env.robot_manager.base_quat[0]  # (4,) tensor [w,x,y,z]
                data["robot_orientation"] = quat.cpu().tolist()
        except Exception as e:
            logger.debug(f"Error getting forge metrics: {e}")
        return data

    def _apply_gamepad_command(self, joy_data: dict) -> bool:
        if self.forge_env is None or joy_data is None:
            return False
        # Apply speed scaling to stick values
        scaled = {
            'leftStickX': joy_data.get('leftStickX', 0.0) * self.speed_scale,
            'leftStickY': joy_data.get('leftStickY', 0.0) * self.speed_scale,
            'rightStickX': joy_data.get('rightStickX', 0.0) * self.speed_scale,
            'rightStickY': joy_data.get('rightStickY', 0.0) * self.speed_scale,
        }
        with self._command_update_lock:
            self.forge_env.set_velocity_from_gamepad(scaled)
        return True

    def _zero_gamepad_command(self):
        if self._command_update_zeroed:
            return
        if self._apply_gamepad_command({
            'leftStickX': 0.0,
            'leftStickY': 0.0,
            'rightStickX': 0.0,
            'rightStickY': 0.0,
        }):
            self._command_update_zeroed = True

    def _handle_button_states(self, buttons: dict):
        """Process button states with rising-edge detection.

        Called from the sync gamepad thread on every button update.
        Only acts on rising edges (button was False, now True).
        """
        prev = self._prev_buttons
        now_time = time.time()

        def rising(key):
            return buttons.get(key, False) and not prev.get(key, False)

        def held(key):
            return buttons.get(key, False)

        # --- L4: deadman / quick-reset (Steam Deck only) ---
        if self.mapping_type == "steam_deck":
            if rising("L4"):
                self._l4_press_time = now_time
                self._l4_held = True
            elif prev.get("L4", False) and not buttons.get("L4", False):
                # L4 released
                self._l4_held = False
                if (now_time - self._l4_press_time) < 0.3:
                    # Quick tap → reset posture
                    self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
                    logger.info("[BUTTONS] L4 tap → posture reset")
            self.deadman_active = self._l4_held
        else:
            # Xbox: no deadman, always active
            self.deadman_active = True

        # --- R4: freeze stance (Steam Deck only) ---
        if self.mapping_type == "steam_deck" and rising("R4"):
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            self.active_skill = "freeze"
            logger.info("[BUTTONS] R4 → freeze stance")

        # --- D-pad: posture nudges ---
        if rising("DpadUp"):
            self.posture_offset["height"] = min(
                self.posture_offset["height"] + 0.03, 0.10)
            logger.debug(f"[BUTTONS] D-Up → height={self.posture_offset['height']:.2f}")
        if rising("DpadDown"):
            self.posture_offset["height"] = max(
                self.posture_offset["height"] - 0.03, -0.10)
            logger.debug(f"[BUTTONS] D-Down → height={self.posture_offset['height']:.2f}")
        if rising("DpadLeft"):
            self.posture_offset["roll"] = max(
                self.posture_offset["roll"] - 0.05, -0.30)
            logger.debug(f"[BUTTONS] D-Left → roll={self.posture_offset['roll']:.2f}")
        if rising("DpadRight"):
            self.posture_offset["roll"] = min(
                self.posture_offset["roll"] + 0.05, 0.30)
            logger.debug(f"[BUTTONS] D-Right → roll={self.posture_offset['roll']:.2f}")

        # --- Bumpers: pitch nudges ---
        if rising("L1"):
            self.posture_offset["pitch"] = min(
                self.posture_offset["pitch"] + 0.05, 0.30)
            logger.debug(f"[BUTTONS] L1 → pitch={self.posture_offset['pitch']:.2f}")
        if rising("R1"):
            self.posture_offset["pitch"] = max(
                self.posture_offset["pitch"] - 0.05, -0.30)
            logger.debug(f"[BUTTONS] R1 → pitch={self.posture_offset['pitch']:.2f}")

        # --- Triggers: speed scaling ---
        if held("L2") and held("R2"):
            self.speed_scale = 0.3  # precision wins
        elif held("L2"):
            self.speed_scale = 0.3
        elif held("R2"):
            self.speed_scale = 1.0
        else:
            self.speed_scale = 0.6

        # --- R2 double-tap: gait cycle ---
        if rising("R2"):
            if (now_time - self._r2_tap_time) < 0.4:
                # Double-tap detected → cycle gait
                gaits = ["walk", "trot", "run"]
                idx = gaits.index(self.gait_preset)
                self.gait_preset = gaits[(idx + 1) % len(gaits)]
                logger.info(f"[BUTTONS] R2 double-tap → gait={self.gait_preset}")
                if self.forge_env is not None:
                    try:
                        self.forge_env.set_gait_preset(self.gait_preset)
                    except Exception:
                        pass
                self._r2_tap_time = 0.0  # reset so triple-tap doesn't re-trigger
            else:
                self._r2_tap_time = now_time

        # --- Face buttons: skill commands ---
        if rising("A"):
            self.active_skill = "stand"
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            logger.info("[BUTTONS] A → stand")
        if rising("B"):
            self.active_skill = "sit"
            logger.info("[BUTTONS] B → sit")
        if rising("X"):
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            self.active_skill = None
            logger.info("[BUTTONS] X → reset posture")
        if rising("Y"):
            self.active_skill = "freeze"
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            logger.info("[BUTTONS] Y → emergency freeze")

        self._prev_buttons = dict(buttons)

    def _update_gamepad_command(self):
        if self.forge_env is None:
            return
        if self.command_source != "gamepad" or not self.deadman_active:
            self._zero_gamepad_command()
            return
        joy_data = self._pending_joystick
        if joy_data is None:
            self._zero_gamepad_command()
            return
        sample_age = time.time() - getattr(self, "_pending_joystick_time", 0)
        if sample_age > self._command_stale_seconds:
            self._zero_gamepad_command()
            return
        if self._apply_gamepad_command(joy_data):
            self._command_update_zeroed = False

    def step_simulation(self):
        """Execute one simulation step with action blending."""
        # Check if auto-stopped
        if self.auto_stopped:
            return

        # Capture pending joystick data (will be applied inside _step_forge_env
        # after get_observations() to avoid GPU sync stall from torch.tensor()).
        _joy_data = None
        if (not self._decouple_command_updates and self._pending_joystick is not None
                and self.forge_env is not None):
            if self.command_source == "gamepad" and self.deadman_active:
                # Apply latest joystick every tick (Genesis expects per-step external command)
                # Drop stale samples to avoid “stuck” velocity commands if web server dies.
                sample_age = time.time() - getattr(self, "_pending_joystick_time", 0)
                if sample_age <= self._command_stale_seconds:
                    _joy_data = self._pending_joystick
                    # Throttle log noise but keep observability
                    if not hasattr(self, '_sync_joy_log_t'):
                        self._sync_joy_log_t = 0.0
                    if sample_age < 0.05 and (time.time() - self._sync_joy_log_t) > 1.0:
                        logger.info(
                            f"[SYNC-JOY] applying joystick: lY={_joy_data.get('leftStickY', 0):.3f} "
                            f"lX={_joy_data.get('leftStickX', 0):.3f} rX={_joy_data.get('rightStickX', 0):.3f}"
                        )
                        self._sync_joy_log_t = time.time()
                else:
                    # Stale input; clear to prevent stuck command
                    _joy_data = None

        # genesis-forge path
        if self.forge_env is not None:
            self._step_forge_env(_joy_data)
            return

        # Get teleop action from bridge
        teleop_action = self.teleop_bridge.get_action()
        
        # Get policy action if available
        policy_action = None
        confidence = None
        if self.policy is not None and self.current_obs is not None:
            try:
                policy_action = self.policy(self.current_obs)
                # Could extract confidence from policy
                confidence = 1.0
            except Exception as e:
                logger.error(f"Policy error: {e}")
        
        # Blend actions
        deadman_held = self.teleop_bridge.is_deadman_active()
        applied_action, blend_log = self.router.blend(
            teleop_action,
            policy_action,
            deadman_held=deadman_held,
            confidence=confidence
        )
        
        # Apply safety clamps
        clamped_action, safety_flags = self.action_spec.clamp_action(
            applied_action,
            self.config.dt
        )
        
        # Anomaly detection
        anomalies = self.anomaly_detector.check_action(clamped_action, self.config.dt)
        if anomalies:
            safety_flags.update(anomalies)
        
        # Check for auto-stop
        if self.anomaly_detector.update(anomalies):
            self._handle_anomaly_autostop(anomalies)
            return
        
        # Step Genesis simulation
        if self.scene is not None and self.scene_built:
            try:
                # Apply action to environment
                # self.env.step(clamped_action)
                self.scene.step()
            except Exception as e:
                logger.error(f"Simulation step error: {e}")
        
        # Log transition if enabled
        if self.config.enable_logging and self.mode in ["teleop_record", "hil_blend"]:
            self.log_transition(
                obs=self.current_obs,
                action_policy=policy_action,
                action_teleop=teleop_action,
                blend_alpha=blend_log["blend_alpha"],
                applied_action=clamped_action,
                reward=0.0,  # Would come from env
                done=False,
                goal=self.current_goal,
                actor_tag=blend_log["actor_tag"],
                safety_flags=safety_flags,
                confidence=confidence,
                anomalies=anomalies
            )
        
        self.step_count += 1
        self.fps_counter += 1
        
        # Update FPS every second
        now = time.time()
        if now - self.fps_time >= 1.0:
            self.fps = self.fps_counter
            self.fps_counter = 0
            self.fps_time = now

    def _emit_controller_vibration(self, payload: Dict[str, Any], min_interval_s: float = 0.5) -> bool:
        """Emit a haptic/vibration event to connected SDR_OS clients."""
        if not self.sio or not getattr(self.sio, "connected", False):
            return False
        now = time.time()
        if now - self._last_anomaly_vibration_time < min_interval_s:
            return False
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return False
        loop.create_task(self.sio.emit("controller_vibration", payload))
        self._last_anomaly_vibration_time = now
        return True

    def _handle_anomaly_autostop(self, anomalies: Dict[str, bool]) -> None:
        """Handle auto-stop due to anomalies (safety crash)."""
        self.auto_stopped = True
        self.paused = True
        self.router.force_alpha_zero(reason="anomaly")
        self._emit_controller_vibration({
            "reason": "anomaly_autostop",
            "duration": 350,
            "strongMagnitude": 0.9,
            "weakMagnitude": 0.4,
            "anomalies": list(anomalies.keys()) if anomalies else [],
        })
        logger.error("Auto-stop triggered - simulation paused")
    
    def _step_forge_env(self, joy_data=None):
        """Step the genesis-forge environment."""
        try:
            import torch
            # Fine-grained timing (reported every N steps)
            if not hasattr(self, '_ft_obs'):
                self._ft_obs = 0.0
                self._ft_policy = 0.0
                self._ft_joy = 0.0
                self._ft_step = 0.0
                self._ft_count = 0
            ft_interval = max(int(self.config.ft_log_interval), 1)

            # [DIAG] Periodic diagnostics (every 500 steps)
            if not hasattr(self, '_diag_step_count'):
                self._diag_step_count = 0
            self._diag_step_count += 1
            diag_log = (self._diag_step_count <= 3) or (self._diag_step_count % 500 == 0)

            # Determine which policy to use (skill overrides locomotion)
            active_skill = self.active_skill
            if active_skill != self._last_active_skill:
                if self.forge_env is not None and getattr(self.forge_env, "gait_command_manager", None) is not None:
                    try:
                        if active_skill is None:
                            self.forge_env.gait_command_manager.clear_fixed_gait()
                        else:
                            self.forge_env.gait_command_manager.set_fixed_gait("walk", period=0.8, clearance=0.05)
                    except Exception:
                        pass
                self._last_active_skill = active_skill
            skill_policy = None
            if active_skill is not None:
                if self._ensure_skill_policy_loaded(active_skill):
                    skill_policy = self.skill_policies.get(active_skill)
                if skill_policy is None:
                    logger.debug(f"[SKILL] No policy loaded for {active_skill}; falling back to locomotion")

            policy_to_use = skill_policy or self.policy

            # Get actions from policy or use zeros
            if policy_to_use is not None:
                _t0 = time.time()
                obs = self.forge_env.get_observations()
                obs_td = self.forge_env.extras.get("observations", obs)
                self._ft_obs += time.time() - _t0

                # Apply joystick AFTER get_observations() so torch.tensor()
                # Update command buffer (read by VelocityCommandManager external_controller)
                if active_skill is None and joy_data is not None:
                    _t0 = time.time()
                    if self._apply_gamepad_command(joy_data):
                        self._command_update_zeroed = False
                    self._ft_joy += time.time() - _t0
                elif active_skill is not None:
                    # Skill mode: zero velocity commands to avoid locomotion drift
                    self._zero_gamepad_command()

                if diag_log:
                    policy_tag = f"skill:{active_skill}" if skill_policy is not None else "locomotion"
                    logger.info(f"[DIAG] step={self._diag_step_count} policy=LOADED({policy_tag}) obs_type={type(obs).__name__} obs_td_type={type(obs_td).__name__}")
                    if hasattr(obs_td, 'keys'):
                        logger.info(f"[DIAG]   obs_td keys={list(obs_td.keys())}, shapes={{ k: tuple(obs_td[k].shape) for k in obs_td.keys() }}")
                    else:
                        logger.info(f"[DIAG]   obs_td shape={tuple(obs_td.shape) if hasattr(obs_td, 'shape') else 'N/A'}")

                _t0 = time.time()
                with torch.no_grad():
                    actions = policy_to_use(obs_td)
                self._ft_policy += time.time() - _t0

                if diag_log:
                    action_abs_mean = actions[0].abs().mean().item()
                    action_max = actions[0].abs().max().item()
                    logger.info(f"[DIAG]   actions shape={tuple(actions.shape)} abs_mean={action_abs_mean:.4f} max={action_max:.4f}")
            else:
                if joy_data is not None:
                    _t0 = time.time()
                    if self._apply_gamepad_command(joy_data):
                        self._command_update_zeroed = False
                    self._ft_joy += time.time() - _t0
                num_actions = self.forge_env.action_manager.num_actions
                actions = torch.zeros(
                    (self.forge_env.num_envs, num_actions),
                    device=gs.device,
                )
                if diag_log:
                    logger.info(f"[DIAG] step={self._diag_step_count} policy=NONE -> zero actions ({num_actions} dims)")

            _t0 = time.time()
            obs, reward, terminated, truncated, extras = self.forge_env.step(actions)
            self._ft_step += time.time() - _t0

            self.current_obs = obs
            self.step_count += 1
            self.fps_counter += 1

            # Fine-grained timing every ft_interval steps
            self._ft_count += 1
            if self._ft_count % ft_interval == 0:
                n = self._ft_count
                logger.info(
                    f"[FT] obs={self._ft_obs/n*1000:.1f}ms "
                    f"joy={self._ft_joy/n*1000:.1f}ms "
                    f"policy={self._ft_policy/n*1000:.1f}ms "
                    f"step={self._ft_step/n*1000:.1f}ms "
                    f"(n={n})"
                )
                self._ft_obs = 0.0
                self._ft_joy = 0.0
                self._ft_policy = 0.0
                self._ft_step = 0.0
                self._ft_count = 0

            # Cache velocity command and action AFTER step (GPU work complete)
            try:
                vel_cmd = self.forge_env.get_velocity_command()
                self._cached_velocity_cmd = np.array([
                    vel_cmd.get("lin_vel_x", 0.0),
                    vel_cmd.get("lin_vel_y", 0.0),
                    vel_cmd.get("ang_vel_z", 0.0),
                ])
            except Exception:
                self._cached_velocity_cmd = None
            self._cached_action_np = actions[0].detach().cpu().numpy()

            # Deferred reward/episode tracking: accumulate on GPU, read infrequently
            if not hasattr(self, '_deferred_reward'):
                self._deferred_reward = None
                self._deferred_done_any = None
            if reward is not None:
                self._deferred_reward = reward
            if terminated is not None and truncated is not None:
                self._deferred_done_any = (terminated | truncated)

            if self.step_count % 50 == 0:
                # One sync every 50 steps (~1 per second at 50Hz)
                if self._deferred_reward is not None:
                    self.total_reward = float(self._deferred_reward[0])
                if self._deferred_done_any is not None and self._deferred_done_any.any().item():
                    self.episode_count += 1
                    self.episode_rewards.append(self.total_reward)
                    self.total_reward = 0.0

            # Update FPS every second
            now = time.time()
            if now - self.fps_time >= 1.0:
                self.fps = self.fps_counter
                self.fps_counter = 0
                self.fps_time = now

        except Exception as e:
            import traceback
            if not hasattr(self, '_step_error_count'):
                self._step_error_count = 0
            self._step_error_count += 1
            if self._step_error_count <= 5:
                logger.error(f"[DIAG] Forge env step error #{self._step_error_count}: {e}\n{traceback.format_exc()}")
            elif self._step_error_count == 6:
                logger.error(f"[DIAG] Suppressing further step errors (total so far: {self._step_error_count})")

    def log_transition(self, **kwargs):
        """Log a transition for training data."""
        self.episode_data.append(kwargs)
        
        # Auto-save if buffer gets too large (prevent memory overflow)
        MAX_BUFFER_SIZE = 10000  # ~10k transitions
        if len(self.episode_data) >= MAX_BUFFER_SIZE:
            logger.warning(f"Episode buffer reached {MAX_BUFFER_SIZE} transitions, auto-saving...")
            self.save_episode()
    
    def save_episode(self, format: str = "npz"):
        """
        Save episode data to disk.
        
        Args:
            format: Output format ('npz' or 'parquet')
        """
        if not self.episode_data:
            return
        
        log_dir = Path(self.config.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # Convert episode data to arrays
        data = {}
        for key in self.episode_data[0].keys():
            values = [t[key] for t in self.episode_data]
            if values and values[0] is not None:
                if isinstance(values[0], np.ndarray):
                    data[key] = np.stack([v if v is not None else np.zeros_like(values[0]) for v in values])
                elif isinstance(values[0], dict):
                    # Flatten dict fields (e.g., safety_flags, anomalies)
                    for sub_key in values[0].keys():
                        sub_values = [v.get(sub_key, False) if v else False for v in values]
                        data[f"{key}_{sub_key}"] = np.array(sub_values)
                else:
                    data[key] = np.array([v if v is not None else 0 for v in values])
        
        if format == "parquet":
            try:
                import pandas as pd
                filename = log_dir / f"episode_{timestamp}_{self.episode_count}.parquet"
                
                # Flatten arrays for DataFrame
                df_data = {}
                for key, value in data.items():
                    if value.ndim == 1:
                        df_data[key] = value
                    elif value.ndim == 2:
                        # Flatten 2D arrays to separate columns
                        for i in range(value.shape[1]):
                            df_data[f"{key}_{i}"] = value[:, i]
                    else:
                        # Skip higher dimensional data
                        continue
                
                df = pd.DataFrame(df_data)
                df.to_parquet(filename, compression='snappy')
                logger.info(f"Saved episode to {filename} (parquet)")
                
            except ImportError:
                logger.warning("pandas/pyarrow not available, falling back to npz")
                format = "npz"
        
        if format == "npz":
            filename = log_dir / f"episode_{timestamp}_{self.episode_count}.npz"
            np.savez_compressed(filename, **data)
            logger.info(f"Saved episode to {filename}")
        
        self.episode_data = []
        self.episode_count += 1
    
    async def handle_websocket(self, websocket):
        """Handle a WebSocket client connection."""
        self.ws_clients.add(websocket)
        client_id = str(id(websocket))
        logger.info(f"WebSocket client connected ({client_id}). Total: {len(self.ws_clients)}")

        try:
            # Keep connection alive and handle messages
            while True:
                try:
                    # Wait for message with timeout to keep connection alive
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    # Handle incoming messages
                    if isinstance(message, str):
                        try:
                            data = json.loads(message)
                            await self._handle_ws_message(websocket, client_id, data)
                        except json.JSONDecodeError:
                            logger.warning(f"Invalid JSON message from {client_id}")
                    else:
                        logger.debug(f"Received binary WebSocket message: {len(message)} bytes")
                except asyncio.TimeoutError:
                    # Timeout is normal - just keep connection alive
                    continue
                except websockets.exceptions.ConnectionClosed:
                    break
        except Exception as e:
            logger.error(f"WebSocket error: {e}")
        finally:
            self.ws_clients.discard(websocket)
            await self.stream_fanout.remove_client(client_id)
            logger.info(f"WebSocket client disconnected ({client_id}). Total: {len(self.ws_clients)}")

    async def _handle_ws_message(self, websocket, client_id: str, data: dict):
        """Handle parsed JSON message from WebSocket client."""
        msg_type = data.get('type')

        if msg_type == 'negotiate':
            # Codec negotiation
            supports_h264 = data.get('supports_h264', False)
            h264_codec = data.get('h264_codec', 'avc1.42E01E')

            # Determine actual codec to use (H.264 only if NVENC available and client supports it)
            has_nvenc = self.gpu_pipeline is not None and self.gpu_pipeline.has_nvenc
            stream_codec = 'h264' if (supports_h264 and has_nvenc) else 'jpeg'

            # Send negotiation response
            await websocket.send(json.dumps({
                'type': 'negotiated',
                'stream_codec': stream_codec,
                'h264_codec': h264_codec if stream_codec == 'h264' else None
            }))

            # Add to fanout
            self.stream_fanout.add_client(client_id, websocket, stream_codec == 'h264', h264_codec)
            logger.info(f"Client {client_id} negotiated codec: {stream_codec}")
        else:
            logger.debug(f"Unhandled message type from {client_id}: {msg_type}")
    
    async def broadcast_frame(self, frame_bytes: Optional[bytes]):
        """Broadcast JPEG frame to all WebSocket clients."""
        if frame_bytes is None:
            logger.debug("broadcast_frame: Skipping None frame")
            self.frames_failed += 1
            return
            
        if not self.ws_clients:
            logger.debug("broadcast_frame: No WebSocket clients connected")
            return
        
        # Track frame statistics
        frame_size = len(frame_bytes)
        self.frames_sent += 1
        self.total_frame_bytes += frame_size
        
        # Snapshot clients to avoid mutation during iteration
        clients = list(self.ws_clients)

        # Send binary frame to all clients
        send_start = time.time()
        results = await asyncio.gather(
            *[client.send(frame_bytes) for client in clients],
            return_exceptions=True
        )
        self._timing_acc["broadcast"] += time.time() - send_start
        self._timing_acc["broadcast_count"] += 1

        # Remove failed clients
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                self.ws_clients.discard(client)
                self.frames_failed += 1
                try:
                    await client.close()
                except Exception:
                    pass
    
    async def _frame_sender_loop(self):
        """Encode BGR frames from a queue and broadcast without blocking the sim loop."""
        if self._frame_queue is None:
            return
        while self.running:
            try:
                frame_bgr = await self._frame_queue.get()
            except asyncio.CancelledError:
                break
            try:
                # Check if any clients need frames
                has_h264 = self.stream_fanout.has_h264_clients()
                has_jpeg = self.stream_fanout.has_jpeg_clients()
                has_legacy = bool(self.ws_clients)

                # Skip encoding if no viewers
                if not has_h264 and not has_jpeg and not has_legacy:
                    continue

                encode_start = time.time()
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                self._frame_id_counter += 1
                frame_id = self._frame_id_counter

                # H.264 to H.264 clients via fanout
                if has_h264 and self.gpu_pipeline is not None and self.gpu_pipeline.has_nvenc:
                    if self.stream_fanout.should_force_keyframe():
                        self.gpu_pipeline.request_keyframe()
                    h264_bytes = self.gpu_pipeline.encode_h264_au(frame_rgb, frame_id)
                    if h264_bytes:
                        header = pack_frame_header(FrameType.H264, len(h264_bytes), frame_id, 0)
                        self.stream_fanout.broadcast_h264(header + h264_bytes)

                # JPEG to JPEG clients via fanout
                if has_jpeg:
                    if self.gpu_pipeline is not None:
                        jpeg_bytes = self.gpu_pipeline.encode_jpeg_numpy(frame_rgb, quality=self.config.jpeg_quality)
                    else:
                        _, jpeg_buf = cv2.imencode(
                            '.jpg', frame_bgr,
                            [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
                        )
                        jpeg_bytes = jpeg_buf.tobytes()
                    header = pack_frame_header(FrameType.JPEG, len(jpeg_bytes), frame_id, 0)
                    self.stream_fanout.broadcast_jpeg(header + jpeg_bytes)

                # Legacy broadcast for non-negotiated clients (raw JPEG, no header)
                if has_legacy:
                    # Only encode if not already done above
                    if not has_jpeg:
                        if self.gpu_pipeline is not None:
                            jpeg_bytes = self.gpu_pipeline.encode_jpeg_numpy(frame_rgb, quality=self.config.jpeg_quality)
                        else:
                            _, jpeg_buf = cv2.imencode(
                                '.jpg', frame_bgr,
                                [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
                            )
                            jpeg_bytes = jpeg_buf.tobytes()
                    await self.broadcast_frame(jpeg_bytes)

                self._timing_acc["encode"] += time.time() - encode_start
                self._timing_acc["encode_count"] += 1
            except Exception as e:
                logger.error(f"[ENCODE-LOOP] {type(e).__name__}: {e}")
            finally:
                self._frame_queue.task_done()
    
    async def broadcast_metrics(self, metrics: Dict):
        """Broadcast JSON metrics to all WebSocket clients."""
        if not self.ws_clients:
            return
        
        clients = list(self.ws_clients)
        message = json.dumps(metrics)
        results = await asyncio.gather(
            *[client.send(message) for client in clients],
            return_exceptions=True
        )
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                self.ws_clients.discard(client)
                try:
                    await client.close()
                except Exception:
                    pass
    
    async def _handle_webrtc_offer(self, request: "aiohttp.web.Request") -> "aiohttp.web.Response":
        """Handle WebRTC offer: create PC, add track, return answer."""
        if not _aiortc or GenesisWebRTCVideoTrack is None or _aiohttp is None:
            r = web.json_response({"error": "WebRTC not available"}, status=503)
            r.headers["Access-Control-Allow-Origin"] = "*"
            return r
        try:
            body = await request.json()
            offer_sdp = body.get("sdp")
            offer_type = body.get("type", "offer")
            if not offer_sdp:
                r = web.json_response({"error": "Missing sdp"}, status=400)
                r.headers["Access-Control-Allow-Origin"] = "*"
                return r
            pc = RTCPeerConnection()
            await pc.setRemoteDescription(RTCSessionDescription(sdp=offer_sdp, type=offer_type))
            pc.addTrack(GenesisWebRTCVideoTrack(self))
            # Prefer H264 codec for higher quality at same bitrate
            try:
                caps = _get_rtc_capabilities("video").codecs
                h264_codecs = [c for c in caps if c.mimeType == "video/H264"]
                other_codecs = [c for c in caps if c.mimeType != "video/H264"]
                if h264_codecs:
                    for t in pc.getTransceivers():
                        if t.kind == "video":
                            t.setCodecPreferences(h264_codecs + other_codecs)
            except Exception:
                logger.debug("Could not set server codec preferences, using defaults")
            # Apply target bitrate to video senders
            try:
                target_bps = int(self.config.webrtc_bitrate_kbps) * 1000
                for sender in pc.getSenders():
                    if sender.track and sender.track.kind == "video":
                        params = sender.getParameters()
                        if not params.encodings:
                            params.encodings = [{}]
                        for enc in params.encodings:
                            enc["maxBitrate"] = target_bps
                        await sender.setParameters(params)
            except Exception:
                logger.debug("Could not set WebRTC bitrate; continuing with defaults")
            # Browser gathers all ICE candidates before sending the offer,
            # so they're already embedded in the SDP — no need for addIceCandidate.
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            # Wait for ICE gathering to complete
            while pc.iceGatheringState != "complete":
                await asyncio.sleep(0.1)
            # Extract gathered ICE candidates from local description
            candidates = []
            for line in pc.localDescription.sdp.splitlines():
                if line.startswith("a=candidate:"):
                    candidates.append({
                        "candidate": line[2:],  # strip "a=" prefix
                        "sdpMid": "0",
                        "sdpMLineIndex": 0,
                    })
            answer_sdp = pc.localDescription.sdp
            answer_type = pc.localDescription.type
            resp = web.json_response({
                "type": answer_type,
                "sdp": answer_sdp,
                "candidates": candidates,
            })
            resp.headers["Access-Control-Allow-Origin"] = "*"
            return resp
        except Exception as e:
            logger.exception("WebRTC offer failed: %s", e)
            r = web.json_response({"error": str(e)}, status=500)
            r.headers["Access-Control-Allow-Origin"] = "*"
            return r
    
    def _create_webrtc_app(self) -> "aiohttp.web.Application":
        """Create aiohttp app for WebRTC signaling with CORS."""
        if _aiohttp is None:
            return None
        app = web.Application()
        app.router.add_post("/offer", self._handle_webrtc_offer)
        async def options_offer(request):
            r = web.Response()
            r.headers["Access-Control-Allow-Origin"] = "*"
            r.headers["Access-Control-Allow-Methods"] = "POST, OPTIONS"
            r.headers["Access-Control-Allow-Headers"] = "Content-Type"
            return r
        app.router.add_route("OPTIONS", "/offer", options_offer)
        return app
    
    def setup_socketio(self):
        """Set up Socket.io client connection to SDR_OS."""
        if socketio is None:
            logger.warning("Socket.io not available - install python-socketio")
            return
        
        logger.info("Setting up Socket.io client...")
        self.sio = socketio.AsyncClient(
            logger=False,
            engineio_logger=False,
            reconnection=True,
            reconnection_attempts=0,           # infinite
            reconnection_delay=1.0,
            reconnection_delay_max=5.0,
        )

        @self.sio.on('connect')
        async def on_connect():
            logger.info("Connected to SDR_OS server via Socket.io")
            # Identify as Genesis bridge with full status
            init_status = self.get_initialization_status()
            await self.sio.emit('genesis_identify', {
                'type': 'genesis_bridge',
                'is_mock_mode': init_status['is_mock_mode'],
                'genesis_available': init_status['genesis_module_available'],
                'genesis_version': init_status['genesis_version'],
            })
            # Send initial env info
            await self.send_env_info()
            # Send initialization status
            await self.sio.emit('genesis_init_status', init_status)
            logger.info(f"Sent initialization status: mock_mode={init_status['is_mock_mode']}")
            
            # Send robot list on connect
            import sys
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).parent))
            try:
                from robot_registry import GenesisROSRobotRegistry
                configs_path = Path(__file__).parent.parent / "configs"
                registry = GenesisROSRobotRegistry(str(configs_path))
                robot_list = registry.get_robot_list()
                logger.info(f"Sending robot list: {len(robot_list)} robots")
                await self.sio.emit('genesis_robot_list', {'robots': robot_list})
            except Exception as e:
                logger.error(f"Failed to load robot list: {e}")
        
        @self.sio.on('disconnect')
        async def on_disconnect():
            logger.info("Disconnected from SDR_OS server")

        @self.sio.on('connect_error')
        async def on_connect_error(data):
            logger.error(f"Socket.io connect error: {data}")

        @self.sio.on('reconnect_attempt')
        async def on_reconnect_attempt(attempt):
            logger.warning(f"Socket.io reconnect attempt: {attempt}")

        @self.sio.on('reconnect_error')
        async def on_reconnect_error(data):
            logger.error(f"Socket.io reconnect error: {data}")

        @self.sio.on('reconnect_failed')
        async def on_reconnect_failed():
            logger.error("Socket.io reconnect failed")
        
        # Joystick + button input is handled by the background gamepad thread
        # (sync socketio.Client in _start_gamepad_thread).  The async handlers
        # below are no-ops to prevent event-loop backlog from tanking FPS.

        @self.sio.on('controller_joystick_state')
        async def on_joystick(data):
            pass  # handled by sync gamepad thread → _pending_joystick

        @self.sio.on('controller_button_states')
        async def on_buttons(data):
            pass  # deadman bypassed; buttons not needed during sim
        
        @self.sio.on('genesis_set_mode')
        async def on_set_mode(data):
            self.mode = data.get('mode', 'teleop_record')
            logger.info(f"Mode set to: {self.mode}")
        
        @self.sio.on('genesis_reset')
        async def on_reset(data):
            logger.info("Resetting environment")
            if self.forge_env is not None:
                obs, _ = self.forge_env.reset()
                self.current_obs = obs
                self.step_count = 0
                self.total_reward = 0.0
            else:
                self.router.reset()
                self.action_spec.reset()
            if self.episode_data:
                self.save_episode()
            # Reset script status to IDLE
            self.script_status = ScriptStatus.IDLE
            self.current_script_name = None
            self.script_error = None
            await self._emit_script_status()
        
        @self.sio.on('genesis_pause')
        async def on_pause(data):
            self.paused = data.get('paused', True)
            # Update script status based on pause state
            if self.script_status == ScriptStatus.RUNNING and self.paused:
                self.script_status = ScriptStatus.PAUSED
                await self._emit_script_status()
            elif self.script_status == ScriptStatus.PAUSED and not self.paused:
                self.script_status = ScriptStatus.RUNNING
                await self._emit_script_status()
            logger.info(f"Simulation paused: {self.paused}")
        
        @self.sio.on('genesis_set_goal')
        async def on_set_goal(data):
            self.current_goal = {
                'position': data.get('position', [0, 0, 0]),
                'orientation': data.get('orientation', [1, 0, 0, 0]),
                'waypoints': data.get('waypoints', [])
            }
            logger.info(f"Goal set: {self.current_goal}")
        
        @self.sio.on('genesis_set_alpha')
        async def on_set_alpha(data):
            alpha = data.get('alpha', 0.0)
            self.router.set_target_alpha(alpha)
            logger.info(f"Target alpha set to: {alpha}")

        @self.sio.on('genesis_estop')
        async def on_estop(data=None):
            logger.warning("E-STOP triggered from UI")
            self.paused = True
            self.auto_stopped = True
            if self.forge_env is not None:
                # Zero out velocity commands
                self._zero_gamepad_command()
            else:
                self.router.force_alpha_zero(reason="estop")
            # Emit status update
            await self.sio.emit('genesis_training_metrics', self.get_training_metrics())

        @self.sio.on('genesis_load_policy')
        async def on_load_policy(data):
            checkpoint_dir = data.get('checkpoint_dir', '')
            model_file = data.get('model_file', None)
            if not checkpoint_dir:
                # Default path
                checkpoint_dir = str(Path(__file__).parent.parent / "rl" / "checkpoints" / "go2-locomotion")
            checkpoint_name = Path(checkpoint_dir).name
            await self.sio.emit('genesis_policy_load_status', {
                'status': 'loading',
                'checkpoint_dir': checkpoint_dir,
                'checkpoint_name': checkpoint_name,
            })
            success = self.load_policy(checkpoint_dir, model_file=model_file)
            if success:
                await self.sio.emit('genesis_policy_load_status', {
                    'status': 'loaded',
                    'checkpoint_dir': checkpoint_dir,
                    'checkpoint_name': checkpoint_name,
                })
            else:
                await self.sio.emit('genesis_policy_load_status', {
                    'status': 'error',
                    'checkpoint_dir': checkpoint_dir,
                    'checkpoint_name': checkpoint_name,
                    'error': f'Failed to load policy from {checkpoint_dir}',
                })
            await self.sio.emit('genesis_training_metrics', self.get_training_metrics())
            logger.info(f"Policy load {'succeeded' if success else 'failed'}: {checkpoint_dir}")

        @self.sio.on('genesis_list_policies')
        async def on_list_policies(data=None):
            """Scan and return available policy checkpoints."""
            logger.info("Scanning for available policies")
            policies = self.scan_policies()
            await self.sio.emit('genesis_policy_list', {'policies': policies})

        @self.sio.on('genesis_set_command_source')
        async def on_set_command_source(data):
            new_source = data.get('source', 'gamepad')
            if new_source not in ('gamepad', 'ros'):
                logger.warning(f"Invalid command source: {new_source}")
                return
            old_source = self.command_source
            self.command_source = new_source
            # Zero velocity on source switch to prevent stale commands
            if self.forge_env is not None and old_source != new_source:
                self._zero_gamepad_command()
            logger.info(f"Command source switched: {old_source} -> {new_source}")
            await self.sio.emit('genesis_training_metrics', self.get_training_metrics())

        @self.sio.on('genesis_load_robot')
        async def on_load_robot(data):
            # #region agent log
            import time
            log_path = Path(__file__).parent.parent / ".cursor" / "debug.log"
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_entry", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:entry", "message": "Load robot event received", "data": {"event_data": data}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
            except: pass
            # #endregion
            
            robot_name = data.get('robot_name', '')
            logger.info(f"Load robot requested: {robot_name}")
            
            # #region agent log
            try:
                with open(log_path, "a") as f:
                    f.write(json.dumps({"id": "log_load_robot_name", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:name", "message": "Extracted robot name", "data": {"robot_name": robot_name, "has_scene": self.scene is not None, "has_robot": self.robot is not None}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "E"}) + "\n")
            except: pass
            # #endregion
            
            # Actually load the robot using load_robot method
            if robot_name:
                # #region agent log
                try:
                    with open(log_path, "a") as f:
                        f.write(json.dumps({"id": "log_load_robot_attempt", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:loading", "message": "Attempting to load robot", "data": {"robot_name": robot_name}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
                except: pass
                # #endregion
                
                try:
                    # Import registry to get URDF path
                    from robot_registry import GenesisROSRobotRegistry
                    configs_path = Path(__file__).parent.parent / "configs"
                    registry = GenesisROSRobotRegistry(str(configs_path))
                    robot_config = registry.get_robot(robot_name)
                    
                    if robot_config:
                        # #region agent log
                        try:
                            with open(log_path, "a") as f:
                                f.write(json.dumps({"id": "log_load_robot_config_found", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:config", "message": "Robot config found", "data": {"robot_name": robot_name, "urdf_path": robot_config.urdf_path}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
                        except: pass
                        # #endregion
                        
                        success = self.load_robot(
                            robot_config.urdf_path,
                            position=robot_config.initial_position,
                            orientation=robot_config.initial_orientation
                        )
                        
                        # #region agent log
                        try:
                            with open(log_path, "a") as f:
                                f.write(json.dumps({"id": "log_load_robot_result", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:result", "message": "Robot load completed", "data": {"robot_name": robot_name, "success": success}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
                        except: pass
                        # #endregion
                        
                        # Update current robot name and emit event
                        if success:
                            self.current_robot_name = robot_name
                            await self.sio.emit('genesis_robot_loaded', {
                                'robot_name': robot_name,
                                'label': robot_config.label or robot_name,
                                'memory_estimate': robot_config.memory_estimate
                            })
                            # Also emit memory estimate as separate event for UI update
                            if robot_config.memory_estimate:
                                await self.sio.emit('genesis_memory_estimate', robot_config.memory_estimate)
                            logger.info(f"Robot loaded and event emitted: {robot_name}")
                        else:
                            await self.sio.emit('genesis_robot_load_failed', {
                                'robot_name': robot_name,
                                'error': 'Failed to load robot'
                            })
                    else:
                        # #region agent log
                        try:
                            with open(log_path, "a") as f:
                                f.write(json.dumps({"id": "log_load_robot_not_found", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:not_found", "message": "Robot config not found in registry", "data": {"robot_name": robot_name}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "E"}) + "\n")
                        except: pass
                        # #endregion
                        logger.error(f"Robot not found in registry: {robot_name}")
                        
                except Exception as e:
                    # #region agent log
                    try:
                        with open(log_path, "a") as f:
                            f.write(json.dumps({"id": "log_load_robot_exception", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:exception", "message": "Exception during robot load", "data": {"robot_name": robot_name, "error": str(e), "error_type": type(e).__name__}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "D"}) + "\n")
                    except: pass
                    # #endregion
                    logger.error(f"Failed to load robot: {e}")
            else:
                # #region agent log
                try:
                    with open(log_path, "a") as f:
                        f.write(json.dumps({"id": "log_load_robot_empty", "timestamp": time.time() * 1000, "location": "bridge_server.py:on_load_robot:empty", "message": "Empty robot name received", "data": {"robot_name": robot_name}, "sessionId": "debug-session", "runId": "run1", "hypothesisId": "E"}) + "\n")
                except: pass
                # #endregion
        
        @self.sio.on('genesis_unload_robot')
        async def on_unload_robot(data=None):
            """Unload the current robot and reset to empty scene."""
            logger.info("Unload robot requested")
            
            previous_robot = self.current_robot_name
            success = self.unload_robot()
            
            if success:
                await self.sio.emit('genesis_robot_unloaded', {
                    'previous_robot': previous_robot,
                    'success': True
                })
                logger.info(f"Robot unloaded: {previous_robot}")
            else:
                await self.sio.emit('genesis_robot_unloaded', {
                    'previous_robot': previous_robot,
                    'success': False,
                    'error': 'Failed to unload robot'
                })
        
        @self.sio.on('genesis_scan_robots')
        async def on_scan_robots(data=None):
            """Scan and return available robots."""
            logger.info("Scanning for available robots")
            # Import registry and load from configs
            import sys
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).parent))
            from robot_registry import GenesisROSRobotRegistry
            
            # Load robots from configs directory
            configs_path = Path(__file__).parent.parent / "configs"
            registry = GenesisROSRobotRegistry(str(configs_path))
            
            robot_list = registry.get_robot_list()
            logger.info(f"Found {len(robot_list)} robots: {[r['name'] for r in robot_list]}")
            
            # Send to all connected clients
            await self.sio.emit('genesis_robot_list', {'robots': robot_list})
        
        @self.sio.on('genesis_get_robot_info')
        async def on_get_robot_info(data):
            """Get detailed info for a specific robot."""
            robot_name = data.get('robot_name', '')
            logger.info(f"Robot info requested: {robot_name}")
            
            import sys
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).parent))
            from robot_registry import GenesisROSRobotRegistry
            
            configs_path = Path(__file__).parent.parent / "configs"
            registry = GenesisROSRobotRegistry(str(configs_path))
            
            robot = registry.get_robot(robot_name)
            if robot:
                # Get full YAML config for detailed info
                robot_dict = robot.to_dict()
                
                # Add additional details from YAML
                if robot.gs_ros_config:
                    try:
                        import yaml
                        with open(robot.gs_ros_config) as f:
                            full_config = yaml.safe_load(f)
                            robot_dict['control'] = full_config.get('robot', {}).get('control', {})
                            robot_dict['sensors'] = full_config.get('sensors', [])
                            robot_dict['dof'] = robot.specs.get('dof') if robot.specs else None
                            robot_dict['mass'] = robot.specs.get('mass') if robot.specs else None
                    except Exception as e:
                        logger.error(f"Failed to load full config: {e}")
                
                await self.sio.emit('genesis_robot_info', robot_dict)
            else:
                logger.warning(f"Robot not found: {robot_name}")
        
        @self.sio.on('genesis_get_memory_estimate')
        async def on_get_memory_estimate(data):
            """Get memory estimate for a specific robot."""
            robot_name = data.get('robot_name', '')
            logger.info(f"Memory estimate requested: {robot_name}")
            
            import sys
            from pathlib import Path
            sys.path.insert(0, str(Path(__file__).parent))
            from robot_registry import GenesisROSRobotRegistry
            
            configs_path = Path(__file__).parent.parent / "configs"
            registry = GenesisROSRobotRegistry(str(configs_path))
            
            robot = registry.get_robot(robot_name)
            if robot and robot.memory_estimate:
                await self.sio.emit('genesis_memory_estimate', {
                    'robot_name': robot_name,
                    **robot.memory_estimate
                })
            else:
                # Return default/unknown estimate
                await self.sio.emit('genesis_memory_estimate', {
                    'robot_name': robot_name,
                    'vram_mb': 256,  # Base Genesis memory
                    'dram_mb': 128,
                    'breakdown': {},
                    'unknown': True
                })
        
        @self.sio.on('genesis_get_camera_list')
        async def on_get_camera_list(data=None):
            """Return list of available cameras."""
            logger.info("Camera list requested")
            # For now, return static list from config
            cameras = [
                {'name': 'main_camera', 'label': 'Main View'},
                {'name': 'wrist_camera', 'label': 'Wrist Camera'},
                {'name': 'training_camera', 'label': 'Training View'}
            ]
            await self.sio.emit('genesis_camera_list', {'cameras': cameras})
        
        @self.sio.on('genesis_get_init_status')
        async def on_get_init_status(data=None):
            """Return current initialization status."""
            init_status = self.get_initialization_status()
            await self.sio.emit('genesis_init_status', init_status)
            logger.debug(f"Sent init status on request: mock_mode={init_status['is_mock_mode']}")
        
        @self.sio.on('genesis_settings')
        async def on_settings_update(data):
            """Update runtime settings (JPEG quality, FPS, resolution, metrics rate)."""
            changed = []

            if 'jpegQuality' in data:
                q = int(data['jpegQuality'])
                if 1 <= q <= 100 and q != self.config.jpeg_quality:
                    self.config.jpeg_quality = q
                    changed.append(f"jpeg_quality={q}")

            if 'streamFps' in data:
                fps = int(data['streamFps'])
                if 1 <= fps <= 120 and fps != self.config.target_fps:
                    self.config.target_fps = fps
                    self.frame_interval = 1.0 / fps
                    changed.append(f"target_fps={fps}")

            if 'frameQueueSize' in data:
                try:
                    qsize = max(1, min(10, int(data['frameQueueSize'])))
                    if qsize != getattr(self.config, "frame_queue_size", 2):
                        self.config.frame_queue_size = qsize
                        changed.append(f"frame_queue_size={qsize}")
                        await self._restart_frame_sender(qsize)
                except (TypeError, ValueError):
                    logger.warning(f"Invalid frameQueueSize value: {data['frameQueueSize']}")

            if 'trainingMetricsRate' in data:
                rate = int(data['trainingMetricsRate'])
                if 1 <= rate <= 60 and rate != self.config.metrics_rate:
                    self.config.metrics_rate = rate
                    changed.append(f"metrics_rate={rate}")

            if 'dt' in data:
                try:
                    new_dt = float(data['dt'])
                    if 0.001 <= new_dt <= 0.1 and abs(new_dt - self.config.dt) > 1e-6:
                        self.config.dt = new_dt
                        changed.append(f"dt={new_dt}")
                except (TypeError, ValueError):
                    logger.warning(f"Invalid dt value: {data['dt']}")

            if 'webrtcBitrateKbps' in data:
                try:
                    br_kbps = int(data['webrtcBitrateKbps'])
                    if 100 <= br_kbps <= 100000 and br_kbps != self.config.webrtc_bitrate_kbps:
                        self.config.webrtc_bitrate_kbps = br_kbps
                        changed.append(f"webrtc_bitrate_kbps={br_kbps}")
                except (TypeError, ValueError):
                    logger.warning(f"Invalid webrtcBitrateKbps value: {data['webrtcBitrateKbps']}")

            if 'cameraRes' in data:
                try:
                    parts = str(data['cameraRes']).split('x')
                    w, h = int(parts[0]), int(parts[1])
                    if (w, h) != self.config.camera_res and 320 <= w <= 3840 and 180 <= h <= 2160:
                        self.config.camera_res = (w, h)
                        changed.append(f"camera_res={w}x{h}")
                        # Recreate camera with new resolution if scene exists
                        if self.scene is not None and self.camera is not None:
                            try:
                                old_pos = None
                                old_lookat = None
                                try:
                                    old_pos = list(self.camera.pos)
                                    old_lookat = list(self.camera.lookat)
                                except Exception:
                                    pass
                                self.camera = self.scene.add_camera(
                                    res=(w, h),
                                    pos=old_pos or [2.0, 2.0, 1.5],
                                    lookat=old_lookat or [0.0, 0.0, 0.5],
                                    fov=40,
                                    GUI=False
                                )
                                changed.append("camera_recreated")
                            except Exception as cam_err:
                                logger.error(f"Failed to recreate camera at {w}x{h}: {cam_err}")
                                changed.append(f"camera_recreate_failed: {cam_err}")
                except (ValueError, IndexError):
                    logger.warning(f"Invalid cameraRes format: {data['cameraRes']}")

            if changed:
                logger.info(f"Settings updated: {', '.join(changed)}")
                try:
                    await self.send_env_info()
                except Exception:
                    logger.debug("Failed to emit env_info after settings update")

        @self.sio.on('genesis_camera')
        async def on_camera_update(data):
            """Update camera position and lookat."""
            position = data.get('position')
            lookat = data.get('lookat')
            
            if self.camera is None:
                logger.debug("Camera update requested but camera is None")
                return
            
            try:
                if position and lookat:
                    # Update camera position and lookat
                    self.camera.set_pose(
                        pos=position,
                        lookat=lookat
                    )
                    logger.debug(f"Camera updated: pos={position}, lookat={lookat}")
                elif position:
                    # Just update position
                    self.camera.set_pose(pos=position)
                    logger.debug(f"Camera position updated: {position}")
            except Exception as e:
                logger.error(f"Error updating camera: {e}")
        
        @self.sio.on('genesis_load_script')
        async def on_load_script(data):
            script_content = data.get('script_content', '')
            script_name = data.get('script_name', 'simulation.py')
            
            # Set status to LOADING
            self.script_status = ScriptStatus.LOADING
            self.current_script_name = script_name
            self.script_error = None
            await self._emit_script_status()
            
            logger.info(f"Loading Genesis script: {script_name}")
            
            if not script_content:
                error_msg = "No script content provided"
                logger.error(error_msg)
                self.script_status = ScriptStatus.ERROR
                self.script_error = error_msg
                await self._emit_script_status()
                return
            
            # Check if Genesis is available
            if gs is None:
                error_msg = "Genesis module not available - cannot execute scripts in mock mode"
                logger.error(error_msg)
                self.script_status = ScriptStatus.ERROR
                self.script_error = error_msg
                await self._emit_script_status()
                return
            
            try:
                # Ensure Genesis is initialized before running user script
                # This is idempotent - calling init() again when already initialized is safe
                try:
                    # Check if Genesis is already initialized by attempting a safe operation
                    # If not initialized, this will fail and we'll initialize it
                    _ = gs.options
                    logger.debug("Genesis already initialized, proceeding with script")
                except Exception:
                    logger.info("Genesis not initialized, calling gs.init() before script execution")
                    try:
                        gs.init(backend=gs.gpu)
                        logger.info("Genesis initialized with GPU backend for script")
                    except Exception as gpu_err:
                        logger.warning(f"GPU backend failed: {gpu_err}, trying CPU")
                        gs.init(backend=gs.cpu)
                        logger.info("Genesis initialized with CPU backend for script")
                
                # Execute the script in a controlled namespace
                # The script can access:
                # - gs: the Genesis module (already initialized)
                # - scene: the current scene (if any)
                # - camera: the current camera (if any)
                # - bridge: reference to this bridge server
                script_globals = {
                    '__name__': '__main__',
                    '__file__': script_name,
                    'gs': gs,
                    'scene': self.scene,
                    'camera': self.camera,
                    'bridge': self
                }
                
                logger.info(f"Executing script with Genesis v{getattr(gs, '__version__', 'unknown')}")
                
                # Execute the script
                exec(script_content, script_globals)
                
                # If script created a scene, use it
                scene_loaded = False
                if 'scene' in script_globals and script_globals['scene']:
                    self.scene = script_globals['scene']
                    scene_loaded = True
                    logger.info("Scene loaded from script")
                else:
                    logger.warning("Script did not create a scene")
                
                # If script created a camera, use it
                camera_loaded = False
                if 'camera' in script_globals and script_globals['camera']:
                    self.camera = script_globals['camera']
                    camera_loaded = True
                    logger.info("Camera loaded from script")
                else:
                    logger.warning("Script did not create a camera")
                
                # Start simulation if not already running
                if not self.running:
                    self.running = True
                    logger.info("Simulation started from script")
                
                # Set status to RUNNING on success
                self.script_status = ScriptStatus.RUNNING
                self.script_error = None
                await self._emit_script_status()
                
                logger.info(f"Script {script_name} executed successfully (scene: {scene_loaded}, camera: {camera_loaded})")
                
            except Exception as e:
                error_msg = str(e)
                logger.error(f"Error executing script {script_name}: {error_msg}")
                import traceback
                traceback_str = traceback.format_exc()
                logger.error(traceback_str)
                
                # Set status to ERROR
                self.script_status = ScriptStatus.ERROR
                self.script_error = error_msg
                await self._emit_script_status()
    
    async def _emit_script_status(self):
        """Emit script status via Socket.io."""
        if self.sio and self.sio.connected:
            status_data = self.get_script_status()
            await self.sio.emit('genesis_script_status', status_data)
            logger.debug(f"Emitted script status: {status_data}")
    
    async def _emit_frame_stats(self):
        """Emit frame statistics via Socket.io."""
        if self.sio and self.sio.connected:
            stats = self.get_frame_statistics()
            await self.sio.emit('genesis_frame_stats', stats)
            # Reset stats timer
            self.frame_stats_time = time.time()
            # Reset counters for next interval
            self.frames_sent = 0
            self.frames_failed = 0
            self.total_frame_bytes = 0
    
    async def send_env_info(self):
        """Send environment info to SDR_OS."""
        if self.sio is None or not self.sio.connected:
            return
        
        env_info = {
            "num_envs": self.config.num_envs,
            "num_actions": self.action_spec.num_actions,
            "num_observations": 48,  # Would come from env
            "robot_name": "mock_robot",
            "dt": self.config.dt,
            "mode": self.mode,
            "camera_res": list(self.config.camera_res),
            "action_spec": {
                "segments": [
                    {
                        "name": seg.name,
                        "start": seg.start,
                        "size": seg.size,
                        "clip": [seg.clip_min, seg.clip_max]
                    }
                    for seg in self.action_spec.segments
                ]
            }
        }
        
        await self.sio.emit('genesis_env_info', env_info)
    
    async def main_loop(self):
        """Main simulation and streaming loop."""
        logger.info("Starting main loop...")

        # Timing diagnostics (configurable interval)
        timing_interval = max(self.config.timing_report_interval, 1.0)
        _timing_last_report = time.time()
        self._reset_timing_acc()

        while self.running:
            loop_start = time.time()

            if not self.paused:
                t0 = time.time()
                self.step_simulation()
                step_dt = time.time() - t0
                self._timing_acc["step"] += step_dt
                self._timing_acc["step_count"] += 1

            # Render and queue frame (encoding handled in background task)
            now = time.time()
            should_render = self.ws_clients or self._webrtc_new_frame_event is not None
            if should_render and now - self.last_frame_time >= self.frame_interval:
                frame_bgr = self.render_frame(encode=False)
                if frame_bgr is not None and self._frame_queue is not None:
                    try:
                        self._frame_queue.put_nowait(frame_bgr)
                    except asyncio.QueueFull:
                        self.frames_dropped_encode += 1
                        try:
                            _ = self._frame_queue.get_nowait()
                            self._frame_queue.task_done()
                        except Exception:
                            pass
                        try:
                            self._frame_queue.put_nowait(frame_bgr)
                        except asyncio.QueueFull:
                            pass
                self.last_frame_time = now

            # Stream metrics (WebSocket and Socket.io for WebRTC clients)
            if now - self.last_metrics_time >= self.metrics_interval:
                metrics = self.get_training_metrics()
                await self.broadcast_metrics(metrics)
                if self.sio and self.sio.connected:
                    await self.sio.emit("genesis_training_metrics", metrics)
                self.last_metrics_time = now

            # Report timing every 5s
            if now - _timing_last_report >= timing_interval and self._timing_acc["step_count"] > 0:
                avg_step = (self._timing_acc["step"] / max(self._timing_acc["step_count"], 1)) * 1000
                avg_render = (self._timing_acc["render"] / max(self._timing_acc["render_count"], 1)) * 1000
                avg_encode = (self._timing_acc["encode"] / max(self._timing_acc["encode_count"], 1)) * 1000
                avg_broadcast = (self._timing_acc["broadcast"] / max(self._timing_acc["broadcast_count"], 1)) * 1000
                logger.info(
                    f"[TIMING] step={avg_step:.1f}ms "
                    f"render={avg_render:.1f}ms "
                    f"encode={avg_encode:.1f}ms "
                    f"send={avg_broadcast:.1f}ms "
                    f"(calls: step={self._timing_acc['step_count']}, "
                    f"render={self._timing_acc['render_count']}, "
                    f"encode={self._timing_acc['encode_count']}, "
                    f"send={self._timing_acc['broadcast_count']})"
                )
                self._reset_timing_acc()
                _timing_last_report = now

            # Emit frame statistics periodically
            if now - self.frame_stats_time >= self.frame_stats_interval:
                await self._emit_frame_stats()

            # Maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = self.config.dt - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)
            else:
                await asyncio.sleep(0)  # Yield to other tasks
    
    async def _restart_frame_sender(self, maxsize: Optional[int] = None):
        """Recreate frame queue and restart sender task with a new maxsize."""
        size = max(1, int(maxsize if maxsize is not None else getattr(self.config, "frame_queue_size", 2)))
        self._frame_queue_size = size
        if self._frame_sender_task:
            self._frame_sender_task.cancel()
            try:
                await self._frame_sender_task
            except asyncio.CancelledError:
                pass
        self._frame_queue = asyncio.Queue(maxsize=size)
        if self.running:
            self._frame_sender_task = asyncio.create_task(self._frame_sender_loop())
    
    async def run(self):
        """Run the bridge server."""
        logger.info("=" * 60)
        logger.info("Starting Genesis Bridge Server...")
        logger.info("=" * 60)
        
        # Log import status first
        if gs is None:
            logger.warning(f"Genesis module status: NOT AVAILABLE")
            if _genesis_import_error:
                logger.warning(f"Import error: {_genesis_import_error}")
        else:
            logger.info(f"Genesis module status: AVAILABLE (v{getattr(gs, '__version__', 'unknown')})")
        
        # Initialize Genesis
        genesis_ready = await self.initialize_genesis()
        
        # Log final mode
        if genesis_ready and self.scene is not None:
            logger.info("=" * 60)
            logger.info("GENESIS BRIDGE RUNNING IN REAL MODE")
            logger.info(f"  Scene: {self.scene}")
            logger.info(f"  Camera: {self.camera}")
            logger.info("=" * 60)
        else:
            logger.warning("=" * 60)
            logger.warning("GENESIS BRIDGE RUNNING IN MOCK MODE")
            logger.warning("  Frames will show placeholder graphics")
            logger.warning("  No physics simulation will occur")
            if _genesis_import_error:
                logger.warning(f"  Reason: {_genesis_import_error_type}: {_genesis_import_error}")
            logger.warning("=" * 60)
        
        # Set up Socket.io
        self.setup_socketio()
        
        # WebRTC: create frame event and start signaling server when enabled
        if _aiortc and _aiohttp is not None and GenesisWebRTCVideoTrack is not None:
            self._webrtc_new_frame_event = asyncio.Event()
            webrtc_app = self._create_webrtc_app()
            if webrtc_app is not None:
                runner = web.AppRunner(webrtc_app)
                await runner.setup()
                site = web.TCPSite(runner, self.config.ws_host, self.config.webrtc_port)
                await site.start()
                logger.info(f"WebRTC signaling server listening on http://{self.config.ws_host}:{self.config.webrtc_port}")
        else:
            if _aiortc is False:
                logger.info("aiortc not installed - WebRTC streaming unavailable (install with: pip install aiortc aiohttp)")
        
        # Connect to SDR_OS (retry up to 10 times since Node.js may start slowly)
        if self.sio:
            for attempt in range(1, 11):
                try:
                    logger.info(f"Connecting to SDR_OS at {self.config.sdr_os_url} "
                                f"(attempt {attempt}/10) ...")
                    await self.sio.connect(self.config.sdr_os_url)
                    logger.info("Socket.io connected to SDR_OS")
                    break
                except Exception as e:
                    logger.error(f"Socket.io connect attempt {attempt} failed: {e}")
                    if attempt < 10:
                        await asyncio.sleep(2)
                    else:
                        logger.error("Giving up on Socket.io connection after 10 attempts")
        
        # Start WebSocket server
        self.running = True

        # Background worker for JPEG encode/broadcast
        await self._restart_frame_sender(self._frame_queue_size)

        # Start background thread for gamepad input (bypasses event loop starvation)
        self._start_gamepad_thread()
        self._start_command_update_thread()
        
        if websockets and ws_serve:
            # websockets 16.0+ handler signature: handler(websocket) only
            async def handler(websocket):
                await self.handle_websocket(websocket)
            
            async with ws_serve(handler, self.config.ws_host, self.config.ws_port):
                logger.info(f"WebSocket server listening on {self.config.ws_host}:{self.config.ws_port}")
                await self.main_loop()
        else:
            logger.warning("WebSocket server not available - running simulation only")
            await self.main_loop()
    
    def _start_command_update_thread(self):
        if not self._decouple_command_updates:
            return
        if self._command_update_thread is not None:
            return
        if self._command_update_hz <= 0:
            logger.warning("[CMD-UPDATE] disabled (hz <= 0)")
            return

        def _command_thread_fn():
            interval = 1.0 / max(self._command_update_hz, 1.0)
            while self.running:
                loop_start = time.time()
                self._update_gamepad_command()
                elapsed = time.time() - loop_start
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        t = threading.Thread(
            target=_command_thread_fn,
            daemon=True,
            name="gamepad-command-updater",
        )
        t.start()
        self._command_update_thread = t
        logger.info(f"[CMD-UPDATE] started ({self._command_update_hz:.0f} Hz)")

    def _start_gamepad_thread(self):
        """Start a background thread with a SYNC socketio client for gamepad input.

        The main asyncio event loop is blocked by step_simulation() (Genesis
        physics + policy inference), so the AsyncClient's event handlers never
        fire.  This thread uses python-socketio's *synchronous* Client which
        runs its own receive loop in a background thread — completely
        independent of the asyncio event loop.  Joystick data is written to
        self._pending_joystick (a dict-ref swap, atomic under CPython's GIL)
        and consumed by step_simulation() each tick.
        """
        if socketio is None:
            return

        def _gamepad_thread_fn(url: str):
            sio_sync = socketio.Client(logger=False, engineio_logger=False)

            @sio_sync.on('controller_joystick_state')
            def on_joystick(data):
                self._pending_joystick = data
                self._pending_joystick_time = time.time()

            @sio_sync.on('controller_button_states')
            def on_buttons(data):
                self._handle_button_states(data)

            @sio_sync.on('controller_mapping_type')
            def on_mapping_type(data):
                new_type = data.get('type', 'xbox')
                self.mapping_type = new_type
                logger.info(f"[GAMEPAD-THREAD] controller mapping type: {new_type}")

            @sio_sync.on('connect')
            def on_connect():
                logger.info("[GAMEPAD-THREAD] connected to SDR_OS")

            @sio_sync.on('disconnect')
            def on_disconnect():
                logger.info("[GAMEPAD-THREAD] disconnected from SDR_OS")

            while self.running:
                try:
                    logger.info(f"[GAMEPAD-THREAD] connecting to {url} ...")
                    sio_sync.connect(url, wait_timeout=10)
                    logger.info("[GAMEPAD-THREAD] connect() returned, calling wait()")
                    sio_sync.wait()  # blocks until disconnect
                except Exception as e:
                    logger.warning(f"[GAMEPAD-THREAD] connection error: {type(e).__name__}: {e}")
                finally:
                    if sio_sync.connected:
                        sio_sync.disconnect()
                if self.running:
                    time.sleep(2)  # retry delay

        t = threading.Thread(
            target=_gamepad_thread_fn,
            args=(self.config.sdr_os_url,),
            daemon=True,
            name="gamepad-input",
        )
        t.start()
        logger.info("[GAMEPAD-THREAD] started")

    def stop(self):
        """Stop the server."""
        self.running = False
        if self.episode_data:
            self.save_episode()


async def main():
    """Entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Genesis Bridge Server")
    parser.add_argument("--ws-port", type=int, default=9091, help="WebSocket port")
    parser.add_argument("--sdr-url", type=str, default="http://localhost:3000", help="SDR_OS URL")
    parser.add_argument("--config", type=str, default="configs/genesis_bridge.yaml", help="Bridge config YAML")
    parser.add_argument("--fps", type=int, default=None, help="Override target FPS")
    parser.add_argument("--quality", type=int, default=None, help="Override JPEG quality (1-100)")
    parser.add_argument("--camera-res", type=str, default=None, help="Override camera resolution WxH (e.g., 960x540)")
    parser.add_argument("--fast-stream", dest="fast_stream", action="store_true", default=None, help="Force fast-stream on")
    parser.add_argument("--no-fast-stream", dest="fast_stream", action="store_false", help="Force fast-stream off")
    parser.add_argument("--log", action="store_true", help="Enable data logging")
    parser.add_argument("--webrtc-port", type=int, default=9092, help="WebRTC signaling HTTP port")
    args = parser.parse_args()

    # Base config from file (if present)
    file_cfg = _load_bridge_config_file(args.config)

    # Parse camera resolution override if provided
    camera_res = None
    if args.camera_res:
        try:
            w, h = args.camera_res.lower().split("x")
            camera_res = (int(w), int(h))
        except Exception:
            logger.error("Invalid --camera-res format (expected WxH), ignoring")

    # Apply CLI overrides on top of file config
    cli_overrides = {
        "ws_port": args.ws_port,
        "sdr_os_url": args.sdr_url,
        "enable_logging": args.log,
        "webrtc_port": args.webrtc_port,
    }
    if args.fps is not None:
        cli_overrides["target_fps"] = args.fps
    if args.quality is not None:
        cli_overrides["jpeg_quality"] = args.quality
    if camera_res:
        cli_overrides["camera_res"] = camera_res
    if args.fast_stream is not None:
        cli_overrides["fast_stream"] = args.fast_stream

    merged_cfg = deepcopy(file_cfg)
    merged_cfg.update(cli_overrides)
    config = BridgeConfig(**_filter_bridge_kwargs(merged_cfg))
    
    server = GenesisBridgeServer(config)
    
    try:
        await server.run()
    except KeyboardInterrupt:
        logger.info("Shutting down...")
        server.stop()


if __name__ == "__main__":
    asyncio.run(main())
