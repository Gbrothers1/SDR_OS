# Render Pipeline GPU Offload Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Offload CPU-bound JPEG encoding and ML inference to GPU using NVJPEG and CUDA device auto-detection.

**Architecture:** New `GpuFramePipeline` class handles capability detection and GPU encoding. Training scripts and inference use `get_device("auto")` for CUDA auto-detection. WebRTC shares GPU tensors directly.

**Tech Stack:** PyTorch CUDA, torchvision.io.encode_jpeg (NVJPEG), PyAV (NVENC H.264 opt-in)

---

## Task 1: Create rl/utils/device.py

**Files:**
- Create: `rl/utils/__init__.py`
- Create: `rl/utils/device.py`
- Create: `rl/utils/test_device.py`

**Step 1: Create rl/utils directory and __init__.py**

```bash
mkdir -p rl/utils
```

```python
# rl/utils/__init__.py
from .device import get_device

__all__ = ['get_device']
```

**Step 2: Write the failing test**

```python
# rl/utils/test_device.py
import torch
import pytest
from rl.utils.device import get_device


def test_get_device_auto_returns_torch_device():
    """get_device('auto') returns a torch.device."""
    device = get_device("auto")
    assert isinstance(device, torch.device)


def test_get_device_cpu_returns_cpu():
    """get_device('cpu') always returns CPU device."""
    device = get_device("cpu")
    assert device.type == "cpu"


def test_get_device_cuda_returns_cuda_or_raises():
    """get_device('cuda') returns CUDA if available."""
    if torch.cuda.is_available():
        device = get_device("cuda")
        assert device.type == "cuda"
    else:
        # On CPU-only machines, requesting cuda should still return cuda device
        # (torch will error later when actually used)
        device = get_device("cuda")
        assert device.type == "cuda"


def test_get_device_auto_prefers_cuda():
    """get_device('auto') returns CUDA when available, else CPU."""
    device = get_device("auto")
    if torch.cuda.is_available():
        assert device.type == "cuda"
    else:
        assert device.type == "cpu"
```

**Step 3: Run test to verify it fails**

Run: `python -m pytest rl/utils/test_device.py -v`
Expected: FAIL with "ModuleNotFoundError: No module named 'rl.utils.device'"

**Step 4: Write minimal implementation**

```python
# rl/utils/device.py
"""Device utilities for RL training and inference."""
import torch


def get_device(requested: str = "auto") -> torch.device:
    """Resolve device string to torch.device.

    Args:
        requested: "auto", "cuda", or "cpu"

    Returns:
        torch.device - cuda:0 if available and requested != "cpu", else cpu
    """
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(requested)
```

**Step 5: Run test to verify it passes**

Run: `python -m pytest rl/utils/test_device.py -v`
Expected: PASS (4 tests)

**Step 6: Commit**

```bash
git add rl/utils/
git commit -m "feat(rl): add get_device() utility for CUDA auto-detection"
```

---

## Task 2: Update train_bc.py to use auto device

**Files:**
- Modify: `rl/scripts/train_bc.py:81` (--device default)
- Modify: `rl/scripts/train_bc.py:122` (use get_device)

**Step 1: Write the test**

```python
# rl/scripts/test_train_bc_device.py
import subprocess
import sys


def test_train_bc_help_shows_auto_default():
    """train_bc.py --help shows device default is 'auto'."""
    result = subprocess.run(
        [sys.executable, "-m", "rl.scripts.train_bc", "--help"],
        capture_output=True,
        text=True
    )
    assert "default='auto'" in result.stdout or "default: auto" in result.stdout.lower()
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest rl/scripts/test_train_bc_device.py -v`
Expected: FAIL (default is 'cpu')

**Step 3: Modify train_bc.py**

At line 81, change:
```python
# OLD
parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"], help="Device")

# NEW
parser.add_argument("--device", type=str, default="auto", choices=["auto", "cpu", "cuda"], help="Device (auto-detects CUDA)")
```

At line 122, add import and use get_device:
```python
# Add at top of file (after other imports)
from rl.utils.device import get_device

# At line 122, change:
# OLD
policy = BCPolicy(args.obs_dim, args.action_dim).to(args.device)

# NEW
device = get_device(args.device)
policy = BCPolicy(args.obs_dim, args.action_dim).to(device)
```

Also update line 94 to print resolved device:
```python
# OLD
print(f"Device: {args.device}")

# NEW
device = get_device(args.device)
print(f"Device: {device} (requested: {args.device})")
```

And update lines 146-147 to use device variable:
```python
# These should already work since device is now a torch.device
batch_obs = batch_obs.to(device)
batch_actions = batch_actions.to(device)
```

**Step 4: Run test to verify it passes**

Run: `python -m pytest rl/scripts/test_train_bc_device.py -v`
Expected: PASS

**Step 5: Commit**

```bash
git add rl/scripts/train_bc.py rl/scripts/test_train_bc_device.py
git commit -m "feat(rl): train_bc.py defaults to auto device detection"
```

---

## Task 3: Update train_rsl_rl.py to use auto device

**Files:**
- Modify: `rl/scripts/train_rsl_rl.py:222` (--device default)
- Modify: `rl/scripts/train_rsl_rl.py:257` (use get_device)

**Step 1: Write the test**

```python
# rl/scripts/test_train_rsl_rl_device.py
import subprocess
import sys


def test_train_rsl_rl_help_shows_auto_default():
    """train_rsl_rl.py --help shows device default is 'auto'."""
    result = subprocess.run(
        [sys.executable, "-m", "rl.scripts.train_rsl_rl", "--help"],
        capture_output=True,
        text=True
    )
    assert "default='auto'" in result.stdout or "default: auto" in result.stdout.lower()
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest rl/scripts/test_train_rsl_rl_device.py -v`
Expected: FAIL (default is 'cpu')

**Step 3: Modify train_rsl_rl.py**

At line 222, change:
```python
# OLD
parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"], help="Device")

# NEW
parser.add_argument("--device", type=str, default="auto", choices=["auto", "cpu", "cuda"], help="Device (auto-detects CUDA)")
```

Add import at top:
```python
from rl.utils.device import get_device
```

After parsing args (around line 235), resolve device:
```python
# After args = parser.parse_args()
device = get_device(args.device)
```

Update print statement:
```python
# OLD
print(f"Device: {args.device}")

# NEW
print(f"Device: {device} (requested: {args.device})")
```

Update make_policy call at line 257:
```python
# OLD
policy = make_policy(env.num_obs, env.num_actions, args.device)

# NEW
policy = make_policy(env.num_obs, env.num_actions, device)
```

Update checkpoint loading at line 265:
```python
# OLD
checkpoint = torch.load(args.checkpoint)

# NEW
checkpoint = torch.load(args.checkpoint, map_location=device)
```

**Step 4: Run test to verify it passes**

Run: `python -m pytest rl/scripts/test_train_rsl_rl_device.py -v`
Expected: PASS

**Step 5: Commit**

```bash
git add rl/scripts/train_rsl_rl.py rl/scripts/test_train_rsl_rl_device.py
git commit -m "feat(rl): train_rsl_rl.py defaults to auto device detection"
```

---

## Task 4: Create GpuFramePipeline class

**Files:**
- Create: `genesis_bridge/gpu_pipeline.py`
- Create: `genesis_bridge/test_gpu_pipeline.py`

**Step 1: Write the failing test**

```python
# genesis_bridge/test_gpu_pipeline.py
import pytest
import numpy as np

# Test without requiring CUDA
class TestGpuFramePipelineCapabilities:
    """Test capability detection (works on any machine)."""

    def test_pipeline_initializes(self):
        """GpuFramePipeline initializes without error."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        assert pipeline is not None

    def test_has_capability_flags(self):
        """Pipeline has has_cuda, has_nvjpeg, has_nvenc flags."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        assert hasattr(pipeline, 'has_cuda')
        assert hasattr(pipeline, 'has_nvjpeg')
        assert hasattr(pipeline, 'has_nvenc')
        assert isinstance(pipeline.has_cuda, bool)
        assert isinstance(pipeline.has_nvjpeg, bool)
        assert isinstance(pipeline.has_nvenc, bool)

    def test_has_device(self):
        """Pipeline has device attribute."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        import torch
        assert isinstance(pipeline.device, torch.device)


class TestGpuFramePipelineEncode:
    """Test encoding (uses CPU fallback if no CUDA)."""

    def test_encode_jpeg_from_numpy(self):
        """encode_jpeg works with HWC uint8 numpy array."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()

        # Create test image (RGB, HWC format)
        rgb_np = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        jpeg_bytes = pipeline.encode_jpeg_numpy(rgb_np, quality=80)

        assert isinstance(jpeg_bytes, bytes)
        assert len(jpeg_bytes) > 0
        # JPEG magic bytes
        assert jpeg_bytes[:2] == b'\xff\xd8'

    def test_encode_jpeg_from_tensor(self):
        """encode_jpeg works with HWC torch tensor."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        import torch
        pipeline = GpuFramePipeline()

        # Create test image (RGB, HWC format) on pipeline's device
        rgb_tensor = torch.randint(0, 255, (480, 640, 3), dtype=torch.uint8, device=pipeline.device)

        jpeg_bytes = pipeline.encode_jpeg(rgb_tensor, quality=80)

        assert isinstance(jpeg_bytes, bytes)
        assert len(jpeg_bytes) > 0
        assert jpeg_bytes[:2] == b'\xff\xd8'

    def test_encode_jpeg_quality_affects_size(self):
        """Higher quality produces larger JPEG."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()

        rgb_np = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        low_q = pipeline.encode_jpeg_numpy(rgb_np, quality=20)
        high_q = pipeline.encode_jpeg_numpy(rgb_np, quality=95)

        assert len(high_q) > len(low_q)
```

**Step 2: Run test to verify it fails**

Run: `python -m pytest genesis_bridge/test_gpu_pipeline.py -v`
Expected: FAIL with "ModuleNotFoundError: No module named 'genesis_bridge.gpu_pipeline'"

**Step 3: Write implementation**

```python
# genesis_bridge/gpu_pipeline.py
"""GPU-accelerated frame encoding pipeline.

Provides NVJPEG (via torchvision) and optional NVENC H.264 encoding
with automatic CPU fallback when GPU capabilities are unavailable.
"""
import logging
from typing import Optional

import cv2
import numpy as np
import torch

logger = logging.getLogger(__name__)


class GpuFramePipeline:
    """GPU-accelerated frame encoding with automatic CPU fallback.

    Capabilities are detected at initialization:
    - has_cuda: CUDA available
    - has_nvjpeg: torchvision JPEG encoding works on CUDA
    - has_nvenc: PyAV h264_nvenc codec available

    Usage:
        pipeline = GpuFramePipeline()
        jpeg_bytes = pipeline.encode_jpeg(rgb_tensor, quality=80)
    """

    def __init__(self):
        self.has_cuda = torch.cuda.is_available()
        self.device = torch.device("cuda" if self.has_cuda else "cpu")
        self.has_nvjpeg = self._detect_nvjpeg()
        self.has_nvenc = self._detect_nvenc()

        # H.264 encoder state (lazy init)
        self._h264_encoder = None
        self._h264_width = 0
        self._h264_height = 0

        logger.info(
            f"GpuFramePipeline initialized: device={self.device}, "
            f"nvjpeg={self.has_nvjpeg}, nvenc={self.has_nvenc}"
        )

    def _detect_nvjpeg(self) -> bool:
        """Detect if NVJPEG encoding is available via torchvision."""
        if not self.has_cuda:
            return False
        try:
            import torchvision.io
            # Test encode a small tensor
            test = torch.zeros(3, 8, 8, dtype=torch.uint8, device='cuda')
            torchvision.io.encode_jpeg(test, quality=80)
            return True
        except Exception as e:
            logger.debug(f"NVJPEG not available: {e}")
            return False

    def _detect_nvenc(self) -> bool:
        """Detect if NVENC H.264 encoding is available via PyAV."""
        try:
            import av
            return 'h264_nvenc' in av.codecs_available
        except ImportError:
            return False
        except Exception as e:
            logger.debug(f"NVENC detection failed: {e}")
            return False

    def encode_jpeg(self, rgb_tensor: torch.Tensor, quality: int = 80) -> bytes:
        """Encode RGB tensor to JPEG bytes.

        Args:
            rgb_tensor: HWC uint8 tensor (RGB format) on any device
            quality: JPEG quality 1-100

        Returns:
            JPEG-encoded bytes
        """
        if self.has_nvjpeg and rgb_tensor.is_cuda:
            return self._encode_jpeg_nvjpeg(rgb_tensor, quality)
        else:
            return self._encode_jpeg_cpu(rgb_tensor, quality)

    def encode_jpeg_numpy(self, rgb_np: np.ndarray, quality: int = 80) -> bytes:
        """Encode RGB numpy array to JPEG bytes.

        Args:
            rgb_np: HWC uint8 numpy array (RGB format)
            quality: JPEG quality 1-100

        Returns:
            JPEG-encoded bytes
        """
        if self.has_nvjpeg:
            # Upload to GPU and encode
            rgb_tensor = torch.from_numpy(rgb_np).to(self.device)
            return self._encode_jpeg_nvjpeg(rgb_tensor, quality)
        else:
            # Direct CPU encode (no upload/download overhead)
            return self._encode_jpeg_cpu_numpy(rgb_np, quality)

    def _encode_jpeg_nvjpeg(self, rgb_tensor: torch.Tensor, quality: int) -> bytes:
        """Encode using NVJPEG via torchvision."""
        import torchvision.io
        # torchvision expects CHW format
        chw = rgb_tensor.permute(2, 0, 1).contiguous()
        jpeg_tensor = torchvision.io.encode_jpeg(chw, quality=quality)
        return bytes(jpeg_tensor.cpu().numpy())

    def _encode_jpeg_cpu(self, rgb_tensor: torch.Tensor, quality: int) -> bytes:
        """Encode using CPU OpenCV (fallback)."""
        rgb_np = rgb_tensor.cpu().numpy() if rgb_tensor.is_cuda else rgb_tensor.numpy()
        return self._encode_jpeg_cpu_numpy(rgb_np, quality)

    def _encode_jpeg_cpu_numpy(self, rgb_np: np.ndarray, quality: int) -> bytes:
        """Encode numpy array using CPU OpenCV."""
        # OpenCV expects BGR
        bgr_np = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        _, buf = cv2.imencode('.jpg', bgr_np, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()

    def encode_h264(self, rgb_tensor: torch.Tensor, bitrate: int = 8_000_000) -> bytes:
        """Encode RGB tensor to H.264 bytes using NVENC.

        Args:
            rgb_tensor: HWC uint8 tensor (RGB format)
            bitrate: Target bitrate in bits/sec

        Returns:
            H.264-encoded bytes (may be empty if no keyframe yet)

        Raises:
            RuntimeError: If NVENC is not available
        """
        if not self.has_nvenc:
            raise RuntimeError("NVENC not available. Install PyAV with NVENC support.")

        import av

        frame_np = rgb_tensor.cpu().numpy()
        height, width = frame_np.shape[:2]

        # Reinitialize encoder if resolution changed
        if self._h264_encoder is None or width != self._h264_width or height != self._h264_height:
            if self._h264_encoder is not None:
                self._h264_encoder.close()

            self._h264_encoder = av.CodecContext.create('h264_nvenc', 'w')
            self._h264_encoder.width = width
            self._h264_encoder.height = height
            self._h264_encoder.pix_fmt = 'yuv420p'
            self._h264_encoder.bit_rate = bitrate
            self._h264_encoder.options = {'preset': 'llhp'}  # Low-latency high performance
            self._h264_encoder.open()

            self._h264_width = width
            self._h264_height = height
            logger.info(f"H.264 encoder initialized: {width}x{height} @ {bitrate/1e6:.1f} Mbps")

        av_frame = av.VideoFrame.from_ndarray(frame_np, format='rgb24')
        packets = self._h264_encoder.encode(av_frame)
        return b''.join(bytes(p) for p in packets)

    def close(self):
        """Clean up encoder resources."""
        if self._h264_encoder is not None:
            try:
                self._h264_encoder.close()
            except Exception:
                pass
            self._h264_encoder = None
```

**Step 4: Run test to verify it passes**

Run: `python -m pytest genesis_bridge/test_gpu_pipeline.py -v`
Expected: PASS (6 tests)

**Step 5: Commit**

```bash
git add genesis_bridge/gpu_pipeline.py genesis_bridge/test_gpu_pipeline.py
git commit -m "feat(bridge): add GpuFramePipeline for GPU-accelerated encoding"
```

---

## Task 5: Add PyAV to requirements.txt

**Files:**
- Modify: `requirements.txt`

**Step 1: Edit requirements.txt**

Add after the aiortc line:
```
# GPU video encoding (NVENC H.264)
av>=10.0.0
```

**Step 2: Commit**

```bash
git add requirements.txt
git commit -m "deps: add PyAV for NVENC H.264 encoding"
```

---

## Task 6: Integrate GpuFramePipeline into bridge_server.py

**Files:**
- Modify: `genesis_bridge/bridge_server.py` (multiple locations)

**Step 1: Add import and initialization**

Near other imports (around line 38), add:
```python
from genesis_bridge.gpu_pipeline import GpuFramePipeline
```

In `__init__` method, after other state initialization (around line 500), add:
```python
# GPU encoding pipeline
self.gpu_pipeline: Optional[GpuFramePipeline] = None
```

In the initialization section where Genesis is set up (around line 665, after `gs.init()`), add:
```python
# Initialize GPU pipeline after Genesis (CUDA context ready)
try:
    self.gpu_pipeline = GpuFramePipeline()
    logger.info(f"GPU pipeline: nvjpeg={self.gpu_pipeline.has_nvjpeg}, nvenc={self.gpu_pipeline.has_nvenc}")
except Exception as e:
    logger.warning(f"GPU pipeline init failed, using CPU encoding: {e}")
    self.gpu_pipeline = None
```

**Step 2: Update render_frame to use GPU pipeline**

At lines 1268-1288, replace the encoding section:

```python
# OLD (lines 1268-1288):
# OpenCV expects BGR
bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)

# Feed WebRTC track when available
if self._webrtc_new_frame_event is not None:
    self._webrtc_latest_frame = np.ascontiguousarray(rgb_array)
    self._webrtc_new_frame_event.set()
if not encode:
    return bgr_array

# Encode as JPEG
logger.debug(f"render_frame: Rendering frame from camera, shape: {bgr_array.shape}")
encode_start = time.time()
_, jpeg_buf = cv2.imencode(
    '.jpg', bgr_array,
    [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
)
self._timing_acc["encode"] += time.time() - encode_start
self._timing_acc["encode_count"] += 1

return jpeg_buf.tobytes()

# NEW:
# Keep as tensor if possible for GPU encoding
if isinstance(rgb_array, torch.Tensor):
    rgb_tensor = rgb_array
else:
    rgb_tensor = torch.from_numpy(rgb_array)

# Feed WebRTC track when available (keep on GPU if possible)
if self._webrtc_new_frame_event is not None:
    if self.gpu_pipeline and rgb_tensor.is_cuda:
        self._webrtc_latest_tensor = rgb_tensor  # Stay on GPU
        self._webrtc_latest_frame = None
    else:
        self._webrtc_latest_frame = np.ascontiguousarray(
            rgb_tensor.cpu().numpy() if rgb_tensor.is_cuda else rgb_tensor.numpy()
        )
        self._webrtc_latest_tensor = None
    self._webrtc_new_frame_event.set()

if not encode:
    # Return numpy BGR for backwards compatibility
    rgb_np = rgb_tensor.cpu().numpy() if rgb_tensor.is_cuda else rgb_tensor.numpy()
    return cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)

# Encode as JPEG (GPU or CPU)
encode_start = time.time()
if self.gpu_pipeline:
    jpeg_bytes = self.gpu_pipeline.encode_jpeg(rgb_tensor, self.config.jpeg_quality)
else:
    rgb_np = rgb_tensor.cpu().numpy() if rgb_tensor.is_cuda else rgb_tensor.numpy()
    bgr_np = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
    _, jpeg_buf = cv2.imencode('.jpg', bgr_np, [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality])
    jpeg_bytes = jpeg_buf.tobytes()

self._timing_acc["encode"] += time.time() - encode_start
self._timing_acc["encode_count"] += 1

return jpeg_bytes
```

**Step 3: Update _frame_sender_loop to use GPU pipeline**

At lines 1968-1974, update the encoding:

```python
# OLD:
encode_start = time.time()
_, jpeg_buf = cv2.imencode(
    '.jpg', frame_bgr,
    [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
)
self._timing_acc["encode"] += time.time() - encode_start
self._timing_acc["encode_count"] += 1
await self.broadcast_frame(jpeg_buf.tobytes())

# NEW:
encode_start = time.time()
if self.gpu_pipeline:
    # frame_bgr is BGR numpy, convert to RGB for GPU pipeline
    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    jpeg_bytes = self.gpu_pipeline.encode_jpeg_numpy(frame_rgb, self.config.jpeg_quality)
else:
    _, jpeg_buf = cv2.imencode(
        '.jpg', frame_bgr,
        [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
    )
    jpeg_bytes = jpeg_buf.tobytes()

self._timing_acc["encode"] += time.time() - encode_start
self._timing_acc["encode_count"] += 1
await self.broadcast_frame(jpeg_bytes)
```

**Step 4: Update WebRTC VideoStreamTrack to use GPU tensor**

Find the WebRTC track class (search for `_webrtc_latest_frame`) and update its recv() method to handle tensors:

```python
# In recv() method, add handling for GPU tensor:
if self._bridge._webrtc_latest_tensor is not None:
    frame_np = self._bridge._webrtc_latest_tensor.cpu().numpy()
    frame_np = np.ascontiguousarray(frame_np)
elif self._bridge._webrtc_latest_frame is not None:
    frame_np = self._bridge._webrtc_latest_frame
else:
    # No frame available
    ...
```

**Step 5: Add _webrtc_latest_tensor state variable**

In `__init__`, near `_webrtc_latest_frame` initialization:
```python
self._webrtc_latest_tensor: Optional[torch.Tensor] = None
```

**Step 6: Test manually**

Run: `python genesis_bridge/bridge_server.py --help`
Expected: No import errors

**Step 7: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat(bridge): integrate GpuFramePipeline for GPU JPEG encoding"
```

---

## Task 7: Fix gs_ros_bridge.py policy loading and inference

**Files:**
- Modify: `genesis_bridge/gs_ros_bridge.py:499-507` (load_policy)
- Modify: `genesis_bridge/gs_ros_bridge.py:344-350` (inference)

**Step 1: Update load_policy method**

At lines 499-507, replace:

```python
# OLD:
def load_policy(self, checkpoint_path: str):
    """Load RL policy from checkpoint."""
    try:
        import torch
        self.policy = torch.load(checkpoint_path)
        self.policy.eval()
        logger.info(f"Policy loaded from {checkpoint_path}")
    except Exception as e:
        logger.error(f"Failed to load policy: {e}")

# NEW:
def load_policy(self, checkpoint_path: str):
    """Load RL policy from checkpoint."""
    try:
        import torch
        from rl.utils.device import get_device

        self.policy_device = get_device("auto")
        checkpoint = torch.load(checkpoint_path, map_location=self.policy_device)

        # Handle both full model saves and state_dict saves
        if isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
            from rl.policies.bc_policy import BCPolicy
            obs_dim = checkpoint.get('obs_dim', 48)
            action_dim = checkpoint.get('action_dim', 12)
            self.policy = BCPolicy(obs_dim, action_dim)
            self.policy.load_state_dict(checkpoint['model_state_dict'])
        else:
            self.policy = checkpoint

        self.policy = self.policy.to(self.policy_device)
        self.policy.eval()
        logger.info(f"Policy loaded from {checkpoint_path} on {self.policy_device}")
    except Exception as e:
        logger.error(f"Failed to load policy: {e}")
        self.policy = None
        self.policy_device = None
```

**Step 2: Add policy_device initialization**

In `__init__`, add:
```python
self.policy_device = None
```

**Step 3: Update inference code**

At lines 344-350, replace:

```python
# OLD:
if self.config.enable_rl and self.policy is not None and self.current_obs is not None:
    try:
        # Inference
        policy_action = self.policy(self.current_obs)
        confidence = 1.0  # Could extract from policy
    except Exception as e:
        logger.error(f"Policy inference error: {e}")

# NEW:
if self.config.enable_rl and self.policy is not None and self.current_obs is not None:
    try:
        import torch
        # Convert numpy observation to tensor on correct device
        obs_tensor = torch.from_numpy(self.current_obs).unsqueeze(0).to(self.policy_device)
        with torch.no_grad():
            policy_action_tensor = self.policy(obs_tensor)
        policy_action = policy_action_tensor.squeeze(0).cpu().numpy()
        confidence = 1.0  # Could extract from policy
    except Exception as e:
        logger.error(f"Policy inference error: {e}")
```

**Step 4: Commit**

```bash
git add genesis_bridge/gs_ros_bridge.py
git commit -m "fix(bridge): policy loading with device mapping and tensor inference"
```

---

## Task 8: Integrate GpuFramePipeline into gs_ros_bridge.py

**Files:**
- Modify: `genesis_bridge/gs_ros_bridge.py:391-409` (stream_frame)

**Step 1: Add gpu_pipeline attribute**

In `__init__`, add:
```python
self.gpu_pipeline: Optional['GpuFramePipeline'] = None
```

**Step 2: Add method to set GPU pipeline**

```python
def set_gpu_pipeline(self, pipeline: 'GpuFramePipeline'):
    """Set GPU pipeline for accelerated encoding (called when Genesis is active)."""
    self.gpu_pipeline = pipeline
    logger.info(f"GPU pipeline set: nvjpeg={pipeline.has_nvjpeg}")
```

**Step 3: Update stream_frame method**

At lines 391-409, replace:

```python
# OLD:
async def stream_frame(self, cv_image: np.ndarray):
    """Encode frame as JPEG and stream to WebSocket clients."""
    now = time.time()
    if now - self.last_frame_time < self.frame_interval:
        return

    self.last_frame_time = now

    # Encode as JPEG
    _, jpeg_buf = cv2.imencode(
        '.jpg',
        cv_image,
        [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
    )

    jpeg_bytes = jpeg_buf.tobytes()

    # Broadcast to all WebSocket clients
    await self.broadcast_frame(jpeg_bytes)

# NEW:
async def stream_frame(self, cv_image: np.ndarray):
    """Encode frame as JPEG and stream to WebSocket clients."""
    now = time.time()
    if now - self.last_frame_time < self.frame_interval:
        return

    self.last_frame_time = now

    # Encode as JPEG (GPU if pipeline available, else CPU)
    if self.gpu_pipeline:
        # GPU pipeline expects RGB, cv_image is BGR from ROS
        import torch
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        jpeg_bytes = self.gpu_pipeline.encode_jpeg_numpy(rgb_image, self.config.jpeg_quality)
    else:
        _, jpeg_buf = cv2.imencode(
            '.jpg',
            cv_image,
            [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
        )
        jpeg_bytes = jpeg_buf.tobytes()

    # Broadcast to all WebSocket clients
    await self.broadcast_frame(jpeg_bytes)
```

**Step 4: Commit**

```bash
git add genesis_bridge/gs_ros_bridge.py
git commit -m "feat(bridge): use GPU encoding in ROS bridge when Genesis active"
```

---

## Task 9: Final integration test

**Files:**
- Create: `genesis_bridge/test_integration_gpu.py`

**Step 1: Write integration test**

```python
# genesis_bridge/test_integration_gpu.py
"""Integration tests for GPU pipeline."""
import pytest
import numpy as np


class TestGpuPipelineIntegration:
    """Test GPU pipeline integration with bridge components."""

    def test_gpu_pipeline_encodes_simulated_frame(self):
        """Simulate a frame through the GPU pipeline."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline

        pipeline = GpuFramePipeline()

        # Simulate 720p RGB frame
        frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)

        jpeg = pipeline.encode_jpeg_numpy(frame, quality=80)

        assert len(jpeg) > 1000  # Reasonable JPEG size
        assert jpeg[:2] == b'\xff\xd8'  # JPEG magic

    def test_device_utility_works(self):
        """get_device returns valid torch.device."""
        from rl.utils.device import get_device
        import torch

        device = get_device("auto")
        assert isinstance(device, torch.device)

        device_cpu = get_device("cpu")
        assert device_cpu.type == "cpu"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

**Step 2: Run all tests**

Run: `python -m pytest genesis_bridge/test_gpu_pipeline.py genesis_bridge/test_integration_gpu.py rl/utils/test_device.py -v`
Expected: All tests PASS

**Step 3: Commit**

```bash
git add genesis_bridge/test_integration_gpu.py
git commit -m "test: add GPU pipeline integration tests"
```

---

## Summary

| Task | Description | Files |
|------|-------------|-------|
| 1 | Create device utility | `rl/utils/device.py` |
| 2 | Update train_bc.py | `rl/scripts/train_bc.py` |
| 3 | Update train_rsl_rl.py | `rl/scripts/train_rsl_rl.py` |
| 4 | Create GpuFramePipeline | `genesis_bridge/gpu_pipeline.py` |
| 5 | Add PyAV dependency | `requirements.txt` |
| 6 | Integrate into bridge_server | `genesis_bridge/bridge_server.py` |
| 7 | Fix policy loading/inference | `genesis_bridge/gs_ros_bridge.py` |
| 8 | GPU encoding in ROS bridge | `genesis_bridge/gs_ros_bridge.py` |
| 9 | Integration tests | `genesis_bridge/test_integration_gpu.py` |

Total: 9 tasks, ~9 commits
