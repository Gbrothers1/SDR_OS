# Render Pipeline GPU Offload Design

**Date:** 2026-02-04
**Status:** Approved
**Branch:** `render-pipeline-offload`

## Overview

Offload CPU-bound image encoding and ML inference to GPU without reducing quality. The current pipeline burns CPU on JPEG encoding, color conversion, and RL inference that could ride the GPU instead.

## Goals

- Eliminate CPU JPEG encoding in the Genesis live view hot path
- Keep Genesis render tensors on GPU through encoding
- Reduce WebRTC color conversion overhead (eliminate double RGB↔BGR)
- Auto-detect CUDA for RL training and inference
- Maintain identical visual quality (same JPEG Q or high-bitrate NVENC)

## Non-Goals

- Frontend changes for H.264 WebSocket playback (future PR)
- Hardware decoding on client side
- Changing the WebSocket/WebRTC protocol for JPEG path

---

## Architecture

### GPU Frame Pipeline

New `GpuFramePipeline` class handles capability detection and GPU-accelerated encoding.

```
┌─────────────────┐
│ Genesis Render  │  ← returns GPU tensor (RGB, HWC)
└────────┬────────┘
         │ (stays on GPU)
         ▼
┌─────────────────┐
│ GpuFramePipeline│
│  .encode_jpeg   │  ← NVJPEG via torchvision.io.encode_jpeg
│  .encode_h264   │  ← NVENC via PyAV (opt-in)
└────────┬────────┘
         │ (CPU bytes out)
         ▼
   WebSocket / WebRTC
```

### Capability Detection (at startup)

```python
class GpuFramePipeline:
    def __init__(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.has_cuda = torch.cuda.is_available()
        self.has_nvjpeg = self._detect_nvjpeg()
        self.has_nvenc = self._detect_nvenc()

    def _detect_nvjpeg(self) -> bool:
        if not self.has_cuda:
            return False
        try:
            import torchvision.io
            # Test encode a small tensor
            test = torch.zeros(3, 8, 8, dtype=torch.uint8, device='cuda')
            torchvision.io.encode_jpeg(test, quality=80)
            return True
        except Exception:
            return False

    def _detect_nvenc(self) -> bool:
        try:
            import av
            return 'h264_nvenc' in av.codecs_available
        except ImportError:
            return False
```

### Fallback Behavior

If GPU capabilities are unavailable, fall back to current CPU path transparently:

| Capability | GPU Path | CPU Fallback |
|------------|----------|--------------|
| JPEG encode | torchvision.io.encode_jpeg (NVJPEG) | cv2.imencode |
| Color convert | Not needed (NVJPEG takes RGB) | cv2.cvtColor |
| H.264 encode | PyAV h264_nvenc | Not available (JPEG only) |

---

## Component Changes

### 1. WebSocket JPEG Path (NVJPEG)

**Files:** `genesis_bridge/bridge_server.py` lines 1268-1288, 1968-1974

**Current:**
```python
bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)  # CPU
_, jpeg_buf = cv2.imencode('.jpg', bgr_array, [cv2.IMWRITE_JPEG_QUALITY, q])  # CPU
```

**New:**
```python
def encode_jpeg(self, rgb_tensor: torch.Tensor, quality: int) -> bytes:
    if self.has_nvjpeg:
        # torchvision expects CHW uint8 tensor on CUDA
        chw = rgb_tensor.permute(2, 0, 1).contiguous()
        jpeg_bytes = torchvision.io.encode_jpeg(chw, quality=quality)
        return bytes(jpeg_bytes.cpu().numpy())
    else:
        # CPU fallback
        rgb_np = rgb_tensor.cpu().numpy() if rgb_tensor.is_cuda else rgb_tensor.numpy()
        bgr_np = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        _, buf = cv2.imencode('.jpg', bgr_np, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return buf.tobytes()
```

**Note:** NVJPEG encodes RGB directly (JFIF standard). No BGR conversion needed on GPU path.

### 2. WebRTC GPU Buffer Sharing

**Files:** `genesis_bridge/bridge_server.py` lines 1230-1232, 1271-1274

**Current:** Double color conversion + numpy copy
```python
self._webrtc_latest_frame = np.ascontiguousarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
```

**New:** Store GPU tensor, single CPU copy at consumption time
```python
# In render path:
if self._webrtc_new_frame_event is not None:
    self._webrtc_latest_tensor = rgb_tensor  # Keep on GPU
    self._webrtc_new_frame_event.set()

# In VideoStreamTrack.recv():
frame_np = self._webrtc_latest_tensor.cpu().numpy()
frame_np = np.ascontiguousarray(frame_np)
```

**Benefit:** Eliminates RGB→BGR→RGB round-trip and one numpy copy.

**Note:** aiortc does not have native NVENC support (issue #323 closed without resolution). Direct GPU→NVENC would require monkey-patching aiortc's encoder. Out of scope for v1.

### 3. NVENC H.264 Opt-in for WebSocket

**Files:** `genesis_bridge/gpu_pipeline.py` (new), config

**Config addition:**
```python
ws_video_codec: str = "jpeg"  # "jpeg" (default) or "h264"
h264_bitrate: int = 8_000_000  # 8 Mbps default
```

**Implementation:**
```python
def encode_h264(self, rgb_tensor: torch.Tensor) -> bytes:
    if not self.has_nvenc:
        raise RuntimeError("NVENC not available")

    frame_np = rgb_tensor.cpu().numpy()
    av_frame = av.VideoFrame.from_ndarray(frame_np, format='rgb24')

    if self._h264_encoder is None:
        self._h264_encoder = av.CodecContext.create('h264_nvenc', 'w')
        self._h264_encoder.width = rgb_tensor.shape[1]
        self._h264_encoder.height = rgb_tensor.shape[0]
        self._h264_encoder.pix_fmt = 'yuv420p'
        self._h264_encoder.bit_rate = self.config.h264_bitrate
        self._h264_encoder.open()

    packets = self._h264_encoder.encode(av_frame)
    return b''.join(bytes(p) for p in packets)
```

**Frontend requirement:** H.264 mode requires `<video>` + MSE/WebCodecs on frontend. Backend-only in v1; frontend playback is a separate PR.

### 4. ROS Camera Relay

**Files:** `genesis_bridge/gs_ros_bridge.py` lines 399-405

**Behavior:** Use GPU pipeline when Genesis is active (GPU already warm), CPU otherwise.

```python
async def stream_frame(self, cv_image: np.ndarray):
    if self.gpu_pipeline:  # Genesis mode active
        tensor = torch.from_numpy(cv_image).to(self.gpu_pipeline.device)
        jpeg_bytes = self.gpu_pipeline.encode_jpeg(tensor, self.config.jpeg_quality)
    else:
        # CPU fallback (standalone ROS bridge)
        _, jpeg_buf = cv2.imencode('.jpg', cv_image,
                                   [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality])
        jpeg_bytes = jpeg_buf.tobytes()

    await self.broadcast_frame(jpeg_bytes)
```

No config flag — piggybacks on existing GPU context.

### 5. RL Training Device Auto-Detection

**Files:** `rl/scripts/train_bc.py`, `rl/scripts/train_rsl_rl.py`, new `rl/utils/device.py`

**New utility:**
```python
# rl/utils/device.py
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

**Training scripts change:**
```python
parser.add_argument("--device", type=str, default="auto",
                    choices=["auto", "cpu", "cuda"], help="Device (auto-detects CUDA)")
# ...
device = get_device(args.device)
```

### 6. ROS Bridge Policy Inference Fix

**Files:** `genesis_bridge/gs_ros_bridge.py` lines 499-507, 344-350

**Current issues:**
- `torch.load()` without device mapping
- Numpy observation passed directly to policy (expects tensor)

**Fixed policy loading:**
```python
def load_policy(self, checkpoint_path: str):
    from rl.utils.device import get_device

    self.policy_device = get_device("auto")
    checkpoint = torch.load(checkpoint_path, map_location=self.policy_device)

    if isinstance(checkpoint, dict) and 'model_state_dict' in checkpoint:
        from rl.policies.bc_policy import BCPolicy
        self.policy = BCPolicy(checkpoint['obs_dim'], checkpoint['action_dim'])
        self.policy.load_state_dict(checkpoint['model_state_dict'])
    else:
        self.policy = checkpoint

    self.policy = self.policy.to(self.policy_device)
    self.policy.eval()
```

**Fixed inference:**
```python
if self.policy is not None and self.current_obs is not None:
    obs_tensor = torch.from_numpy(self.current_obs).unsqueeze(0).to(self.policy_device)
    with torch.no_grad():
        policy_action = self.policy(obs_tensor)
    policy_action = policy_action.squeeze(0).cpu().numpy()
```

---

## File Structure

**New files:**
```
genesis_bridge/
  gpu_pipeline.py      # GpuFramePipeline class

rl/utils/
  __init__.py
  device.py            # get_device() helper
```

**Modified files:**
```
genesis_bridge/bridge_server.py   # Use GpuFramePipeline
genesis_bridge/gs_ros_bridge.py   # Use GpuFramePipeline, fix policy inference
rl/scripts/train_bc.py            # --device default "auto"
rl/scripts/train_rsl_rl.py        # --device default "auto"
requirements.txt                  # Add av (PyAV)
```

---

## Dependencies

| Package | Purpose | Install |
|---------|---------|---------|
| torch (CUDA) | GPU tensors, device management | Already present |
| torchvision (CUDA) | NVJPEG via encode_jpeg | Already present |
| av (PyAV) | NVENC H.264 encoding | `pip install av` |

**PyAV NVENC check:**
```bash
python -c "import av; print('h264_nvenc' in av.codecs_available)"
```

Recent PyAV wheels (5.0+) include FFmpeg with NVENC on Linux x86_64. For other platforms or older versions, build from source with NVENC-enabled FFmpeg.

---

## Testing Strategy

1. **Unit tests for GpuFramePipeline:**
   - Test NVJPEG encode produces valid JPEG
   - Test CPU fallback when CUDA unavailable
   - Test NVENC encode produces valid H.264 NALUs

2. **Integration tests:**
   - Verify WebSocket JPEG stream works with GPU pipeline
   - Verify WebRTC receives correct RGB frames
   - Verify ROS relay uses GPU when Genesis active

3. **Device tests:**
   - `get_device("auto")` returns CUDA when available
   - Policy loads correctly with `map_location`
   - Inference works with tensor observations

4. **Performance benchmarks:**
   - Compare CPU vs GPU encode latency at 720p, 1080p
   - Measure frame rate improvement at high FPS targets

---

## Rollout

1. **Phase 1:** GPU JPEG pipeline (NVJPEG) — default path
2. **Phase 2:** RL device auto-detection — training scripts + inference
3. **Phase 3:** NVENC H.264 opt-in — backend only
4. **Phase 4 (future PR):** Frontend H.264 playback via MSE/WebCodecs
