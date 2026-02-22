# AMDGPU Zero-Copy

Technical documentation for running Genesis physics simulation with the AMDGPU Taichi backend and DLPack zero-copy on AMD APUs (tested on Steam Deck).

## Overview

On AMD APUs like the Steam Deck (Van Gogh, gfx1033 RDNA2), the GPU and CPU share unified physical memory. SDR_OS exploits this by:

1. Running Genesis physics kernels on the **AMDGPU Taichi backend** (instead of Vulkan)
2. Using **HIP** (AMD's CUDA equivalent) for PyTorch tensor operations
3. Enabling **DLPack zero-copy** between Taichi GPU fields and PyTorch HIP tensors — no data copying, just pointer sharing

This gives a significant speedup over the Vulkan backend:

| Scenario | Vulkan | AMDGPU + Zero-Copy | Speedup |
|----------|--------|-------------------|---------|
| Go2 robot | 14.1ms/step | 11.5ms/step (87 FPS) | 18% |
| Plane scene | 14.1ms/step | 2.8ms/step (432 FPS) | 5x |

## How It Works

### Backend Routing

Genesis only has built-in support for `gs.gpu` (auto-detect), `gs.cuda`, and `gs.vulkan` backends. The zero-copy code path requires `backend == gs_backend.cuda` and `device.type == "cuda"`. To make this work on AMD:

```python
# In genesis_sim_runner.py → _init_genesis_amdgpu()
import gstaichi as _ti
from genesis.constants import backend as gs_backend, TI_ARCH

# Remap: when Genesis asks for CUDA, Taichi compiles for AMDGPU
TI_ARCH['Linux'][gs_backend.cuda] = _ti.amdgpu

# Now gs.init(backend=gs.cuda) uses ti.amdgpu kernels
# but gs.device = torch.device("cuda", 0) → HIP
gs.init(backend=gs.cuda, performance_mode=True)
```

### DLPack Binary Patches

gstaichi 4.6.0's DLPack implementation (`field.to_dlpack()`) only whitelists CPU, Metal, and CUDA devices. To support AMDGPU, `src/sdr_os/amdgpu_dlpack_patch.py` applies 4 binary patches at runtime:

| # | Address | Original | Patched | Effect |
|---|---------|----------|---------|--------|
| 1 | `0x88e066` | `je +0x18` | `nop nop` | `validate_arch()`: accept all backends |
| 2 | `0x88e1df` | `je +0x1d` | `nop nop` | `get_raw_ptr()`: don't error on AMDGPU |
| 3 | `0x88e1ed` | `call CudaDevice::get_alloc_info` | `call AmdgpuDevice::get_alloc_info` | Redirect to correct device allocator |
| 4 | `0x88e1f7` | `mov edx, 2` (kDLCUDA) | `mov edx, 10` (kDLROCM) | Set correct DLPack device type for PyTorch |

The patches are applied by:

1. Finding the `.so` base address in `/proc/self/maps`
2. `mprotect()` to make the code pages writable
3. `memmove()` to overwrite the bytes
4. Verifying expected bytes before patching (version check)

This works because on APU unified memory, HSA (AMDGPU runtime) and HIP share the same address space. A pointer from a Taichi AMDGPU field is directly usable as a HIP/ROCm tensor pointer.

### Sync Fences

Even though memory is shared, GPU compute operations from different APIs (Taichi AMDGPU kernels vs HIP/PyTorch) must not overlap. The sim uses dual fences at domain boundaries:

```python
import gstaichi as _ti
_ti.sync()                    # Flush pending Taichi AMDGPU kernels
torch.cuda.synchronize()      # Flush pending HIP operations
```

These appear in:

- `render_and_enqueue()` — before pyrender reads camera pose from GPU fields
- `step_sim()` — before transferring observations to the inference device
- `env.reset()` — after resetting Taichi fields, before HIP tensor operations

## Required Environment Variables

| Variable | Value | Required | Purpose |
|----------|-------|----------|---------|
| `HSA_OVERRIDE_GFX_VERSION` | `10.3.0` | Steam Deck | Maps gfx1033 ISA to supported gfx1030 |
| `HSA_ENABLE_SDMA` | `0` | Steam Deck | Disables DMA engine — prevents `hipMemcpy` deadlock when HIP + Vulkan/AMDGPU coexist on APU ([ROCm/HIP #3874](https://github.com/ROCm/HIP/issues/3874)) |
| `HIP_LAUNCH_BLOCKING` | `1` | All AMDGPU | Forces synchronous HIP dispatch — fixes gstaichi async dispatch race where SNode init isn't flushed before user kernels (nil pointer crash) |
| `GS_ENABLE_ZEROCOPY` | `1` | Auto | Set by `_init_genesis_amdgpu()` after DLPack patches succeed |
| `TORCHDYNAMO_DISABLE` | `1` | Recommended | Disables torch.compile dynamo tracing (not compatible with gstaichi) |

The sim runner (`genesis_sim_runner.py`) sets all of these automatically when it detects an AMD GPU.

## ROCm `ld.lld` Requirement

gstaichi 4.6.0 uses LLVM 20, which produces ELF v3 relocatable objects. The system `ld.lld` (typically v14 on SteamOS/Arch) cannot link them:

```
ld.lld: error: unrecognized relocation (42) in section .text
```

ROCm 6.3 ships `ld.lld` v18 at `/opt/rocm-6.3.0/llvm/bin/ld.lld`. The sim runner creates a symlink on startup:

```python
os.makedirs("/tmp/lld_only", exist_ok=True)
os.symlink("/opt/rocm-6.3.0/llvm/bin/ld.lld", "/tmp/lld_only/ld.lld")
os.environ["PATH"] = f"/tmp/lld_only:{os.environ['PATH']}"
```

## File Reference

| File | Purpose |
|------|---------|
| `scripts/genesis_sim_runner.py` | Main sim runner with AMDGPU auto-detection and init |
| `src/sdr_os/amdgpu_dlpack_patch.py` | Binary patches for gstaichi DLPack AMDGPU support |
| `scripts/run_sim_viewer.sh` | Viewer launcher with DISPLAY/XAUTHORITY auto-detection |
| `scripts/test_hip_vulkan.py` | Diagnostic script for testing HIP+Vulkan coexistence |
| `process-compose.yml` | Process orchestration with `sim` and `sim-viewer` entries |

## Limitations

- **gstaichi version-locked**: Binary patches target gstaichi 4.6.0 (commit 72f02b06). A version update requires re-finding patch addresses.
- **Genesis 0.4.0 incompatible**: quadrants 0.4.0 has Vulkan SPIRV codegen regressions on gfx1033. Stick with genesis 0.3.14 + gstaichi 4.6.0.
- **First-run compilation**: AMDGPU kernel compilation takes ~220s for Go2 on first run. Cached at `~/.cache/comgr/` and `~/.cache/gstaichi/` for subsequent runs.
- **No NVENC**: AMD GPUs don't have NVENC. Video encoding uses JPEG (turbojpeg) or software H.264 (libx264).
- **Single GPU**: On APU, there's only one GPU shared between physics (AMDGPU Taichi) and tensors (HIP). Sync fences prevent concurrent use but add ~1ms overhead per boundary crossing.

## Debugging

**Verify AMDGPU backend is active:**

```bash
# Look for these in sim logs:
# "AMD GPU detected — routing gs.cuda → ti.amdgpu"
# "DLPack patches applied — enabling zero-copy"
# "Genesis AMDGPU init: device=cuda:0, backend=..., zerocopy=True"
```

**Test HIP independently:**

```bash
export HSA_OVERRIDE_GFX_VERSION=10.3.0
uv run python -c "
import torch
print(f'Device: {torch.cuda.get_device_name(0)}')
x = torch.randn(100, 100, device='cuda')
print(f'Tensor on GPU: {x.device}, sum={x.sum().item():.2f}')
"
```

**Run the diagnostic script:**

```bash
uv run scripts/test_hip_vulkan.py
```

This tests HIP+Vulkan coexistence, zero-copy, and basic physics stepping.
