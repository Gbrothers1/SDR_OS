"""Binary-patch gstaichi for AMDGPU DLPack zero-copy on AMD APU unified memory.

On AMD APUs (like Steam Deck's Van Gogh), the AMDGPU backend and HIP share
the same physical memory. gstaichi's DLPack implementation only supports
CPU/Metal/CUDA, but we can patch it at runtime to also support AMDGPU by
redirecting through AmdgpuDevice::get_alloc_info and using kDLROCM device type.

This enables true zero-copy between gstaichi fields and PyTorch HIP tensors:
no data copying, just pointer sharing.

Usage:
    import gstaichi as ti
    from sdr_os.amdgpu_dlpack_patch import patch_gstaichi_dlpack
    patch_gstaichi_dlpack()
    # Now field.to_dlpack() works with AMDGPU backend

Patches applied (4 total):
1. validate_arch(): NOP conditional jump → always returns OK
2. get_raw_ptr(): NOP error jump → falls through to device-specific path
3. get_raw_ptr(): Redirect CudaDevice::get_alloc_info → AmdgpuDevice::get_alloc_info
4. get_raw_ptr(): Change DLPack device_type from kDLCUDA(2) to kDLROCM(10)

Requirements:
- gstaichi 4.6.0 (llvm 20.1.0, commit 72f02b06)
- AMD APU with unified memory (tested on gfx1033 Van Gogh)
- PyTorch compiled with ROCm (HIP)
"""

import ctypes
import ctypes.util
import os
import struct
import logging

logger = logging.getLogger(__name__)

# Virtual addresses of symbols in gstaichi_python.cpython-312-x86_64-linux-gnu.so
# These are from gstaichi 4.6.0 (commit 72f02b06)
_PATCHES = [
    # (vaddr, expected_bytes, replacement_bytes, description)
    # PATCH 1: validate_arch - NOP the 'je' that skips to CUDA check
    (0x88e066, b'\x74\x18', b'\x90\x90', 'validate_arch: accept all archs'),
    # PATCH 2: get_raw_ptr - NOP the 'je' that jumps to error on non-CPU/CUDA
    (0x88e1df, b'\x74\x1d', b'\x90\x90', 'get_raw_ptr: no error on amdgpu'),
    # PATCH 3: get_raw_ptr - redirect call from CudaDevice to AmdgpuDevice
    # CudaDevice::get_alloc_info at 0x408fe10
    # AmdgpuDevice::get_alloc_info at 0x40b9cc0
    # call offset = target - (call_addr + 5)
    (0x88e1ed, b'\xe8\x1e\x1c\x80\x03',
     b'\xe8' + struct.pack('<i', 0x40b9cc0 - 0x88e1f2),
     'get_raw_ptr: CudaDevice -> AmdgpuDevice'),
    # PATCH 4: get_raw_ptr - change device_type from kDLCUDA(2) to kDLROCM(10)
    (0x88e1f7, b'\xba\x02\x00\x00\x00', b'\xba\x0a\x00\x00\x00',
     'get_raw_ptr: kDLCUDA -> kDLROCM'),
]

_patched = False


def patch_gstaichi_dlpack():
    """Apply binary patches to enable AMDGPU DLPack support.

    Must be called after importing gstaichi but before any DLPack conversions.
    Safe to call multiple times (patches are only applied once).

    Returns True if patches were applied successfully, False otherwise.
    """
    global _patched
    if _patched:
        return True

    # Find the loaded shared library's base address
    so_name = 'gstaichi_python.cpython-312-x86_64-linux-gnu.so'
    image_base = None
    with open('/proc/self/maps', 'r') as f:
        for line in f:
            if so_name in line:
                image_base = int(line.split('-')[0], 16)
                break

    if image_base is None:
        logger.error('gstaichi shared library not found in /proc/self/maps')
        return False

    libc = ctypes.CDLL(ctypes.util.find_library('c'))
    patched_pages = set()
    page_size = 4096

    for vaddr, expected, replacement, desc in _PATCHES:
        addr = image_base + vaddr

        # Verify expected bytes
        actual = bytes((ctypes.c_ubyte * len(expected)).from_address(addr))
        if actual != expected:
            logger.error(
                f'Patch "{desc}" failed: expected {expected.hex()} '
                f'at 0x{vaddr:x}, got {actual.hex()}. '
                f'gstaichi version mismatch?'
            )
            return False

        # Make page writable if needed
        page_start = addr & ~(page_size - 1)
        if page_start not in patched_pages:
            ret = libc.mprotect(ctypes.c_void_p(page_start), page_size * 2, 7)
            if ret != 0:
                logger.error(f'mprotect failed at 0x{page_start:x}')
                return False
            patched_pages.add(page_start)

        # Apply patch
        ctypes.memmove(addr, replacement, len(replacement))
        logger.info(f'Patched: {desc}')

    _patched = True
    logger.info('AMDGPU DLPack patches applied successfully')
    return True
