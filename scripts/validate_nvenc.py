#!/usr/bin/env python3
"""NVENC capability validation and GPU telemetry.

Runs inside the CUDA container to verify NVENC availability,
GPU info, codec support, and driver compatibility.

Exit codes:
    0 - NVENC available and functional
    1 - CUDA available but NVENC not functional
    2 - CUDA not available
"""
import json
import subprocess
import sys


def get_cuda_info() -> dict:
    """Detect CUDA availability and GPU properties via torch."""
    info = {"cuda_available": False}
    try:
        import torch
        info["cuda_available"] = torch.cuda.is_available()
        if not info["cuda_available"]:
            return info
        info["cuda_version"] = torch.version.cuda
        info["device_count"] = torch.cuda.device_count()
        info["devices"] = []
        for i in range(torch.cuda.device_count()):
            props = torch.cuda.get_device_properties(i)
            info["devices"].append({
                "index": i,
                "name": props.name,
                "total_memory_mb": props.total_mem // (1024 * 1024),
                "compute_capability": f"{props.major}.{props.minor}",
                "multi_processor_count": props.multi_processor_count,
            })
    except ImportError:
        info["error"] = "torch not installed"
    except Exception as e:
        info["error"] = str(e)
    return info


def get_nvidia_smi_info() -> dict:
    """Query nvidia-smi for driver info and NVENC session count."""
    info = {}
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=driver_version,gpu_name,encoder.stats.sessionCount,encoder.stats.averageFps",
             "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            parts = [p.strip() for p in result.stdout.strip().split(",")]
            info["driver_version"] = parts[0] if len(parts) > 0 else "unknown"
            info["gpu_name"] = parts[1] if len(parts) > 1 else "unknown"
            info["nvenc_active_sessions"] = parts[2] if len(parts) > 2 else "unknown"
            info["nvenc_average_fps"] = parts[3] if len(parts) > 3 else "unknown"
    except FileNotFoundError:
        info["error"] = "nvidia-smi not found"
    except Exception as e:
        info["error"] = str(e)
    return info


def get_nvenc_codec_support() -> dict:
    """Detect NVENC codec support via nvidia-smi and ffmpeg."""
    codecs = {"h264_nvenc": False, "hevc_nvenc": False}

    # Check via ffmpeg
    try:
        result = subprocess.run(
            ["ffmpeg", "-hide_banner", "-encoders"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            codecs["h264_nvenc"] = "h264_nvenc" in result.stdout
            codecs["hevc_nvenc"] = "hevc_nvenc" in result.stdout
            codecs["ffmpeg_available"] = True
        else:
            codecs["ffmpeg_available"] = False
    except FileNotFoundError:
        codecs["ffmpeg_available"] = False
    except Exception as e:
        codecs["ffmpeg_error"] = str(e)

    # Check via PyAV
    try:
        import av
        available = av.codecs_available
        codecs["pyav_h264_nvenc"] = "h264_nvenc" in available
        codecs["pyav_hevc_nvenc"] = "hevc_nvenc" in available
        codecs["pyav_available"] = True
    except ImportError:
        codecs["pyav_available"] = False
    except Exception as e:
        codecs["pyav_error"] = str(e)

    return codecs


def probe_nvenc_encode() -> dict:
    """Attempt a real NVENC encode to verify functionality."""
    result = {"functional": False}

    # Try PyAV first
    try:
        import av
        if "h264_nvenc" in av.codecs_available:
            encoder = av.CodecContext.create("h264_nvenc", "w")
            encoder.width = 256
            encoder.height = 256
            encoder.pix_fmt = "yuv420p"
            encoder.bit_rate = 1_000_000
            encoder.open()
            encoder.close()
            result["functional"] = True
            result["method"] = "pyav"
            return result
    except Exception as e:
        result["pyav_error"] = str(e)

    # Try ffmpeg subprocess
    try:
        proc = subprocess.run(
            ["ffmpeg", "-y",
             "-f", "lavfi", "-i", "testsrc=duration=0.1:size=256x256:rate=10",
             "-c:v", "h264_nvenc", "-f", "null", "-"],
            capture_output=True, timeout=15
        )
        if proc.returncode == 0:
            result["functional"] = True
            result["method"] = "ffmpeg"
            return result
        else:
            result["ffmpeg_error"] = proc.stderr.decode(errors="replace")[:300]
    except FileNotFoundError:
        result["ffmpeg_error"] = "ffmpeg not found"
    except Exception as e:
        result["ffmpeg_error"] = str(e)

    return result


def main():
    report = {
        "cuda": get_cuda_info(),
        "nvidia_smi": get_nvidia_smi_info(),
        "nvenc_codecs": get_nvenc_codec_support(),
        "nvenc_probe": probe_nvenc_encode(),
    }

    # Determine overall status
    if not report["cuda"].get("cuda_available"):
        report["status"] = "NO_CUDA"
        exit_code = 2
    elif report["nvenc_probe"].get("functional"):
        report["status"] = "NVENC_OK"
        exit_code = 0
    else:
        report["status"] = "NVENC_UNAVAILABLE"
        exit_code = 1

    print(json.dumps(report, indent=2))
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
