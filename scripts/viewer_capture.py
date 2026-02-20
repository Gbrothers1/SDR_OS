#!/usr/bin/env python3
"""
Viewer Capture Sidecar — X11 grab → JPEG → SHM

Captures the Genesis viewer window from a virtual X display using ffmpeg
x11grab, parses JPEG frames from the pipe, and writes them to the SHM
ringbuffer for the transport-server to relay via WebSocket.

Usage:
    python scripts/viewer_capture.py --display :2 --res 640x360 --fps 30
"""

import sys
import os
import signal
import logging
import argparse
import subprocess
import time
from pathlib import Path

_project_root = str(Path(__file__).resolve().parent.parent)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from src.sdr_os.ipc.shm_ringbuffer import ShmRingWriter, FrameFlags, Codec

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("viewer_capture")

SHM_PATH = os.environ.get("SDR_SHM_PATH", "/dev/shm/sdr_os_ipc/frames")
SHM_SIZE = int(os.environ.get("SDR_SHM_SIZE", 4 * 1024 * 1024))

# JPEG markers
SOI = b"\xff\xd8"
EOI = b"\xff\xd9"


def run_capture(display: str, width: int, height: int, fps: int, quality: int):
    """Launch ffmpeg x11grab and pipe JPEG frames to SHM."""

    crc_enabled = os.environ.get("SDR_CRC_ENABLED", "1") != "0"
    shm = ShmRingWriter(path=SHM_PATH, buffer_size=SHM_SIZE, crc_enabled=crc_enabled)
    logger.info(f"SHM writer ready: {SHM_PATH} ({SHM_SIZE} bytes, CRC={'on' if crc_enabled else 'off'})")

    # mjpeg quality: ffmpeg -q:v ranges 2 (best) to 31 (worst)
    # Map our 1-100 quality to ffmpeg's inverted scale
    ffmpeg_q = max(2, min(31, int(31 - (quality / 100) * 29)))

    cmd = [
        "ffmpeg",
        "-f", "x11grab",
        "-video_size", f"{width}x{height}",
        "-framerate", str(fps),
        "-i", display,
        "-c:v", "mjpeg",
        "-q:v", str(ffmpeg_q),
        "-f", "image2pipe",
        "-an",
        "pipe:1",
    ]

    logger.info(f"Starting capture: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=0,
    )

    frame_id = 0
    buf = b""
    frames_written = 0
    last_log_time = time.monotonic()

    def shutdown(sig, frame):
        logger.info(f"Received signal {sig}, stopping...")
        proc.terminate()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    try:
        while True:
            chunk = proc.stdout.read(65536)
            if not chunk:
                break

            buf += chunk

            # Extract complete JPEG frames (SOI...EOI)
            while True:
                soi_pos = buf.find(SOI)
                if soi_pos == -1:
                    buf = b""
                    break

                eoi_pos = buf.find(EOI, soi_pos + 2)
                if eoi_pos == -1:
                    # Incomplete frame — keep from SOI onward
                    buf = buf[soi_pos:]
                    break

                # Complete JPEG frame: SOI to EOI inclusive
                jpeg_data = buf[soi_pos:eoi_pos + 2]
                buf = buf[eoi_pos + 2:]

                # Write to SHM
                ok = shm.write(
                    payload=jpeg_data,
                    frame_id=frame_id,
                    flags=FrameFlags.KEYFRAME,  # Every JPEG is a keyframe
                    codec=Codec.JPEG,
                )

                if ok:
                    frames_written += 1
                    frame_id += 1
                else:
                    logger.warning(f"SHM write failed (frame too large: {len(jpeg_data)} bytes)")

                # Log stats every ~5s
                now = time.monotonic()
                if now - last_log_time > 5.0:
                    elapsed = now - last_log_time
                    logger.info(
                        f"capture: {frames_written} frames, "
                        f"~{frames_written / elapsed:.1f} fps, "
                        f"last frame {len(jpeg_data)} bytes"
                    )
                    frames_written = 0
                    last_log_time = now

    except Exception as e:
        logger.error(f"Capture error: {e}")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        shm.close()
        logger.info("Capture stopped")


def main():
    parser = argparse.ArgumentParser(description="Viewer Capture Sidecar (X11 grab → SHM)")
    parser.add_argument("--display", type=str, default=":2",
                        help="X11 display to capture (default: :2)")
    parser.add_argument("--res", type=str, default="640x360",
                        help="Capture resolution WxH (default: 640x360)")
    parser.add_argument("--fps", type=int, default=30,
                        help="Capture framerate (default: 30)")
    parser.add_argument("--quality", type=int, default=80,
                        help="JPEG quality 1-100 (default: 80)")
    args = parser.parse_args()

    w, h = args.res.lower().split("x")
    run_capture(
        display=args.display,
        width=int(w),
        height=int(h),
        fps=args.fps,
        quality=args.quality,
    )


if __name__ == "__main__":
    main()
