# Viewer-Native Steam Deck Mode — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Eliminate the glReadPixels bottleneck by using the Genesis viewer on a virtual display, captured by ffmpeg and piped into the existing SHM/WebSocket stream.

**Architecture:** Genesis viewer renders to Xvfb virtual display at 60fps (no pixel readback). A Python capture sidecar runs ffmpeg x11grab to capture that display, parses JPEG frames from its stdout pipe, and writes them to ShmRingWriter. Transport-server and browser are unchanged.

**Tech Stack:** Python 3, ffmpeg (x11grab + mjpeg), Xvfb, ShmRingWriter, existing NATS/transport pipeline

---

### Task 1: Fix viewer max_FPS cap in Go2BridgeEnv

**Files:**
- Modify: `src/sdr_os/envs/go2_env.py:81`

**Step 1: Make the change**

In `src/sdr_os/envs/go2_env.py` line 81, replace:

```python
                max_FPS=int(0.5 / self.dt),
```

with:

```python
                max_FPS=60,
```

The old value evaluated to 25, artificially capping the viewer.

**Step 2: Verify no other references**

Run: `grep -rn 'max_FPS' src/sdr_os/`
Expected: Only the one line in `go2_env.py`

**Step 3: Commit**

```bash
git add src/sdr_os/envs/go2_env.py
git commit -m "fix: uncap Genesis viewer from 25fps to 60fps"
```

---

### Task 2: Add --viewer flag to genesis_sim_runner.py

**Files:**
- Modify: `scripts/genesis_sim_runner.py:439-445` (constructor)
- Modify: `scripts/genesis_sim_runner.py:489-504` (init_genesis)
- Modify: `scripts/genesis_sim_runner.py:987-1000` (run — init section)
- Modify: `scripts/genesis_sim_runner.py:1008-1021` (run — loop body)
- Modify: `scripts/genesis_sim_runner.py:1113-1145` (main — argparse + runner creation)

**Step 1: Add `headless` param to constructor**

In the `__init__` method (line 439), add `headless: bool = True` parameter and store it:

```python
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
```

**Step 2: Pass headless through to Go2BridgeEnv**

In `init_genesis()` (line 498-504), change `headless=True` to `headless=self.headless`:

```python
        self.env = Go2BridgeEnv(
            num_envs=1,
            dt=1 / 50,
            max_episode_length_s=None,
            headless=self.headless,
            camera_res=self.camera_res,
        )
```

**Step 3: Conditionally skip SHM/encoder in run()**

In the `run()` method (line 995-996), wrap SHM and encoder startup:

```python
        await self.connect_nats()
        if self.headless:
            self._start_encoder_thread()
```

**Step 4: Conditionally skip render_and_enqueue in the loop**

In the main loop body (line 1020-1021), wrap the render call:

```python
                # Render and submit to encoder thread (headless stream only)
                if self.headless:
                    self.render_and_enqueue()
```

**Step 5: Conditionally skip encoder stats in the loop**

In the encoder stats publish block (line 1056-1077), wrap it:

```python
                # Publish encoder stats every ~1s (from encoder thread)
                if self.headless and now - self._last_encode_stats_time > 1.0 and self.nc and self.nc.is_connected:
```

**Step 6: Use asyncio.sleep(0) when in viewer mode**

At the bottom of the loop (line 1142-1145), change the sleep to yield immediately in viewer mode since there's no frame pacing needed (the viewer handles its own timing):

```python
                if self.headless:
                    # Frame pacing — minimum 1ms yield so NATS keepalives are processed
                    elapsed = time.monotonic() - t0
                    sleep_time = frame_interval - elapsed
                    await asyncio.sleep(max(sleep_time, 0.001))
                else:
                    # Viewer mode — viewer thread handles display timing.
                    # Pace physics at sim dt rate, yield to NATS.
                    elapsed = time.monotonic() - t0
                    physics_dt = self.env.dt if self.env else 0.02
                    sleep_time = physics_dt - elapsed
                    await asyncio.sleep(max(sleep_time, 0))
```

**Step 7: Add --viewer CLI arg and wire it up**

In `main()` (line 1113-1145), add the arg and pass to runner:

After the `--gpu` argument (line 1120-1121), add:

```python
    parser.add_argument("--viewer", action="store_true",
                        help="Open Genesis viewer on DISPLAY (skip SHM encode pipeline)")
```

Change runner creation (line 1140-1145) to:

```python
    runner = GenesisSimRunner(
        target_fps=args.fps,
        camera_res=camera_res,
        jpeg_quality=args.jpeg_quality,
        checkpoint_dir=checkpoint,
        headless=not args.viewer,
    )
```

Wrap `init_shm()` (line 1156):

```python
    runner.init_genesis()
    if not args.viewer:
        runner.init_shm()
    await runner.run()
```

**Step 8: Smoke test**

Run: `DISPLAY=:2 python scripts/genesis_sim_runner.py --viewer --camera-res 640x360 --fps 30`
Expected: Viewer window opens on :2. Physics steps in logs. No SHM/encoder logs.
Verify: `ls -la /dev/shm/sdr_os_ipc/frames` should NOT be updated (no SHM writes).

**Step 9: Commit**

```bash
git add scripts/genesis_sim_runner.py
git commit -m "feat: add --viewer flag to skip SHM pipeline and use Genesis viewer"
```

---

### Task 3: Create viewer_capture.py sidecar

**Files:**
- Create: `scripts/viewer_capture.py`

**Step 1: Write the capture script**

```python
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
import struct
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
```

**Step 2: Make executable**

Run: `chmod +x scripts/viewer_capture.py`

**Step 3: Smoke test (requires Xvfb on :2 and ffmpeg)**

Run: `Xvfb :2 -screen 0 640x360x24 &` then `python scripts/viewer_capture.py --display :2 --fps 5`
Expected: ffmpeg starts, logs "Starting capture". With no window open on :2, frames will be black but should still appear in SHM. Ctrl-C stops cleanly.

**Step 4: Commit**

```bash
git add scripts/viewer_capture.py
git commit -m "feat: add viewer_capture.py sidecar for X11 grab to SHM pipeline"
```

---

### Task 4: Create start_viewer_mode.sh launcher

**Files:**
- Create: `scripts/start_viewer_mode.sh`

**Step 1: Write the launcher script**

```bash
#!/usr/bin/env bash
# start_viewer_mode.sh — Launch Genesis viewer + capture pipeline for Steam Deck
#
# Usage: ./scripts/start_viewer_mode.sh [--res 640x360] [--fps 30] [--quality 80]
#
# Starts:
#   1. Xvfb on DISPLAY=:2 (if not already running)
#   2. genesis_sim_runner.py --viewer on :2
#   3. viewer_capture.py capturing :2 into SHM
#
# Ctrl-C stops all processes.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Defaults
DISPLAY_NUM=":2"
RES="640x360"
FPS="30"
QUALITY="80"
CHECKPOINT=""
EXTRA_SIM_ARGS=""

# Parse args
while [[ $# -gt 0 ]]; do
    case "$1" in
        --res)       RES="$2"; shift 2 ;;
        --fps)       FPS="$2"; shift 2 ;;
        --quality)   QUALITY="$2"; shift 2 ;;
        --display)   DISPLAY_NUM="$2"; shift 2 ;;
        --checkpoint) CHECKPOINT="--checkpoint $2"; shift 2 ;;
        *)           EXTRA_SIM_ARGS="$EXTRA_SIM_ARGS $1"; shift ;;
    esac
done

WIDTH="${RES%%x*}"
HEIGHT="${RES##*x}"

# Track child PIDs for cleanup
PIDS=()

cleanup() {
    echo ""
    echo "[viewer-mode] Stopping all processes..."
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill "$pid" 2>/dev/null || true
        fi
    done
    # Wait briefly then force-kill stragglers
    sleep 1
    for pid in "${PIDS[@]}"; do
        if kill -0 "$pid" 2>/dev/null; then
            kill -9 "$pid" 2>/dev/null || true
        fi
    done
    echo "[viewer-mode] Done."
}
trap cleanup EXIT INT TERM

# 1. Start Xvfb if not already running on target display
if ! xdpyinfo -display "$DISPLAY_NUM" >/dev/null 2>&1; then
    echo "[viewer-mode] Starting Xvfb on $DISPLAY_NUM (${WIDTH}x${HEIGHT}x24)..."
    Xvfb "$DISPLAY_NUM" -screen 0 "${WIDTH}x${HEIGHT}x24" &
    PIDS+=($!)
    sleep 1
    if ! xdpyinfo -display "$DISPLAY_NUM" >/dev/null 2>&1; then
        echo "[viewer-mode] ERROR: Xvfb failed to start on $DISPLAY_NUM"
        exit 1
    fi
    echo "[viewer-mode] Xvfb running on $DISPLAY_NUM"
else
    echo "[viewer-mode] Display $DISPLAY_NUM already active"
fi

# 2. Launch genesis_sim_runner with --viewer
echo "[viewer-mode] Starting sim runner (viewer mode, ${RES})..."
DISPLAY="$DISPLAY_NUM" python "$SCRIPT_DIR/genesis_sim_runner.py" \
    --viewer \
    --camera-res "$RES" \
    --fps "$FPS" \
    $CHECKPOINT \
    $EXTRA_SIM_ARGS &
SIM_PID=$!
PIDS+=($SIM_PID)

# Wait for viewer window to appear (up to 30s)
echo "[viewer-mode] Waiting for viewer window on $DISPLAY_NUM..."
for i in $(seq 1 30); do
    if DISPLAY="$DISPLAY_NUM" xdotool search --name "." >/dev/null 2>&1; then
        echo "[viewer-mode] Viewer window detected"
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[viewer-mode] ERROR: Sim runner exited before viewer appeared"
        exit 1
    fi
    sleep 1
done

# 3. Launch capture sidecar
echo "[viewer-mode] Starting capture (${RES} @ ${FPS}fps, quality=${QUALITY})..."
python "$SCRIPT_DIR/viewer_capture.py" \
    --display "$DISPLAY_NUM" \
    --res "$RES" \
    --fps "$FPS" \
    --quality "$QUALITY" &
PIDS+=($!)

echo "[viewer-mode] All processes running. Press Ctrl-C to stop."
echo "[viewer-mode]   Sim runner PID: $SIM_PID"
echo "[viewer-mode]   Capture PID:    ${PIDS[-1]}"

# Wait for sim runner to exit (it's the primary process)
wait "$SIM_PID"
```

**Step 2: Make executable**

Run: `chmod +x scripts/start_viewer_mode.sh`

**Step 3: Commit**

```bash
git add scripts/start_viewer_mode.sh
git commit -m "feat: add start_viewer_mode.sh launcher for Steam Deck viewer pipeline"
```

---

### Task 5: Integration test

**Step 1: Start the full pipeline**

```bash
./scripts/start_viewer_mode.sh --res 640x360 --fps 30
```

Expected output:
```
[viewer-mode] Starting Xvfb on :2 (640x360x24)...
[viewer-mode] Xvfb running on :2
[viewer-mode] Starting sim runner (viewer mode, 640x360)...
[viewer-mode] Waiting for viewer window on :2...
[viewer-mode] Viewer window detected
[viewer-mode] Starting capture (640x360 @ 30fps, quality=80)...
[viewer-mode] All processes running. Press Ctrl-C to stop.
```

**Step 2: Verify transport-server picks up frames**

Start transport-server (if not running):
```bash
SDR_NATS_URL=nats://localhost:4222 ./services/transport-server/target/release/transport-server
```

Expected: Transport-server logs show frames arriving from SHM.

**Step 3: Verify browser video**

Open `http://localhost:3000` in browser. SimViewer panel should show the Genesis sim feed.

**Step 4: Verify gamepad commands via NATS**

Press gamepad buttons. Sim runner logs should show `set_cmd_vel` commands arriving.

**Step 5: Ctrl-C cleanup test**

Press Ctrl-C on start_viewer_mode.sh. All three processes (Xvfb, sim runner, capture) should stop cleanly.

**Step 6: Final commit (if any fixups needed)**

```bash
git add -A
git commit -m "fix: integration test fixups for viewer mode"
```
