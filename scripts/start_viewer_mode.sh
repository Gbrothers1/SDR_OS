#!/usr/bin/env bash
# start_viewer_mode.sh — Launch Genesis viewer + capture pipeline for Steam Deck
#
# Usage: ./scripts/start_viewer_mode.sh [--res 640x360] [--fps 30] [--quality 80]
#
# Starts Xvfb, sim runner with --viewer, and capture sidecar.
# Ctrl-C stops all processes.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Defaults
VDISPLAY=":99"
RES="640x360"
FPS="30"
QUALITY="80"
SIM_ARGS=""

# Parse args — everything not recognized is forwarded to the sim runner
while [[ $# -gt 0 ]]; do
    case "$1" in
        --res)       RES="$2"; shift 2 ;;
        --fps)       FPS="$2"; shift 2 ;;
        --quality)   QUALITY="$2"; shift 2 ;;
        --display)   VDISPLAY="$2"; shift 2 ;;
        *)           SIM_ARGS="$SIM_ARGS $1"; shift ;;
    esac
done

WIDTH="${RES%%x*}"
HEIGHT="${RES##*x}"

PIDS=()
cleanup() {
    echo ""
    echo "[viewer-mode] Stopping all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
        kill -9 "$pid" 2>/dev/null || true
    done
    echo "[viewer-mode] Done."
}
trap cleanup EXIT INT TERM

# 1. Start Xvfb
if ! xdpyinfo -display "$VDISPLAY" >/dev/null 2>&1; then
    echo "[viewer-mode] Starting Xvfb on $VDISPLAY (${WIDTH}x${HEIGHT})..."
    Xvfb "$VDISPLAY" -screen 0 "${WIDTH}x${HEIGHT}x24" &
    PIDS+=($!)
    sleep 1
fi

# 2. Start sim runner with --viewer on the virtual display
# LIBGL_ALWAYS_SOFTWARE=1 forces Mesa llvmpipe so GL renders into the
# Xvfb framebuffer (hardware GL bypasses it, causing black capture).
echo "[viewer-mode] Starting sim runner..."
DISPLAY="$VDISPLAY" LIBGL_ALWAYS_SOFTWARE=1 \
    uv run "$SCRIPT_DIR/genesis_sim_runner.py" \
    --viewer \
    --camera-res "$RES" \
    --fps "$FPS" \
    $SIM_ARGS &
SIM_PID=$!
PIDS+=($SIM_PID)

# 3. Wait for sim to initialize (viewer window must exist before capture)
echo "[viewer-mode] Waiting for viewer to initialize..."
for i in $(seq 1 90); do
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[viewer-mode] ERROR: Sim runner exited during init"
        exit 1
    fi
    # Check if any window exists on the virtual display
    if xwininfo -display "$VDISPLAY" -root -tree 2>/dev/null | grep -q '0x.*:'; then
        echo "[viewer-mode] Viewer window detected"
        break
    fi
    sleep 1
done

# 4. Start capture sidecar (captures full Xvfb display — no window detection needed)
echo "[viewer-mode] Starting capture (${RES} @ ${FPS}fps, quality=${QUALITY})..."
uv run "$SCRIPT_DIR/viewer_capture.py" \
    --display "$VDISPLAY" \
    --res "$RES" \
    --fps "$FPS" \
    --quality "$QUALITY" &
PIDS+=($!)

echo "[viewer-mode] Running. Ctrl-C to stop."
wait "$SIM_PID"
