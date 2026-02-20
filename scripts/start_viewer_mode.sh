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

# Check if window was found (loop may have exhausted without finding one)
if ! DISPLAY="$DISPLAY_NUM" xdotool search --name "." >/dev/null 2>&1; then
    echo "[viewer-mode] WARNING: No viewer window detected after 30s, starting capture anyway"
fi

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
