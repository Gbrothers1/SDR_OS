#!/usr/bin/env bash
# start_viewer_mode.sh — Launch Genesis viewer + capture pipeline for Steam Deck
#
# Usage: ./scripts/start_viewer_mode.sh [--res 640x360] [--fps 30] [--quality 80]
#
# Runs Genesis viewer on the real display (hardware Vulkan), detects the
# window ID with xwininfo, and captures it via ffmpeg x11grab → SHM.
# Ctrl-C stops all processes.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Defaults
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
        *)           SIM_ARGS="$SIM_ARGS $1"; shift ;;
    esac
done

WIDTH="${RES%%x*}"
HEIGHT="${RES##*x}"

# Use the current session display
if [ -z "${DISPLAY:-}" ]; then
    echo "[viewer-mode] ERROR: No DISPLAY set. Run from a graphical session."
    exit 1
fi
echo "[viewer-mode] Using display $DISPLAY (hardware Vulkan)"

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

# 1. Start sim runner with --viewer on the real display (Vulkan GPU rendering)
echo "[viewer-mode] Starting sim runner..."
uv run "$SCRIPT_DIR/genesis_sim_runner.py" \
    --viewer \
    --camera-res "$RES" \
    --fps "$FPS" \
    $SIM_ARGS &
SIM_PID=$!
PIDS+=($SIM_PID)

# 2. Wait for viewer window to appear, then grab its window ID
echo "[viewer-mode] Waiting for viewer window..."
WINDOW_ID=""
for i in $(seq 1 120); do
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[viewer-mode] ERROR: Sim runner exited during init"
        exit 1
    fi
    # Look for the genesis_sim_runner window (Genesis viewer sets the script path as title)
    WINDOW_ID=$(xwininfo -root -tree 2>/dev/null \
        | grep -i "genesis_sim_runner\|genesis\|sim_runner" \
        | head -1 \
        | awk '{print $1}' || true)
    if [ -n "$WINDOW_ID" ]; then
        echo "[viewer-mode] Viewer window detected: $WINDOW_ID"
        break
    fi
    sleep 1
done

if [ -z "$WINDOW_ID" ]; then
    echo "[viewer-mode] WARNING: Could not detect viewer window after 120s, capturing full display"
fi

# Small delay to let the window fully initialize and render first frame
sleep 2

# 3. Start capture sidecar
CAPTURE_ARGS=(
    --display "$DISPLAY"
    --res "$RES"
    --fps "$FPS"
    --quality "$QUALITY"
)
if [ -n "$WINDOW_ID" ]; then
    CAPTURE_ARGS+=(--window-id "$WINDOW_ID")
fi

echo "[viewer-mode] Starting capture (${RES} @ ${FPS}fps, quality=${QUALITY}, window=${WINDOW_ID:-full})..."
uv run "$SCRIPT_DIR/viewer_capture.py" "${CAPTURE_ARGS[@]}" &
PIDS+=($!)

echo "[viewer-mode] Running. Ctrl-C to stop."
wait "$SIM_PID"
