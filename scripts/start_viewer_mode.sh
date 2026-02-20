#!/usr/bin/env bash
# start_viewer_mode.sh — Launch Genesis viewer + capture pipeline for Steam Deck
#
# Usage: ./scripts/start_viewer_mode.sh [--res 640x360] [--fps 30] [--quality 80]
#
# Starts:
#   1. genesis_sim_runner.py --viewer on the real display (hardware GL)
#   2. viewer_capture.py capturing the viewer window into SHM
#
# The viewer window is moved offscreen so only the browser is visible.
# Ctrl-C stops all processes.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Defaults
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
        --display)   export DISPLAY="$2"; shift 2 ;;
        --checkpoint) CHECKPOINT="--checkpoint $2"; shift 2 ;;
        *)           EXTRA_SIM_ARGS="$EXTRA_SIM_ARGS $1"; shift ;;
    esac
done

# Auto-detect a display with hardware GL if DISPLAY is not set
if [[ -z "${DISPLAY:-}" ]]; then
    for d in ":1" ":0"; do
        if DISPLAY="$d" glxinfo >/dev/null 2>&1; then
            export DISPLAY="$d"
            echo "[viewer-mode] Auto-detected display $DISPLAY with hardware GL"
            break
        fi
    done
    if [[ -z "${DISPLAY:-}" ]]; then
        echo "[viewer-mode] ERROR: No display with hardware GL found. Set DISPLAY or use --display."
        exit 1
    fi
fi

echo "[viewer-mode] Using DISPLAY=$DISPLAY"

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

# 1. Launch genesis_sim_runner with --viewer on the real display
echo "[viewer-mode] Starting sim runner (viewer mode, ${RES})..."
uv run "$SCRIPT_DIR/genesis_sim_runner.py" \
    --viewer \
    --camera-res "$RES" \
    --fps "$FPS" \
    $CHECKPOINT \
    $EXTRA_SIM_ARGS &
SIM_PID=$!
PIDS+=($SIM_PID)

# 2. Wait for viewer window to appear (up to 60s — Genesis init can be slow)
echo "[viewer-mode] Waiting for viewer window on $DISPLAY..."
VIEWER_WID=""
for i in $(seq 1 60); do
    VIEWER_WID=$(xdotool search --name "." 2>/dev/null | head -1) || true
    if [[ -n "$VIEWER_WID" ]]; then
        echo "[viewer-mode] Viewer window detected (WID: $VIEWER_WID)"
        break
    fi
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "[viewer-mode] ERROR: Sim runner exited before viewer appeared"
        exit 1
    fi
    sleep 1
done

if [[ -z "$VIEWER_WID" ]]; then
    echo "[viewer-mode] ERROR: No viewer window detected after 60s"
    exit 1
fi

# 3. Move viewer window offscreen so it doesn't clutter the desktop
xdotool windowmove "$VIEWER_WID" -9999 -9999 2>/dev/null || true
echo "[viewer-mode] Viewer window moved offscreen"

# Small delay for the window to settle after move
sleep 0.5

# 4. Launch capture sidecar targeting the specific window
echo "[viewer-mode] Starting capture (${RES} @ ${FPS}fps, quality=${QUALITY}, window=$VIEWER_WID)..."
uv run "$SCRIPT_DIR/viewer_capture.py" \
    --display "$DISPLAY" \
    --window-id "$VIEWER_WID" \
    --res "$RES" \
    --fps "$FPS" \
    --quality "$QUALITY" &
PIDS+=($!)

echo "[viewer-mode] All processes running. Press Ctrl-C to stop."
echo "[viewer-mode]   Sim runner PID: $SIM_PID"
echo "[viewer-mode]   Capture PID:    ${PIDS[-1]}"

# Wait for sim runner to exit (it's the primary process)
wait "$SIM_PID"
