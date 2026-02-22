#!/usr/bin/env bash
# run_sim_viewer.sh — Launch genesis sim in viewer mode on Steam Deck
#
# Auto-detects DISPLAY and XAUTHORITY. Works from distrobox containers
# by reading the host plasmashell environment via distrobox-host-exec.
#
# Usage: ./scripts/run_sim_viewer.sh [extra args for genesis_sim_runner.py]

set -euo pipefail

# ── Find DISPLAY and XAUTHORITY ──
if [ -z "${DISPLAY:-}" ]; then
    # Inside distrobox: read from host plasmashell process
    if command -v distrobox-host-exec &>/dev/null; then
        HOST_ENV=$(distrobox-host-exec bash -c '
            PID=$(pgrep -f "/usr/bin/plasmashell" | head -1)
            [ -n "$PID" ] && tr "\0" "\n" < /proc/$PID/environ 2>/dev/null | grep -E "^(XAUTHORITY|DISPLAY)="
        ' 2>/dev/null) || true
        if [ -n "$HOST_ENV" ]; then
            eval "$HOST_ENV"
            export DISPLAY XAUTHORITY
        fi
    fi

    # Fallback: parse kwin_wayland cmdline for --xwayland-display / --xwayland-xauthority
    if [ -z "${DISPLAY:-}" ]; then
        for procfs in /run/host/proc /proc; do
            KWIN_PID=$(pgrep -f "kwin_wayland.*--xwayland-display" 2>/dev/null | head -1) || true
            [ -n "$KWIN_PID" ] && [ -r "$procfs/$KWIN_PID/cmdline" ] || continue
            CMDLINE=$(tr '\0' ' ' < "$procfs/$KWIN_PID/cmdline" 2>/dev/null) || true
            DISPLAY=$(echo "$CMDLINE" | grep -oP '(?<=--xwayland-display )\S+') || true
            XAUTH=$(echo "$CMDLINE" | grep -oP '(?<=--xwayland-xauthority )\S+') || true
            if [ -n "$DISPLAY" ]; then
                export DISPLAY
                [ -n "$XAUTH" ] && [ -f "$XAUTH" ] && export XAUTHORITY="$XAUTH"
                break
            fi
        done
    fi

    # Last resort: try displays that exist
    if [ -z "${DISPLAY:-}" ]; then
        for d in :0 :1 :2; do
            [ -e "/tmp/.X11-unix/X${d#:}" ] || continue
            if xdpyinfo -display "$d" >/dev/null 2>&1; then
                export DISPLAY="$d"
                break
            fi
        done
    fi

    if [ -z "${DISPLAY:-}" ]; then
        echo "ERROR: No accessible X display found. Run from desktop mode." >&2
        exit 1
    fi
fi

echo "Using DISPLAY=$DISPLAY XAUTHORITY=${XAUTHORITY:-<none>}"

# Verify display is accessible
if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    echo "ERROR: Cannot connect to display $DISPLAY" >&2
    echo "  XAUTHORITY=${XAUTHORITY:-<unset>}" >&2
    exit 1
fi

# ── AMD GPU env vars ──
export HSA_OVERRIDE_GFX_VERSION="${HSA_OVERRIDE_GFX_VERSION:-10.3.0}"
export HSA_ENABLE_SDMA="${HSA_ENABLE_SDMA:-0}"
export HIP_LAUNCH_BLOCKING="${HIP_LAUNCH_BLOCKING:-1}"

# ── Launch viewer ──
exec uv run scripts/genesis_sim_runner.py \
    --camera-res 1280x720 --fps 60 --viewer "$@"
