#!/usr/bin/env bash
# docker-build-progress.sh — Monitor a docker compose build with a progress bar
#
# Usage:
#   ./scripts/docker-build-progress.sh <log_file>
#
# Example:
#   docker compose --profile sim up --build -d genesis-sim 2>&1 | tee /tmp/build.log &
#   ./scripts/docker-build-progress.sh /tmp/build.log

set -euo pipefail

LOG_FILE="${1:?Usage: $0 <log_file>}"

if [ ! -f "$LOG_FILE" ]; then
  echo "Log file not found: $LOG_FILE"
  exit 1
fi

while true; do
  last_line=$(tail -1 "$LOG_FILE" 2>/dev/null || echo "")

  # Detect completion
  if echo "$last_line" | grep -qE "Started|Container.*Started"; then
    echo ""
    echo "=== BUILD COMPLETE ==="
    tail -5 "$LOG_FILE"
    break
  fi

  # Detect failure
  if echo "$last_line" | grep -qiE "^ERROR|FAILED|fatal"; then
    echo ""
    echo "=== BUILD FAILED ==="
    tail -10 "$LOG_FILE"
    exit 1
  fi

  # Estimate progress based on Dockerfile step and content
  if echo "$last_line" | grep -qE "exporting to image"; then
    pct=90
  elif echo "$last_line" | grep -qE "Recreat|Creating|Starting"; then
    pct=95
  elif echo "$last_line" | grep -qE "COPY \.|copy"; then
    pct=80
  elif echo "$last_line" | grep -qE "Installed|Installing|\+"; then
    pct=70
  elif echo "$last_line" | grep -qE "Downloaded"; then
    pct=55
  elif echo "$last_line" | grep -qE "Resolved|Resolving"; then
    pct=45
  elif echo "$last_line" | grep -qE "pip install|uv sync"; then
    pct=35
  elif echo "$last_line" | grep -qE "venv"; then
    pct=20
  elif echo "$last_line" | grep -qE "apt-get|dpkg|Setting up"; then
    pct=15
  elif echo "$last_line" | grep -qE "FROM|CACHED"; then
    pct=5
  else
    pct=60
  fi

  # Build the bar (40 chars wide)
  filled=$((pct * 40 / 100))
  empty=$((40 - filled))
  bar=$(printf '█%.0s' $(seq 1 $filled); printf '░%.0s' $(seq 1 $empty))

  # Truncate last line for display
  short=$(echo "$last_line" | sed 's/^#[0-9]* [0-9.]* *//' | cut -c1-50)

  printf "\r[%s] %3d%%  %s          " "$bar" "$pct" "$short"

  sleep 3
done
