#!/usr/bin/env bash
# Post-job hook: clean up Docker resources after each CI job.
# Optional — uncomment in .env if disk usage grows over time.

set -euo pipefail

echo "=== Post-run cleanup ==="

# Remove dangling images (untagged intermediates from builds)
docker image prune -f 2>/dev/null || true

# Remove stopped containers from CI jobs
docker container prune -f 2>/dev/null || true

AVAIL_GB=$(df --output=avail / | tail -1 | awk '{printf "%.0f", $1/1024/1024}')
echo "Disk after cleanup: ${AVAIL_GB} GB free"
