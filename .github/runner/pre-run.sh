#!/usr/bin/env bash
# Pre-job hook: verify GPU and Docker are healthy before each CI job.
# Set ACTIONS_RUNNER_HOOK_JOB_STARTED in .env to point here.
#
# This runs before every job on this runner. If it exits non-zero,
# the job fails fast instead of timing out on broken infrastructure.

set -euo pipefail

echo "=== Pre-run GPU health check ==="

# GPU accessible?
if ! nvidia-smi --query-gpu=name,memory.free --format=csv,noheader 2>/dev/null; then
    echo "FATAL: GPU not accessible"
    exit 1
fi

# Docker daemon alive?
if ! docker info &>/dev/null; then
    echo "FATAL: Docker daemon not responding"
    exit 1
fi

# Enough disk? (fail if <20 GB free)
AVAIL_GB=$(df --output=avail / | tail -1 | awk '{printf "%.0f", $1/1024/1024}')
if (( AVAIL_GB < 20 )); then
    echo "FATAL: Only ${AVAIL_GB} GB free on /. Need at least 20 GB."
    exit 1
fi

# Enough RAM? (fail if <1 GB available)
AVAIL_MB=$(awk '/MemAvailable/ {printf "%.0f", $2/1024}' /proc/meminfo)
if (( AVAIL_MB < 1024 )); then
    echo "WARN: Only ${AVAIL_MB} MB RAM available. Jobs may OOM."
fi

echo "GPU: OK | Docker: OK | Disk: ${AVAIL_GB} GB free | RAM: ${AVAIL_MB} MB available"
