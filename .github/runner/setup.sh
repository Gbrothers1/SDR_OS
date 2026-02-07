#!/usr/bin/env bash
# SDR_OS Self-Hosted GPU Runner Setup
# Run as the user who will own the runner (not root).
#
# Prerequisites:
#   - Ubuntu 22.04+ with NVIDIA GPU
#   - NVIDIA Container Toolkit installed
#   - Docker installed and user in 'docker' group
#   - GitHub personal access token or org-level runner token
#
# Usage:
#   ./setup.sh <GITHUB_RUNNER_TOKEN>
#
# The token is single-use and obtained from:
#   Settings → Actions → Runners → New self-hosted runner
#   Or via API: gh api repos/Gbrothers1/SDR_OS/actions/runners/registration-token

set -euo pipefail

RUNNER_VERSION="2.321.0"
RUNNER_DIR="${HOME}/actions-runner"
REPO_URL="https://github.com/Gbrothers1/SDR_OS"
LABELS="self-hosted,linux,gpu,nvidia"

# ── Preflight checks ──────────────────────────────────────────────
echo "=== SDR_OS GPU Runner Setup ==="

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <GITHUB_RUNNER_TOKEN>"
    echo ""
    echo "Get a token from:"
    echo "  gh api -X POST repos/Gbrothers1/SDR_OS/actions/runners/registration-token --jq .token"
    exit 1
fi

TOKEN="$1"

echo "Checking prerequisites..."

# Docker
if ! command -v docker &>/dev/null; then
    echo "ERROR: Docker not installed. Install with: sudo apt install docker.io"
    exit 1
fi
if ! docker info &>/dev/null; then
    echo "ERROR: Docker daemon not running or user not in docker group."
    echo "Fix: sudo usermod -aG docker $USER && newgrp docker"
    exit 1
fi

# NVIDIA
if ! command -v nvidia-smi &>/dev/null; then
    echo "ERROR: nvidia-smi not found. Install NVIDIA drivers."
    exit 1
fi
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader
echo ""

# NVIDIA Container Toolkit
if ! docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi &>/dev/null; then
    echo "ERROR: NVIDIA Container Toolkit not working."
    echo "Fix: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html"
    exit 1
fi
echo "NVIDIA Container Toolkit: OK"

# ── Download runner ───────────────────────────────────────────────
echo ""
echo "Installing GitHub Actions runner v${RUNNER_VERSION}..."

mkdir -p "${RUNNER_DIR}"
cd "${RUNNER_DIR}"

if [[ ! -f ./config.sh ]]; then
    ARCH=$(uname -m)
    case "$ARCH" in
        x86_64) RUNNER_ARCH="linux-x64" ;;
        aarch64) RUNNER_ARCH="linux-arm64" ;;
        *) echo "Unsupported architecture: $ARCH"; exit 1 ;;
    esac

    TARBALL="actions-runner-${RUNNER_ARCH}-${RUNNER_VERSION}.tar.gz"
    curl -fsSL -o "${TARBALL}" \
        "https://github.com/actions/runner/releases/download/v${RUNNER_VERSION}/${TARBALL}"
    tar xzf "${TARBALL}"
    rm "${TARBALL}"
fi

# ── Configure runner ──────────────────────────────────────────────
echo ""
echo "Configuring runner with labels: ${LABELS}"

./config.sh \
    --url "${REPO_URL}" \
    --token "${TOKEN}" \
    --labels "${LABELS}" \
    --name "$(hostname)-gpu" \
    --work "_work" \
    --replace

# ── Install as service ────────────────────────────────────────────
echo ""
echo "Installing systemd service..."

sudo ./svc.sh install "${USER}"
sudo ./svc.sh start

echo ""
echo "=== Runner installed and started ==="
echo "Runner name: $(hostname)-gpu"
echo "Labels: ${LABELS}"
echo "Work dir: ${RUNNER_DIR}/_work"
echo ""
echo "Commands:"
echo "  sudo ./svc.sh status    # Check status"
echo "  sudo ./svc.sh stop      # Stop runner"
echo "  sudo ./svc.sh start     # Start runner"
echo "  journalctl -u actions.runner.*.service -f  # View logs"
