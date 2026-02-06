# SDR_OS task runner
# Install: curl -sSf https://just.systems/install.sh | bash
# Usage: just <recipe>

# Default: show available recipes
default:
    @just --list

# ──────── Verification ────────

# Run CPU-only verification (fast, no Docker builds)
verify:
    ./scripts/verify.sh

# Run verification including Docker container builds
verify-gpu:
    ./scripts/verify.sh --gpu

# Run full verification including CUDA GPU checks
verify-all:
    ./scripts/verify.sh --all

# ──────── Tests ────────

# Run unit tests
test:
    PYTHONPATH=src pytest tests/unit/ -v

# Run unit tests with coverage
test-cov:
    PYTHONPATH=src pytest tests/unit/ -v --cov=sdr_os --cov-report=term-missing

# ──────── Docker ────────

# Build ROS2 Jazzy container
build-ros:
    docker compose --profile dev build ros-bridge

# Build CUDA container
build-cuda:
    docker compose --profile cuda build app-cuda

# Start dev environment (ROS2 + Caddy)
dev:
    docker compose --profile dev up

# Start simulation environment (full pipeline)
sim:
    docker compose --profile sim up

# Validate NVENC inside CUDA container
nvenc:
    docker compose --profile cuda run --rm app-cuda python3 scripts/validate_nvenc.py

# Shell into ROS2 container
ros-shell:
    docker compose --profile dev run --rm ros-bridge bash

# Shell into CUDA container
cuda-shell:
    docker compose --profile cuda run --rm app-cuda bash

# ──────── Docs ────────

# Build documentation site
docs:
    mkdocs build

# Serve docs locally
docs-serve:
    mkdocs serve

# ──────── Compose ────────

# Validate docker-compose.yml
compose-check:
    docker compose config --quiet && echo "OK"

# Stop all containers
down:
    docker compose --profile dev --profile sim --profile train --profile obs down
