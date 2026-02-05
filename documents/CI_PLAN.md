# CI/CD Plan (GPU-Targeted)

Date: 2026-02-05

## Overview
- CI validates three hardware targets: CUDA, ROCm, and MLX.
- CPU-only checks run on every PR.
- GPU smoke tests run on target-specific runners with minimal duration.

## Runner Placement
- **CUDA**: Linux GPU runners with NVIDIA drivers and Docker + `nvidia-container-toolkit`.
- **ROCm**: Linux GPU runners with ROCm drivers and `/dev/kfd` access.
- **MLX**: Apple Silicon macOS runners (no VM). Runs directly on host.

## Test Split
- `tests/unit`: fast CPU-only; runs on every runner.
- `tests/integration`: GPU-optional; runs on CUDA/ROCm/MLX runners when available.
- `tests/smoke`:
  - CUDA: single short sim step + training warmup (seconds).
  - ROCm: same test suite with ROCm adapter.
  - MLX: short inference + minimal training loop in MLX backend.

## Pipeline Stages
1. **Lint/Format (CPU)**: `ruff`, `pytest -q tests/unit`, frontend lint if applicable.
2. **Build Images (CPU)**: `docker build` for `containers/cuda` and `containers/rocm`.
3. **GPU Smoke (Targeted)**:
   - CUDA: `docker compose --profile cuda run --rm app-cuda pytest -q tests/smoke`.
   - ROCm: `docker compose --profile rocm run --rm app-rocm pytest -q tests/smoke`.
   - MLX: `uv sync --frozen && pytest -q tests/smoke` on macOS host.
4. **Docs Build (CPU)**:
   - Build docs site with MkDocs (Material) using `mkdocs build`.
   - For tags/releases, publish versioned docs with `mike deploy <tag> --push`.
5. **Artifact/Release**: publish build metadata and docker tags if required.

## Additional Workflows
- **Compatibility Matrix**: CPU-only integration tests across backend dependency groups on every PR touching core code.
- **GPU Benchmarks**: scheduled weekly + manual trigger; upload benchmark artifacts per backend.
- **Perf Gate**: manual workflow to compare current perf against a baseline (block release if regression).

## Vercel (If Used)
- Vercel deploys only the web UI and static assets.
- No GPU tests on Vercel; it triggers preview builds from `feat/*` and production from `main`.
- CI should pass before Vercel deploys to production.

## Minimal Overhead Guidance
- Prefer containerized GPU tests on bare metal runners.
- Avoid VMs; use macOS host runners for MLX tests.
- Keep smoke tests under 5 minutes per target.

## Environment Reproducibility
- Always enforce `uv.lock` for Python.
- Enforce `package-lock.json` or `pnpm-lock.yaml` for Node.

## MLX Practicalities
- macOS Apple Silicon: `pip install mlx` (GPU).
- Linux parity: `pip install mlx[cpu]` for CPU-only; `mlx[cuda12]` / `mlx[cuda13]` on CUDA Linux.
- CI should treat MLX Linux installs as interface/compat checks; performance tests stay on macOS GPUs.
