# SDR_OS Multi-Target Dev + CI/CD Plan

Date: 2026-02-05

## Objectives
- Single source tree under `src/` with shared logic and minimal per-backend deltas.
- Reproducible dev/test environments via containers plus lockfiles (`uv.lock`, `package-lock.json` or `pnpm-lock.yaml`).
- GPU-enabled CI smoke tests for sim/training on CUDA, ROCm, and MLX targets.
- Low operational/RAM overhead: containers, no VMs.

## Target Matrix
- **CUDA**: NVIDIA desktop/server GPUs (Linux).
- **ROCm**: AMD GPUs including Steam Deck-class and workstation (Linux).
- **MLX**: Apple Silicon Macs (macOS host).

## Proposed Source Layout (single tree)
```
src/
  sdr_os/
    core/
      config/
      io/
      math/
      telemetry/
    sim/
      common/
      interfaces/
    backends/
      cuda/
        __init__.py
        torch_adapter.py
        genesis_adapter.py
      rocm/
        __init__.py
        torch_adapter.py
        genesis_adapter.py
      mlx/
        __init__.py
        mlx_adapter.py
    training/
      rsl_rl/
      policies/
      runners/
    gateway/
      api/
      signaling/
    ui/
      client/
  tests/
    unit/
    integration/
    smoke/
```

## Backend Abstraction Rules
- Common interfaces live in `src/sdr_os/sim/interfaces/` and `src/sdr_os/backends/` only adapt device specifics.
- Any unavoidable backend-specific code must be isolated to `src/sdr_os/backends/<target>/`.
- Shared config schemas live in `src/sdr_os/core/config/` with per-backend overrides via env.

## Containers and Devcontainers
- Containers provide reproducible Linux environments for CUDA/ROCm; MLX container is tooling-only.
- Devcontainers map to the compose services.
- Lockfiles drive installs inside containers for deterministic builds.

Files:
- `docker-compose.yml` with profiles: `cuda`, `rocm`, `mlx`.
- `containers/cuda/Dockerfile`, `containers/rocm/Dockerfile`, `containers/mlx/Dockerfile`.
- `.devcontainer/cuda/devcontainer.json`, `.devcontainer/rocm/devcontainer.json`, `.devcontainer/mlx/devcontainer.json`.

## CI/CD Plan (summary)
- **CUDA**: GPU runners on Linux (self-hosted or managed GPU runners). Smoke tests run inside `app-cuda`.
- **ROCm**: Linux GPU runners with ROCm drivers and `/dev/kfd` access. Smoke tests in `app-rocm`.
- **MLX**: macOS Apple Silicon runner (no VM). Run tests on host with `uv sync --frozen`.
- **Split tests**:
  - `tests/unit`: CPU-only, runs everywhere.
  - `tests/integration`: GPU-optional; gated by target profile.
  - `tests/smoke`: short sim/training loop per backend.
- **Docs**: MkDocs site under `docs/` with versioned publishing via Git tags.

Full details in `documents/CI_PLAN.md`.

## Git Workflow
- Branches: `main`, `dev`, `feat/<short-desc>`, `fix/<short-desc>`.
- Branches represent features/experiments, not hardware targets.
- Use worktrees for parallel target validation:
  - `worktree-cuda`, `worktree-rocm`, `worktree-mlx`.
- Sparse checkout (optional): for storage-constrained devices, check out only `src/` and needed container files.
- Submodules/subtree: for large pinned vendor sources (e.g., `mlx-examples` or a custom sim engine); prefer lockfiles for standard deps.
- Cherry-pick/rebase: backport fixes across targets without merging unrelated changes.
- Git LFS: store large model weights/datasets (e.g., `.safetensors`) to avoid bloating the repo.
- Hooks:
  - `pre-commit`: run `uv sync --frozen` check, `ruff`/`pytest -q tests/unit` (CPU only).
  - `pre-push`: optional `pytest -q tests/integration` when GPU available.

## MLX Backend Notes
- MLX targets Apple Silicon GPUs on macOS via `pip install mlx`.
- Linux support exists for API parity: `pip install mlx[cpu]` (CPU) or `pip install mlx[cuda12]` / `mlx[cuda13]`.
- In CI, MLX GPU tests run on macOS runners; Linux can still validate MLX interfaces with CPU or CUDA backends.
- MLX from source: clone the repo and run `pip install .` or `pip install -e ".[dev]"`.

## MLX Runtime Guidance
- Write MLX scripts in `src/` alongside other backends.
- CUDA container: install `mlx[cuda12]` (or `mlx[cuda13]`) and run on NVIDIA GPU.
- ROCm container: use MLX CPU mode or another framework until AMD support lands.
- macOS host: `pip install mlx` and run on Apple GPU.

## Environment Parity
- Maintain per-backend dependency overlays (CUDA, ROCm, MLX-CPU/CUDA) while keeping one codebase.
- Use feature flags or runtime checks to select backend.
- Compose services share the same code volume but install target-specific dependencies.

### MLX Variant Switching (Compose)
- Default is CPU parity mode for Linux: `MLX_VARIANT=mlx_cpu`.
- To use CUDA MLX on Linux NVIDIA hosts, set `MLX_VARIANT=mlx_cuda12` or `mlx_cuda13`.
- Example (temporary override):
  - `MLX_VARIANT=mlx_cuda12 docker compose --profile mlx build app-mlx`
  - `MLX_VARIANT=mlx_cuda12 docker compose --profile mlx run --rm app-mlx bash`
- To make it permanent, edit `docker-compose.yml` and change `services.app-mlx.build.args.MLX_VARIANT`.
- macOS host MLX stays native; compose MLX is mainly for Linux parity and CI checks.

## Reproducibility
- Python: `uv.lock` is authoritative.
- Node: `package-lock.json` or `pnpm-lock.yaml` enforced in containers.
- Container builds only copy lockfiles first to maximize cache stability.

## References
- Web references for tools and manuals are tracked in `ref/web/reff_web_memory.md`.
