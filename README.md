# SDR_OS

SDR_OS is a multi-backend robotics simulation and control platform for low-latency teleoperation, Genesis simulation, and RL training. It targets **CUDA** (NVIDIA), **ROCm** (AMD), and **MLX** (Apple Silicon).

## Features

- Browser-based control (gamepad/keyboard) with real-time telemetry and video
- Genesis physics simulation with GPU encoding (NVENC) and streaming to the client
- ROS2 integration for real robots and sensor pipelines
- Multi-service backend: Rust transport server, Python sim/ROS bridge/training, NATS message bus
- Single source tree with backend-specific containers (CUDA, ROCm, MLX)

## Quick start

**Requirements:** Python ≥3.10 (&lt;3.14), [uv](https://docs.astral.sh/uv/), [pnpm](https://pnpm.io/) (use these instead of pip/npm).

```bash
git clone <repo-url>
cd SDR_OS

# Python
uv sync

# Node (if using the web UI)
pnpm install

# Verify
uv run python -c "import genesis; print(genesis.__version__)"
```

See `documents/SETUP.md` for full environment setup (PyTorch, Genesis, CUDA, etc.).

## Backends

| Target | Use case |
|--------|----------|
| **CUDA** | NVIDIA GPUs (Linux); primary path for Genesis + NVENC |
| **ROCm** | AMD GPUs (Linux) |
| **MLX** | Apple Silicon (macOS); Linux parity via CPU or CUDA variants |

Containers: `containers/cuda/`, `containers/rocm/`, `containers/mlx/`. Compose profiles: `cuda`, `rocm`, `mlx`.

## Repository layout

| Path | Purpose |
|------|---------|
| `src/` | Shared source (single codebase) |
| `containers/` | Backend-specific Dockerfiles (cuda, rocm, mlx) |
| `docs/` | Release-facing documentation (MkDocs) |
| `documents/` | Internal architecture and planning |

## Documentation

- **Built from `docs/`** with MkDocs; versioned with Git tags via `mike`.
- **Local docs build:**

  ```bash
  pip install -r requirements/requirements-docs.txt
  mkdocs serve
  ```

  Or with uv: `uv sync --group docs` then `uv run mkdocs serve`.

- **Key internal docs:**  
  `documents/ARCHITECTURE_OVERVIEW.md`, `documents/NEW_ARCHITECTURE_OVERVIEW.md`, `documents/CI_PLAN.md`, `docs/plans/2026-02-05-multi-service-backend-design.md`.

## License

GPL-3.0. See `LICENSE`. Documentation and code are released under the [GNU General Public License v3.0](https://choosealicense.com/licenses/gpl-3.0/).
