# Containers

SDR_OS uses Docker Compose with profiles to provide reproducible environments for each GPU backend. VS Code devcontainers are configured for each target.

## Docker Compose

The `docker-compose.yml` defines three services (one per backend) using Compose profiles:

| Service | Profile | Base image | GPU access |
|---------|---------|-----------|------------|
| `app-cuda` | `cuda` | `nvidia/cuda:12.4.1-runtime-ubuntu22.04` | NVIDIA Container Toolkit (all GPUs) |
| `app-rocm` | `rocm` | `rocm/rocm-terminal:6.1.2` | `/dev/kfd` + `/dev/dri`, `video` group |
| `app-mlx` | `mlx` | `python:3.11-slim` | None (CPU parity on Linux) |

All services mount the repo as `/workspace` and allocate 2 GB shared memory.

### Common commands

```bash
# Build a specific backend
docker compose --profile cuda build
docker compose --profile rocm build
docker compose --profile mlx build

# Run interactively
docker compose --profile cuda run --rm app-cuda bash

# Run smoke tests
docker compose --profile cuda run --rm app-cuda pytest -q tests/smoke
docker compose --profile rocm run --rm app-rocm pytest -q tests/smoke
```

### MLX variant switching

The MLX container supports multiple variants via the `MLX_VARIANT` build arg:

| Variant | Use case |
|---------|----------|
| `mlx_cpu` (default) | Linux CPU parity mode |
| `mlx_cuda12` | NVIDIA Linux with CUDA 12 |
| `mlx_cuda13` | NVIDIA Linux with CUDA 13 |
| `mlx` | macOS Apple Silicon (native, not containerized) |

```bash
# Override at build time
MLX_VARIANT=mlx_cuda12 docker compose --profile mlx build app-mlx

# Override at run time
MLX_VARIANT=mlx_cuda12 docker compose --profile mlx run --rm app-mlx bash
```

## Dockerfiles

All three Dockerfiles follow the same pattern:

1. Install system packages (Python, Node, git, build-essential)
2. Create a virtualenv at `/opt/venv`
3. Install `uv` for Python package management
4. Copy lockfiles first (maximizes Docker layer cache)
5. Install deps from lockfile using the appropriate dependency group
6. Copy full repo

### CUDA (`containers/cuda/Dockerfile`)

```dockerfile
FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04
# ... system deps ...
COPY pyproject.toml uv.lock ./
COPY requirements/requirements-cuda.txt ./requirements/
RUN uv sync --frozen --group cuda
COPY . /workspace
```

### ROCm (`containers/rocm/Dockerfile`)

```dockerfile
FROM rocm/rocm-terminal:6.1.2
# ... system deps ...
COPY pyproject.toml uv.lock ./
COPY requirements/requirements-rocm.txt ./requirements/
RUN uv sync --frozen --group rocm
COPY . /workspace
```

### MLX (`containers/mlx/Dockerfile`)

```dockerfile
FROM python:3.11-slim
ARG MLX_VARIANT=mlx_cpu
# ... system deps ...
COPY pyproject.toml uv.lock ./
COPY requirements/requirements-mlx*.txt ./requirements/
RUN uv sync --frozen --group ${MLX_VARIANT}
COPY . /workspace
```

## Devcontainers

Three VS Code devcontainer configs in `.devcontainer/`:

| Config | Service | Workspace |
|--------|---------|-----------|
| `.devcontainer/cuda/devcontainer.json` | `app-cuda` | `/workspace` |
| `.devcontainer/rocm/devcontainer.json` | `app-rocm` | `/workspace` |
| `.devcontainer/mlx/devcontainer.json` | `app-mlx` | `/workspace` |

All reference `docker-compose.yml` and use `stopCompose` as the shutdown action. Open the repo in VS Code, select "Reopen in Container", and choose the target backend.

## Planned multi-service architecture

Phase 1 defines a more granular Compose setup (see [Architecture](architecture.md)):

| Service | Image | Role | Network |
|---------|-------|------|---------|
| webserver | Caddy 2 | Reverse proxy, TLS | edge + backplane |
| genesis-sim | Custom (CUDA) | Simulation + NVENC | backplane |
| ros-bridge | Custom (ROS 2 Jazzy) | ROS 2 to NATS relay | backplane |
| training-runner | Custom (CUDA) | RL training | backplane |
| nats | Official NATS | Message bus + JetStream | backplane |
| transport-server | Custom (Rust) | Frame fanout, WebSocket | backplane |

This uses two Docker networks (edge + backplane) with tmpfs shared memory (`/dev/shm/sdr_os_ipc`) for zero-copy video frames.

## Dependency management in containers

- Python deps installed via `uv sync --frozen --group <backend>` from `uv.lock`
- Fallback: `uv pip sync requirements/requirements-<backend>.txt`
- Node deps installed via `npm ci` (from `package-lock.json`) or `pnpm install --frozen-lockfile`
- Lockfiles are copied before the full repo to maximize Docker layer cache hits
