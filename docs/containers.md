# Containers

SDR_OS uses Docker Compose with profiles for both the multi-service production stack and reproducible CI environments for each GPU backend.

## Docker Compose

### Production services (`sim` profile)

The `sim` profile starts the full simulation stack:

```bash
docker compose --profile sim up --build -d
```

| Service | Image | Network | Port | Purpose |
|---------|-------|---------|------|---------|
| `webserver` | `caddy:2-alpine` | edge + backplane | 80, 443 | Reverse proxy, static file server |
| `node` | `sdr_os-node` | backplane | 3000 (internal) | Socket.io gamepad relay, /api |
| `transport-server` | `sdr_os-transport` | edge + backplane | 8080 | SHM→WS fanout, NATS relay, video gate |
| `genesis-sim` | `sdr_os-cuda` | host | — | Genesis simulation, NVENC/JPEG encoder, SHM writer |
| `nats` | `nats:2-alpine` | backplane | 4222, 8222 | Message broker (command + telemetry) |
| `ros-bridge` | `sdr_os-ros-jazzy` | host | 9090 | ROS 2 rosbridge WebSocket |

### CI targets

| Service | Profile | Base image | GPU access |
|---------|---------|-----------|------------|
| `app-cuda` | `cuda` | `nvidia/cuda:12.4.1-runtime-ubuntu22.04` | NVIDIA Container Toolkit (all GPUs) |
| `app-rocm` | `rocm` | `rocm/rocm-terminal:6.1.2` | `/dev/kfd` + `/dev/dri`, `video` group |
| `app-mlx` | `mlx` | `python:3.11-slim` | None (CPU parity on Linux) |

All CI services mount the repo as `/workspace` and allocate 2 GB shared memory.

### All profiles

| Profile | Services | Use |
|---------|----------|-----|
| `sim` | webserver, node, transport-server, genesis-sim, nats, ros-bridge | Full simulation stack |
| `dev` | webserver, node, ros-bridge | Web development (no sim) |
| `train` | training-runner, nats | RL training |
| `obs` | prometheus, grafana | Observability |
| `cuda` | app-cuda | CUDA CI target |
| `rocm` | app-rocm | ROCm CI target |
| `mlx` | app-mlx | MLX CI target |

### Common commands

```bash
# Full simulation stack
docker compose --profile sim up --build -d
docker compose --profile sim down

# Rebuild individual services
docker compose build node               # After server.js changes
docker compose build transport-server   # After Rust code changes
docker compose build genesis-sim        # After Dockerfile/dependency changes

# CI targets
docker compose --profile cuda run --rm app-cuda pytest -q tests/smoke
docker compose --profile rocm run --rm app-rocm pytest -q tests/smoke
```

### Networks

| Network | Type | Purpose |
|---------|------|---------|
| `edge` | bridge | External-facing services (Caddy, transport WS) |
| `backplane` | bridge (internal) | Internal services (NATS, node); no external DNS |

### Shared volumes

| Volume | Type | Purpose |
|--------|------|---------|
| `sdr_ipc` | tmpfs (512 MB) | SHM ringbuffer for zero-copy video frames (`/dev/shm/sdr_os_ipc`) |
| `nats_data` | named | NATS JetStream persistence |

## Dockerfiles

### Node (`containers/node/Dockerfile`)

Minimal Alpine image for the gamepad relay server.

```dockerfile
FROM node:20-alpine
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci --omit=dev
COPY server.js ./
EXPOSE 3000
CMD ["node", "server.js"]
```

### CUDA (`containers/cuda/Dockerfile`)

```dockerfile
FROM nvidia/cuda:12.4.1-runtime-ubuntu22.04
# System deps, Python 3.10, uv
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --group cuda
# cp site-packages to /opt/venv (volume mount workaround)
COPY . /workspace
```

### ROS Jazzy (`containers/ros-jazzy/Dockerfile`)

```dockerfile
FROM ros:jazzy-ros-base
# rosbridge_server, rosbridge_library
COPY scripts/ros/ /workspace/scripts/ros/
CMD ["/workspace/scripts/ros/run_nodes.sh"]
```

### Transport Server (`services/transport-server/Dockerfile`)

Rust multi-stage build (build + runtime). See `services/transport-server/` for source.

### ROCm (`containers/rocm/Dockerfile`)

```dockerfile
FROM rocm/rocm-terminal:6.1.2
# System deps, Python, uv
COPY pyproject.toml uv.lock ./
RUN uv sync --frozen --group rocm
COPY . /workspace
```

### MLX (`containers/mlx/Dockerfile`)

```dockerfile
FROM python:3.11-slim
ARG MLX_VARIANT=mlx_cpu
# System deps, uv
COPY pyproject.toml uv.lock ./
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

## Dependency management

- Python deps: `uv sync --frozen --group <backend>` from `uv.lock`
- Node deps: `npm ci --omit=dev` (from `package-lock.json`)
- Lockfiles are copied before the full repo to maximize Docker layer cache hits
- **Important**: `uv pip sync` is destructive (removes unlisted packages) — use `uv pip install -r` for additive installs
- **Important**: Host `.venv` has Python 3.12, containers have Python 3.10 — don't confuse them
- **Important**: `backplane` network has no external DNS — all packages must be baked into images at build time
