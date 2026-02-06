# Phase 1 Summary: CUDA Docker + Basic Pipeline

**Date:** 2026-02-05
**Branch:** `feature/phase1-cuda-docker-pipeline`

## What Was Built

Phase 1 establishes the Docker infrastructure, ROS2 integration, NVENC validation, and zero-copy IPC for the SDR_OS multi-service backend.

### Deliverables

| # | Deliverable | Status | Key Files |
|---|-------------|--------|-----------|
| 1 | Multi-service Docker Compose | Done | `docker-compose.yml` |
| 2 | ROS2 Jazzy container | Done | `containers/ros-jazzy/Dockerfile` |
| 3 | NVENC validation telemetry | Done | `scripts/validate_nvenc.py` |
| 4 | SHM ringbuffer (full protocol) | Done | `src/sdr_os/ipc/shm_ringbuffer.py` |
| 5 | Documentation | Done | `docs/` |

### Verification

All checks pass:

| Check | Result |
|-------|--------|
| `docker compose config` | Valid |
| `pytest tests/unit/` | 17/17 passed |
| ROS2 Jazzy container | `ROS_DISTRO=jazzy`, rosbridge present |
| NVENC script syntax | Valid |

## Docker Compose Architecture

12 services across 7 profiles with edge/backplane network isolation:

```
Browser → Caddy (edge) → { transport-server, ros-bridge, web app }
                              ↕                    ↕
                        SHM ringbuffer        NATS backplane
                              ↕                    ↕
                        genesis-sim           training-runner
```

### Profiles

| Profile | Services | Use |
|---------|----------|-----|
| `dev` | webserver, ros-bridge | Development |
| `sim` | webserver, genesis-sim, ros-bridge, nats | Full simulation pipeline |
| `train` | training-runner, nats | RL training |
| `obs` | prometheus, grafana | Observability |
| `cuda` | app-cuda | CUDA CI target |
| `rocm` | app-rocm | ROCm CI target |
| `mlx` | app-mlx | MLX CI target |

### Containers

| Image | Base | Purpose |
|-------|------|---------|
| `sdr_os-cuda` | `nvidia/cuda:12.4.1-runtime-ubuntu22.04` | CUDA workloads + simulation |
| `sdr_os-rocm` | `rocm/rocm-terminal:6.1.2` | ROCm CI |
| `sdr_os-mlx` | `python:3.11-slim` | MLX Linux parity |
| `sdr_os-ros-jazzy` | `ros:jazzy-ros-base` | ROS2 + rosbridge (host network) |

## SHM Ringbuffer Protocol

Zero-copy frame transport for the video pipeline:

```
genesis-sim → [SHM ringbuffer] → transport-server → clients
```

### Layout

```
[0:8]   sequence_number  (written LAST - commit marker)
[8:16]  write_index
[16:]   frame data (header + payload)
```

### Frame Header (32 bytes)

| Field | Type | Description |
|-------|------|-------------|
| frame_id | uint64 | Application-level ID |
| frame_seq | uint64 | Monotonic sequence (lap detection) |
| size | uint32 | Payload size in bytes |
| flags | uint16 | KEYFRAME, WRAP_MARKER |
| codec | uint16 | 1=H.264, 2=HEVC |
| crc32 | uint32 | Payload integrity check |
| reserved | uint32 | Future use |

### Key Properties

- **Atomic commit**: Writer writes data first, sequence number last. Reader checks seq before and after read — rejects torn reads.
- **Lap detection**: Reader tracks `last_frame_seq`. If frame_seq jumps or goes backwards, resync to next keyframe.
- **Drop policy**: "Latest wins" — writer always overwrites with freshest frame. Reader must keep up.
- **CRC validation**: Every frame payload verified with `zlib.crc32`.

## NVENC Validation

`scripts/validate_nvenc.py` checks GPU capability inside the CUDA container:

- CUDA availability via PyTorch
- GPU info via `nvidia-smi` (driver, model, active NVENC sessions)
- Codec support via ffmpeg and PyAV (`h264_nvenc`, `hevc_nvenc`)
- Functional probe (attempts a real encode)

Output is structured JSON for telemetry collection.

## CI Pipeline

### CPU Jobs (free GitHub runners, every PR)

| Job | What |
|-----|------|
| `validate-compose` | Docker Compose YAML validation |
| `lint-and-unit` | Python unit tests (SHM ringbuffer) |
| `ros-jazzy-build` | Build + verify ROS2 Jazzy container |
| `docs-build` | MkDocs site builds |

### GPU Jobs (self-hosted runners)

| Job | Runner | What |
|-----|--------|------|
| `cuda-smoke` | `linux, gpu, nvidia` | NVENC validation + smoke tests |
| `rocm-smoke` | `linux, gpu, rocm` | ROCm smoke tests |
| `mlx-smoke` | `macos, arm64, mlx` | MLX smoke tests |

## Files Created/Modified

### New Files

```
containers/ros-jazzy/Dockerfile        # ROS2 Jazzy container
.devcontainer/ros-jazzy/devcontainer.json
configs/Caddyfile                      # Reverse proxy config
configs/nats.conf                      # NATS JetStream config
configs/prometheus.yml                 # Metrics scraping
scripts/ros/run_nodes.sh               # ROS2 node launcher
scripts/validate_nvenc.py              # NVENC telemetry
scripts/verify.sh                      # Local verification script
justfile                               # Task runner
src/sdr_os/__init__.py
src/sdr_os/ipc/__init__.py
src/sdr_os/ipc/shm_ringbuffer.py       # SHM ringbuffer protocol
tests/unit/test_shm_ringbuffer.py      # 17 unit tests
docs/testing.md                        # Testing guide
docs/github-actions.md                 # GitHub Actions explainer
docs/phase1-summary.md                 # This file
```

### Modified Files

```
docker-compose.yml                     # 3 services → 12 services, 7 profiles
containers/cuda/Dockerfile             # Fixed COPY globs
containers/rocm/Dockerfile             # Fixed COPY globs
containers/mlx/Dockerfile              # Fixed COPY globs
.github/workflows/ci.yml               # Added compose, ROS2, unit test jobs
mkdocs.yml                             # Added new nav pages
docs/index.md                          # Updated with quick start
docs/architecture.md                   # Updated with service architecture
docs/ci.md                             # Updated with container details
```

## What's Next: Phase 2

**Rust Transport Server** — the next piece of the video pipeline:

1. Scaffold Rust crate (`services/transport-server/`) with tokio + axum
2. Implement SHM reader with lap detection (Rust version of the Python ringbuffer)
3. WebSocket frame fanout to browser clients
4. NATS telemetry relay
5. Integration test with genesis-sim
