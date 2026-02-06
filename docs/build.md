# Build Log

Chronological record of local builds, Docker image builds, and infrastructure milestones.

## 2026-02-05 — Phase 1: CUDA Docker + Basic Pipeline

**Branch:** `feature/phase1-cuda-docker-pipeline`
**PR:** [#1](https://github.com/Gbrothers1/SDR_OS/pull/1)
**Merged:** `dcb1c92` (2026-02-05)

### Docker Compose

Expanded from 3 services to 12 services across 7 profiles.

```bash
docker compose config --quiet   # Validates all 12 services
```

### Container builds

| Container | Command | Result |
|-----------|---------|--------|
| ROS2 Jazzy | `docker compose --profile dev build ros-bridge` | `ROS_DISTRO=jazzy`, rosbridge present |
| CUDA | `docker compose --profile cuda build app-cuda` | `nvidia/cuda:12.4.1-runtime-ubuntu22.04` |
| ROCm | `docker compose --profile rocm build app-rocm` | `rocm/rocm-terminal:6.1.2` |
| MLX | `docker compose --profile mlx build app-mlx` | `python:3.11-slim` |

### Unit tests

```bash
PYTHONPATH=src pytest tests/unit/ -v
```

17/17 passed — SHM ringbuffer protocol (pack/unpack, CRC, lap detection, keyframe resync, context managers).

### NVENC validation

```bash
docker compose --profile cuda run --rm app-cuda python3 scripts/validate_nvenc.py
```

Output (RTX 2080 Ti):

```json
{
  "cuda": {"cuda_available": true, "cuda_version": "12.4", "device_count": 1},
  "nvenc_codecs": {"h264_nvenc": true, "hevc_nvenc": true},
  "nvenc_probe": {"functional": true, "method": "pyav"},
  "status": "NVENC_OK"
}
```

### CI results (post-merge)

| Job | Status | Time |
|-----|--------|------|
| `validate-compose` | Pass | 13s |
| `lint-and-unit` | Pass | 9s |
| `docs-build` | Pass | 17s |
| `ros-jazzy-build` | Pass | 42s |
| `linux-cpu-matrix (cuda)` | Pass | 49s |
| `linux-cpu-matrix (rocm)` | Pass | 40s |
| `linux-cpu-matrix (mlx_cpu)` | Pass | 37s |
| `cuda-smoke` | Pending | Self-hosted runner not configured |
| `rocm-smoke` | Pending | Self-hosted runner not configured |
| `mlx-smoke` | Pending | Self-hosted runner not configured |

### CI fixes applied during Phase 1

| Issue | Root cause | Fix |
|-------|-----------|-----|
| `uv sync --frozen --group docs` fails | `uv.lock` stale after `pyproject.toml` group changes | Regenerated with `uv lock` |
| `torch==2.0.1` no Python 3.12 wheels | `genesis-world` pins `torch==2.0.1` (cp310/cp311 only) | Removed `uv sync` from jobs that don't need torch; added `-p 3.11` to compat-matrix |
| `mkdocs build` fails on `repo_url: .` | MkDocs requires full URL with scheme | Set `repo_url: https://github.com/Gbrothers1/SDR_OS` |

### Verification script

```bash
./scripts/verify.sh         # CPU-only (4 checks, ~2s)
./scripts/verify.sh --gpu   # + Docker builds (~2min)
./scripts/verify.sh --all   # + NVENC on GPU (~5min)
```

---

## 2026-02-05 — Project restructure

**Commits:** `9cb162e`..`9492523`

- Archived original codebase to `_archive/`
- Restructured for `uv` (Python) + `pnpm` (Node.js)
- Added `pyproject.toml`, `.gitignore`, reference materials
- Created MkDocs documentation site
- Added multi-service backend design and Phase 2 Rust transport server plan

---

## 2025-04 — Original development

**Commits:** `0f60091`..`6c6e435`

Initial SDR_OS development:

- React 18 + Three.js frontend with 40+ components
- Node.js/Express + Socket.io backend
- ROS2 integration via rosbridge
- Gamepad controller with deadman switch
- Telemetry panel (IMU, odometry, GPS)
- Webcam streaming via ROS `web_video_server`
- Multi-client controller state sync
- PyQt6 desktop fallback app
