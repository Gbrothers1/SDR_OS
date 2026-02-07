# Build Log

Chronological record of local builds, Docker image builds, and infrastructure milestones.

## 2026-02-06 — Phase 2: NATS Backbone, Safety Stack, Transport Integration

**Branch:** `main` (direct commits + merged `ui-reimplement`)
**Commits:** `c0a52ac`..`70d29bc` (17 commits)
**Version:** `0.2.0`

### What changed

Replaced Socket.io Genesis relay with NATS messaging through the Rust transport server. Added a 3-layer safety stack (HOLD/ESTOP) and H.264/JPEG codec pipeline. Added a Node.js Docker service and Caddy production routing.

### Key commits

| Commit | Change |
|--------|--------|
| `7e2bdf8` | Add `0x04` COMMAND protocol, `CODEC_JPEG`, NATS subject schema |
| `533b5ac` | Transport server handles inbound commands, publishes to NATS |
| `97066c0` | Transport Layer 2 video gate (HOLD 1s, ESTOP 5s) |
| `c24ebfd` | Genesis sim: replace Socket.io with NATS for commands + telemetry |
| `2c57dff` | Frontend: binary WS commands, safety state, JPEG codec path |
| `354763d` | server.js: remove 40+ genesis events, keep gamepad relay only |
| `8d4b8b9` | TrustStrip safety badges, SimViewer VIDEO LOST overlay |
| `d400747` | NVENC H.264 encoder with JPEG fallback (PyAV) |
| `61b1ee3` | Add `node` Docker service for production Caddy routing |
| `ef6f89b` | Fix Alpine IPv6 healthcheck (localhost → 127.0.0.1) |

### Docker services (sim profile)

```bash
docker compose --profile sim up --build -d
```

| Service | Image | Status |
|---------|-------|--------|
| `webserver` | `caddy:2-alpine` | Healthy |
| `node` | `sdr_os-node` | Healthy |
| `transport-server` | `sdr_os-transport` | Healthy |
| `genesis-sim` | `sdr_os-cuda` | Healthy |
| `nats` | `nats:2-alpine` | Healthy |
| `ros-bridge` | `sdr_os-ros-jazzy` | Healthy |

### Caddy routing (production)

| Route | Upstream | Verified |
|-------|----------|----------|
| `/stream/ws` | `transport-server:8080` | Binary WS upgrade works |
| `/socket.io/*` | `node:3000` | Gamepad relay functional |
| `/api/*` | `node:3000` | `/api/status` returns JSON |
| `/ros/*` | `ros-bridge:9090` | rosbridge WS upgrade works |
| `/*` | `/srv/www` (static) | SPA loads from `dist/` |

### Safety stack verification

| Layer | Component | Behavior | Confirmed |
|-------|-----------|----------|-----------|
| 1 | Frontend (GenesisContext) | VIDEO LOST overlay after 500ms stale | Yes (UI) |
| 2 | Transport (Rust) | HOLD at 1s, ESTOP at 5s SHM stale | Yes (telemetry.safety.video_gate) |
| 3 | Sim (genesis_sim_runner) | HOLD at 200ms cmd TTL, ESTOP at 2s | Yes (telemetry.safety.state) |

### Tests added

| Test | Type | Standalone |
|------|------|-----------|
| `test_ws_frame_delivery.py` | Integration | No (needs transport-server) |
| `test_nats_command_roundtrip.py` | Integration | No (needs NATS + sim) |
| `test_safety_video_gate.py` | Integration | No (needs NATS + transport) |
| `test_cmd_ttl_decay.py` | Integration | No (needs NATS + sim) |
| `test_safety_state_authority.py` | Integration | No (needs NATS + sim) |

### CI updates

- Added cross-language SHM protocol test to `lint-and-unit` job
- Added `rust-unit` job (transport-server `cargo test` with caching)

### Issues fixed during Phase 2

| Issue | Root cause | Fix |
|-------|-----------|-----|
| Node healthcheck fails in Docker | Alpine `wget` resolves `localhost` to `::1` (IPv6), Node listens IPv4 only | Changed to `127.0.0.1` |
| Caddy can't route to node | No `node` service in compose | Created `containers/node/Dockerfile` + compose service |
| Caddy serves empty site | No `dist/` volume mount | Added `./dist:/srv/www:ro` to webserver |
| `nats-py` missing in genesis-sim | Package not in cuda dep group, backplane has no DNS | `docker exec` install; needs baking into image |
| Hardcoded `192.168.12.147` in webpack | Dev proxy pointed at specific robot IP | Changed to `localhost` with array-style proxy config |

---

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
