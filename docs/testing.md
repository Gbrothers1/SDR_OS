# Testing & Verification

SDR_OS has three ways to run the same verification checks — locally via script, via `just` task runner, or automatically in CI.

## Quick Reference

| Method | Command | Speed | When to use |
|--------|---------|-------|-------------|
| Script (CPU only) | `./scripts/verify.sh` | ~2 sec | Every time before committing |
| Script (+ Docker) | `./scripts/verify.sh --gpu` | ~2 min | After changing Dockerfiles or compose |
| Script (full) | `./scripts/verify.sh --all` | ~5 min | Before PRs, after GPU pipeline changes |
| Just (CPU only) | `just verify` | ~2 sec | Same as script, nicer interface |
| Just (+ Docker) | `just verify-gpu` | ~2 min | Same as script --gpu |
| Just (full) | `just verify-all` | ~5 min | Same as script --all |
| CI | Automatic on PR/push | ~10 min | Always runs, no manual action needed |

## What Gets Checked

### CPU Checks (always run, fast)

| Check | What it verifies | Command |
|-------|-----------------|---------|
| Compose valid | `docker-compose.yml` parses without errors | `docker compose config --quiet` |
| Unit tests | SHM ringbuffer protocol (17 tests) | `PYTHONPATH=src pytest tests/unit/ -v` |
| NVENC script | `validate_nvenc.py` has valid Python syntax | `python3 -c "import ast; ..."` |
| Package import | `sdr_os.ipc.shm_ringbuffer` is importable | `python3 -c "from sdr_os.ipc..."` |

### Docker Checks (--gpu flag, slower)

| Check | What it verifies | Command |
|-------|-----------------|---------|
| ROS2 Jazzy build | Container builds from `ros:jazzy-ros-base` | `docker compose --profile dev build ros-bridge` |
| ROS2 rosbridge | rosbridge_server package is installed | `ros2 pkg list \| grep rosbridge_server` |

### GPU Checks (--all flag, needs NVIDIA GPU)

| Check | What it verifies | Command |
|-------|-----------------|---------|
| CUDA build | CUDA container builds | `docker compose --profile cuda build app-cuda` |
| NVENC validation | GPU detected, NVENC encoders functional | `python3 scripts/validate_nvenc.py` |

## Running Tests Locally

### Prerequisites

- Python 3.10+ with `pytest` installed
- Docker + Docker Compose v2+
- (Optional) NVIDIA GPU with drivers for `--all` mode
- (Optional) [just](https://just.systems) task runner

### Run CPU-only checks

```bash
./scripts/verify.sh
```

Expected output:

```
========================================
  SDR_OS Verification
========================================

CPU checks:
  docker-compose.yml valid                PASS
  SHM ringbuffer tests                    PASS
  NVENC script syntax                     PASS
  Python package importable               PASS

Docker checks:
  ROS2 Jazzy build (use --gpu)            SKIP
  NVENC validation (use --all)            SKIP

========================================
  Results: 4 passed, 2 skipped / 6 total
========================================
```

### Run with Docker builds

```bash
./scripts/verify.sh --gpu
```

Expected output adds:

```
Docker checks:
  ROS2 Jazzy container builds             PASS
  ROS2 Jazzy rosbridge present            PASS
```

### Run everything (needs NVIDIA GPU)

```bash
./scripts/verify.sh --all
```

Expected output adds:

```
  CUDA container builds                   PASS
  NVENC validation (GPU)                  PASS
```

### NVENC validation output

When run standalone, `validate_nvenc.py` outputs JSON telemetry:

```bash
docker compose --profile cuda run --rm app-cuda python3 scripts/validate_nvenc.py
```

```json
{
  "cuda": {
    "cuda_available": true,
    "cuda_version": "12.4",
    "device_count": 1,
    "devices": [{
      "name": "NVIDIA GeForce RTX 2080 Ti",
      "total_memory_mb": 11264,
      "compute_capability": "7.5"
    }]
  },
  "nvenc_codecs": {
    "h264_nvenc": true,
    "hevc_nvenc": true
  },
  "nvenc_probe": {
    "functional": true,
    "method": "pyav"
  },
  "status": "NVENC_OK"
}
```

Exit codes: `0` = NVENC OK, `1` = CUDA available but NVENC not functional, `2` = no CUDA.

## Unit Test Details

### SHM Ringbuffer Tests (`tests/unit/test_shm_ringbuffer.py`)

| Test | What it verifies |
|------|-----------------|
| `test_pack_unpack_roundtrip` | 32-byte header serialization/deserialization |
| `test_unpack_too_short_raises` | Rejects truncated headers |
| `test_flags_bitmask` | KEYFRAME and WRAP_MARKER flags work correctly |
| `test_basic_write_read` | End-to-end write → read with CRC validation |
| `test_crc_validation` | Corrupted payload detected and rejected |
| `test_no_new_data_returns_none` | Reader returns None when no new frames |
| `test_multiple_writes_latest_wins` | "Latest wins" drop policy works |
| `test_oversized_payload_rejected` | Payloads exceeding buffer are rejected |
| `test_frame_seq_monotonic` | Frame sequence numbers increase monotonically |
| `test_codec_field` | H.264 and HEVC codec identifiers preserved |
| `test_sequential_frames_no_lap` | Normal sequential reads don't trigger lap detection |
| `test_gap_triggers_lap_and_keyframe_wait` | Skipped frames trigger resync to next keyframe |
| `test_non_keyframe_skipped_during_resync` | Non-keyframes ignored during resync |
| `test_empty_payload` | Zero-length payloads handled correctly |
| `test_max_payload_fits` | Maximum-size payloads fill buffer exactly |
| `test_writer_context_manager` | Context manager cleanup |
| `test_reader_context_manager` | Context manager cleanup |

## CI Troubleshooting Log

Issues encountered and fixed during Phase 1 CI setup:

### 1. `torch==2.0.1` has no Python 3.12 wheels

**Symptom:** `uv sync --frozen` fails with:
```
error: Distribution `torch==2.0.1` can't be installed because it doesn't have
a source distribution or wheel for the current platform

hint: You're using CPython 3.12 (cp312), but torch (v2.0.1) only has wheels
with the following Python ABI tags: cp310, cp311
```

**Cause:** `genesis-world` pins `torch==2.0.1`, which only published wheels for Python 3.10 and 3.11. CI jobs using Python 3.12 fail when `uv sync` tries to install the full project.

**Fix:** Jobs that don't need torch (`lint-and-unit`, `docs-build`) skip `uv sync` entirely and install only what they need via `pip`. The `compat-matrix` job pins `uv sync -p 3.11`.

### 2. `mkdocs build` fails on `repo_url: .`

**Symptom:** `Config value 'repo_url': The URL isn't valid, it should include the http:// (scheme)`

**Cause:** `mkdocs.yml` had placeholder `repo_url: .` instead of a full URL.

**Fix:** Set `repo_url: https://github.com/Gbrothers1/SDR_OS`.

### 3. `uv.lock` stale after `pyproject.toml` changes

**Symptom:** `uv sync --frozen --group <name>` fails with `Group 'X' is not defined`.

**Cause:** `uv.lock` was generated before dependency groups were added to `pyproject.toml`. The lock file didn't know about the new groups.

**Fix:** Regenerated `uv.lock` with `uv lock`. Keep the lock file in sync whenever you change `pyproject.toml`.

### 4. Docker Compose v5 rejects `device_requests`

**Symptom:** `docker compose config` fails with `additional properties 'device_requests' not allowed`.

**Cause:** Docker Compose v5 uses a different syntax for GPU access than the v3.x `device_requests` field.

**Fix:** Replace `device_requests` with `deploy.resources.reservations.devices`:
```yaml
# Wrong (v3.x style)
device_requests:
  - capabilities: [gpu]

# Correct (v5.x)
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

### 5. Bash `((var++))` fails with `set -e` when var=0

**Symptom:** Script exits immediately on first counter increment.

**Cause:** `((0))` evaluates to false in bash, returning exit code 1, which triggers `set -e`.

**Fix:** Use `var=$((var + 1))` instead of `((var++))`.

## Using the justfile

Install just: `curl -sSf https://just.systems/install.sh | bash`

```bash
just              # List all available recipes
just test         # Run unit tests
just verify       # CPU-only verification
just verify-gpu   # Include Docker builds
just verify-all   # Full check with GPU
just build-ros    # Build ROS2 container
just build-cuda   # Build CUDA container
just nvenc        # Run NVENC validation
just dev          # Start dev environment
just sim          # Start simulation environment
just docs         # Build documentation
```
