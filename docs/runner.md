# Self-Hosted GPU Runner

SDR_OS CI requires a self-hosted runner for GPU-dependent jobs (CUDA smoke tests, NVENC validation, future integration tests). The development machine doubles as the runner.

## Hardware

| Component | Spec | CI Headroom |
|-----------|------|-------------|
| CPU | AMD Ryzen 5 2600 (6c/12t) | Cores 0-1, 5-11 free (sim pinned to 2-4) |
| RAM | 8 GB | ~4 GB available after OS + services |
| GPU | RTX 2080 Ti (11 GB VRAM) | ~8 GB free during typical dev work |
| Disk | 615 GB | 311 GB free |
| NVENC | 2 concurrent sessions | 1 for sim, 1 for CI validation |

## Setup

```bash
cd .github/runner

# Get a registration token (requires gh CLI authenticated)
TOKEN=$(gh api -X POST repos/Gbrothers1/SDR_OS/actions/runners/registration-token --jq .token)

# Install, register, and start as systemd service
./setup.sh "$TOKEN"

# Copy environment config
cp env.template ~/actions-runner/.env
```

## What runs on this runner

Jobs tagged with `runs-on: [self-hosted, linux, gpu, nvidia]`:

| Job | Workflow | What It Does |
|-----|----------|--------------|
| `cuda-smoke` | `ci.yml` | Build CUDA container, NVENC validation, smoke tests |
| `cuda` | `compat-matrix.yml` | Integration tests in CUDA container |
| Future | — | `docker compose --profile sim` integration suite |

## Pre-job health check

The `pre-run.sh` hook runs before every job and verifies:

- GPU accessible via `nvidia-smi`
- Docker daemon responding
- At least 20 GB disk free
- Warns if RAM drops below 1 GB

## Resource constraints

| Resource | Limit | Reason |
|----------|-------|--------|
| Concurrent jobs | 1 | 8 GB RAM is tight |
| Docker build cache | ~56 GB | Prune with `docker builder prune` |
| GPU sharing | 2 NVENC sessions | RTX 2080 Ti hardware limit |

## Coexistence with local dev

The runner and `docker compose --profile sim` can run simultaneously:

- **GPU**: Sim uses ~2-4 GB VRAM, CI builds use ~1 GB — both fit
- **CPU**: Sim pinned to cores 2-4 via `cpuset`, CI uses remaining cores
- **Disk**: 311 GB free is plenty
- **RAM**: Tightest constraint — stop runner before RL training (`sudo ~/actions-runner/svc.sh stop`)

## Maintenance

```bash
# Status
sudo ~/actions-runner/svc.sh status

# Logs
journalctl -u actions.runner.*.service -f

# Stop/start
sudo ~/actions-runner/svc.sh stop
sudo ~/actions-runner/svc.sh start

# Reclaim space
docker builder prune        # Build cache only
docker system prune -a      # All unused images (nuclear)
```

## CI jobs that run on this runner

See [CI/CD](ci.md) for the full workflow breakdown. The `cuda-smoke` and `compat-matrix` GPU jobs require this runner to be online.
