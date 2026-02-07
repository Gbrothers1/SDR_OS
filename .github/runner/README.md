# Self-Hosted GPU Runner

This directory contains setup and maintenance scripts for running a GitHub Actions
self-hosted runner on the SDR_OS development machine.

## Hardware

| Component | Spec |
|-----------|------|
| CPU | AMD Ryzen 5 2600 (6c/12t) |
| RAM | 8 GB |
| GPU | NVIDIA GeForce RTX 2080 Ti (11 GB VRAM) |
| Disk | 615 GB (311 GB free as of 2026-02-06) |
| OS | Ubuntu 22.04, kernel 6.8.0 |

## Quick Start

```bash
# 1. Get a registration token
TOKEN=$(gh api -X POST repos/Gbrothers1/SDR_OS/actions/runners/registration-token --jq .token)

# 2. Run setup (installs runner, registers, starts systemd service)
chmod +x setup.sh pre-run.sh post-run.sh
./setup.sh "$TOKEN"

# 3. Copy env template
cp env.template ~/actions-runner/.env
```

## Files

| File | Purpose |
|------|---------|
| `setup.sh` | Downloads runner, registers with GitHub, installs systemd service |
| `pre-run.sh` | Pre-job hook: checks GPU, Docker, disk, RAM before each CI job |
| `post-run.sh` | Post-job hook: prunes Docker images/containers after each job |
| `env.template` | Runner environment variables (copy to `~/actions-runner/.env`) |

## What Runs on This Runner

Jobs with `runs-on: [self-hosted, linux, gpu, nvidia]` labels:

- `cuda-smoke` — Build CUDA container, run NVENC validation, smoke tests
- Future: integration tests against `docker compose --profile sim` stack

## Resource Constraints

- **1 concurrent job** — 8 GB RAM is tight, serialized jobs prevent OOM
- **Docker cache** — ~56 GB build cache; `post-run.sh` prunes danglers
- **GPU sharing** — CI jobs and local `docker compose --profile sim` can coexist
  if only one uses NVENC at a time (2 session limit on RTX 2080 Ti)

## Maintenance

```bash
# Check runner status
sudo ~/actions-runner/svc.sh status

# View logs
journalctl -u actions.runner.*.service -f

# Stop runner (before heavy local GPU work)
sudo ~/actions-runner/svc.sh stop

# Restart runner
sudo ~/actions-runner/svc.sh start

# Reclaim Docker disk space
docker system prune -a    # WARNING: removes all unused images
docker builder prune       # Clear build cache only
```

## Coexistence with Local Development

The runner and local `docker compose --profile sim` can run simultaneously:

- **GPU**: RTX 2080 Ti has 11 GB VRAM. Genesis sim uses ~2-4 GB, CI builds use ~1 GB.
  Both fit if neither runs large training jobs.
- **CPU**: 12 threads. Pin sim to cores 2-4 (already in compose `cpuset`),
  leaving cores 0-1 and 5-11 for CI.
- **Disk**: 311 GB free is plenty for both. Enable `post-run.sh` if it drops below 50 GB.
- **RAM**: Tightest constraint. Stop the runner before running RL training.
