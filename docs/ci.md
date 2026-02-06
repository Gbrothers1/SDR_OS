# CI/CD

SDR_OS has 5 GitHub Actions workflows covering linting, docs, GPU smoke tests, backend compatibility, benchmarks, and performance regression gates.

## Workflows

### 1. CI (`ci.yml`)

Runs on every push/PR to `main` or `dev`.

| Job | Runner | What it does |
|-----|--------|-------------|
| `lint-and-unit` | `ubuntu-latest` | `uv sync --frozen` → `pytest -q tests/unit` |
| `docs-build` | `ubuntu-latest` | Install docs deps → `mkdocs build` |
| `cuda-smoke` | Self-hosted NVIDIA GPU | `docker compose --profile cuda run --rm app-cuda pytest -q tests/smoke` |
| `rocm-smoke` | Self-hosted ROCm GPU | `docker compose --profile rocm run --rm app-rocm pytest -q tests/smoke` |
| `mlx-smoke` | Self-hosted macOS ARM64 | `uv sync --frozen --group mlx` → `pytest -q tests/smoke` |

### 2. Docs Release (`docs-release.yml`)

Runs on version tags (`v*`). Publishes versioned docs via `mike deploy --push --update-aliases`.

### 3. Backend Compatibility Matrix (`compat-matrix.yml`)

Runs on PRs that touch `src/`, `containers/`, `requirements/`, `pyproject.toml`, or `uv.lock`. Tests three backends in parallel on `ubuntu-latest`:

| Backend | Dep group |
|---------|-----------|
| `cuda` | `--group cuda` |
| `rocm` | `--group rocm` |
| `mlx_cpu` | `--group mlx_cpu` |

Each installs deps and runs `pytest -q tests/integration` (if the directory exists).

### 4. GPU Benchmarks (`benchmarks.yml`)

Weekly (Monday 06:00 UTC) + manual dispatch. Runs per-backend benchmark scripts and uploads artifacts.

| Runner | Script | Artifacts |
|--------|--------|-----------|
| NVIDIA GPU | `tests/benchmarks/run_cuda.sh` | `benchmarks/results/cuda` |
| ROCm GPU | `tests/benchmarks/run_rocm.sh` | `benchmarks/results/rocm` |
| macOS ARM64 | `tests/benchmarks/run_mlx.sh` | `benchmarks/results/mlx` |

### 5. Perf Regression Gate (`perf-regression.yml`)

Manual dispatch only. Runs `tests/benchmarks/perf_gate_cuda.sh` on a self-hosted NVIDIA GPU runner. Intended to block releases if performance regresses beyond a threshold.

## Runner requirements

| Target | Labels | Requirements |
|--------|--------|-------------|
| CUDA | `self-hosted, linux, gpu, nvidia` | NVIDIA driver + `nvidia-container-toolkit`, Docker + Compose v2 |
| ROCm | `self-hosted, linux, gpu, rocm` | ROCm drivers, `/dev/kfd` accessible, Docker + Compose v2 |
| MLX | `self-hosted, macos, arm64, mlx` | macOS 14+ on Apple Silicon, Python 3.11, no Docker needed |

Full runner setup guide: [.github/RUNNERS.md](https://github.com/Gbrothers1/SDR_OS/blob/main/.github/RUNNERS.md).

## Test split

| Directory | Scope | GPU needed? | When |
|-----------|-------|-------------|------|
| `tests/unit/` | Fast CPU-only tests | No | Every push/PR |
| `tests/integration/` | Backend interface checks | Optional | PRs touching core code |
| `tests/smoke/` | Short sim/training loop | Yes (per target) | Every push to `main`/`dev` |
| `tests/benchmarks/` | Throughput/latency measurement | Yes | Weekly + manual |

## Where to find build logs

| What | Where |
|------|-------|
| CI runs (lint, docs, smoke) | [Actions → CI](https://github.com/Gbrothers1/SDR_OS/actions/workflows/ci.yml) |
| Docs release | [Actions → Docs Release](https://github.com/Gbrothers1/SDR_OS/actions/workflows/docs-release.yml) |
| Compat matrix | [Actions → Backend Compatibility Matrix](https://github.com/Gbrothers1/SDR_OS/actions/workflows/compat-matrix.yml) |
| Benchmarks | [Actions → GPU Benchmarks](https://github.com/Gbrothers1/SDR_OS/actions/workflows/benchmarks.yml) |
| Perf gate | [Actions → Perf Regression Gate](https://github.com/Gbrothers1/SDR_OS/actions/workflows/perf-regression.yml) |
| Local build | See [Build log](build.md) |

## Reproducibility

- **Python**: `uv.lock` is authoritative. All CI jobs use `uv sync --frozen`.
- **Node**: `package-lock.json` or `pnpm-lock.yaml` enforced in containers (`npm ci` or `pnpm install --frozen-lockfile`).
- **Containers**: Lockfiles copied first to maximize Docker layer cache stability.
