# GitHub Actions Guide

## What Is GitHub Actions?

GitHub Actions is CI/CD (Continuous Integration / Continuous Deployment) built into GitHub. When you push code or open a pull request, GitHub automatically runs your tests on their servers. If tests fail, the PR gets a red X. If they pass, green checkmark.

**The key idea:** You never have to remember to run tests. GitHub runs them for you on every push.

### How It Works

```
You push code → GitHub reads .github/workflows/*.yml → GitHub spins up a VM → Runs your commands → Reports pass/fail
```

1. **Workflow files** live in `.github/workflows/`. Each `.yml` file defines a pipeline.
2. **Triggers** define when the workflow runs (on push, on PR, on schedule, manually).
3. **Jobs** are groups of steps that run on a specific machine (called a "runner").
4. **Steps** are individual commands (checkout code, install deps, run tests).
5. **Runners** are the machines that execute jobs. GitHub provides free ones (`ubuntu-latest`), or you can self-host your own (needed for GPU tests).

### The Workflow File

Our CI lives at `.github/workflows/ci.yml`. Here's how to read it:

```yaml
name: CI                    # Name shown in GitHub UI

on:                         # WHEN does this run?
  pull_request:             #   On every PR
  push:
    branches: [main, dev]   #   On pushes to main or dev

jobs:                       # WHAT does it run?
  validate-compose:         #   Job name (shown in GitHub UI)
    runs-on: ubuntu-latest  #   Which machine (GitHub-hosted Ubuntu)
    steps:                  #   Commands to execute, in order
      - uses: actions/checkout@v4    # Step 1: Download your code
      - name: Validate compose       # Step 2: Run a command
        run: docker compose config --quiet
```

## Our CI Pipeline

### Jobs That Run on Every PR

These run on free GitHub-hosted runners. No setup needed — they work automatically once the workflow file exists.

```
┌─────────────────────┐   ┌─────────────────────┐
│  validate-compose   │   │   lint-and-unit      │
│  (ubuntu-latest)    │   │   (ubuntu-latest)    │
│                     │   │                      │
│  Checks compose     │   │  Python 3.12         │
│  file is valid      │   │  pytest tests/unit/  │
└─────────────────────┘   └──────────────────────┘

┌─────────────────────┐   ┌─────────────────────┐
│  ros-jazzy-build    │   │   docs-build         │
│  (ubuntu-latest)    │   │   (ubuntu-latest)    │
│                     │   │                      │
│  Builds ROS2 Jazzy  │   │  mkdocs build        │
│  container, checks  │   │  (catches broken     │
│  rosbridge works    │   │   docs/links)        │
└─────────────────────┘   └──────────────────────┘
```

**Cost:** Free. GitHub gives 2,000 minutes/month on the free plan.

### Jobs That Need Self-Hosted Runners

GPU tests can't run on GitHub's machines (no GPUs). These require **self-hosted runners** — your own machine registered with GitHub.

```
┌─────────────────────┐   ┌─────────────────────┐   ┌─────────────────────┐
│    cuda-smoke       │   │    rocm-smoke        │   │    mlx-smoke         │
│  (self-hosted,      │   │  (self-hosted,       │   │  (self-hosted,       │
│   linux, gpu,       │   │   linux, gpu,        │   │   macos, arm64,      │
│   nvidia)           │   │   rocm)              │   │   mlx)               │
│                     │   │                      │   │                      │
│  NVENC validation   │   │  ROCm smoke tests    │   │  MLX smoke tests     │
│  CUDA smoke tests   │   │                      │   │                      │
└─────────────────────┘   └──────────────────────┘   └──────────────────────┘
```

**These jobs will be skipped** until you set up self-hosted runners. The CPU jobs still run fine.

## Setting Up (Step by Step)

### Step 1: Push the Workflow File

That's it for the CPU jobs. Once `.github/workflows/ci.yml` is in your repo on GitHub, Actions runs automatically. You can see results at:

```
https://github.com/<your-org>/SDR_OS/actions
```

### Step 2: Check Results on PRs

When you open a PR, you'll see at the bottom:

```
✅ validate-compose — passed
✅ lint-and-unit — passed
✅ ros-jazzy-build — passed
✅ docs-build — passed
⏭️ cuda-smoke — skipped (no matching runner)
⏭️ rocm-smoke — skipped (no matching runner)
⏭️ mlx-smoke — skipped (no matching runner)
```

The GPU jobs show "skipped" because there's no self-hosted runner yet. That's fine.

### Step 3: Setting Up a Self-Hosted Runner (Optional, for GPU)

To run GPU tests, you register your RTX 2080 Ti machine as a runner:

1. Go to your repo → **Settings** → **Actions** → **Runners** → **New self-hosted runner**
2. GitHub gives you a download link and token. Run on your machine:

```bash
# Download (GitHub will show the exact URL)
mkdir actions-runner && cd actions-runner
curl -o actions-runner.tar.gz -L https://github.com/actions/runner/releases/download/v2.XXX/...
tar xzf actions-runner.tar.gz

# Configure (GitHub will show the exact token)
./config.sh --url https://github.com/<org>/SDR_OS --token <TOKEN> \
  --labels self-hosted,linux,gpu,nvidia

# Run
./run.sh  # or install as systemd service
```

3. After that, the `cuda-smoke` job will pick up your machine and run GPU tests automatically.

!!! note
    Self-hosted runners are optional. The CPU-based jobs (compose validation, unit tests, docs build, ROS2 container build) all work on GitHub's free runners immediately.

## Workflow Files Reference

| File | Trigger | What it does |
|------|---------|-------------|
| `ci.yml` | Every PR + push to main/dev | Core tests: compose, unit tests, ROS2, docs |
| `benchmarks.yml` | Weekly + manual | GPU benchmarks on all three backends |
| `compat-matrix.yml` | PRs touching `src/` or `containers/` | CPU-only tests across cuda/rocm/mlx_cpu deps |
| `docs-release.yml` | Git tags (`v*`) | Publish versioned docs via `mike` |
| `perf-regression.yml` | Manual only | Compare performance against baseline |

## How to Trigger Manually

Some workflows have `workflow_dispatch` which means you can trigger them from the GitHub UI:

1. Go to **Actions** tab
2. Select the workflow (e.g., "GPU Benchmarks")
3. Click **Run workflow**
4. Select branch and click **Run**

## Reading CI Results

### In the GitHub UI

- Go to **Actions** tab → click a workflow run
- Each job shows as a box. Click to expand and see step-by-step logs
- Failed steps show the error output inline

### On Pull Requests

- Scroll to the bottom of the PR page
- Green checkmark = passed, Red X = failed
- Click "Details" next to any check to see logs

### Locally (Same Commands)

```bash
# Run the exact same checks CI runs
./scripts/verify.sh       # CPU checks
./scripts/verify.sh --gpu # + Docker builds
./scripts/verify.sh --all # + GPU checks
```

## Common Issues

| Problem | Cause | Fix |
|---------|-------|-----|
| GPU jobs skipped | No self-hosted runner | Set up a runner (Step 3 above) |
| `ros-jazzy-build` slow | First build downloads base image | Cached after first run (~2 min) |
| `docs-build` fails | Missing mkdocs deps | Check `pyproject.toml` `[dependency-groups] docs` |
| Compose validation fails | YAML syntax error | Run `docker compose config` locally to see the error |
