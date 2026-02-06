# SDR_OS Documentation

SDR_OS is a multi-backend robotics simulation and control platform for low-latency teleoperation, Genesis physics simulation, and reinforcement learning. It targets **CUDA** (NVIDIA), **ROCm** (AMD), and **MLX** (Apple Silicon) from a single source tree.

## What it does

- **Browser-based control** — Gamepad and keyboard input via a React UI, with real-time 3D visualization (Three.js) and telemetry dashboards.
- **Genesis simulation** — GPU-hosted physics simulation with NVENC video encoding and frame streaming to the browser.
- **ROS 2 integration** — Full ROS 2 Jazzy support for real robots: sensor pipelines, `/cmd_vel`, joint states, TF.
- **Multi-service backend** — Caddy reverse proxy, NATS message bus (JetStream), SHM ringbuffer for zero-copy video, Rust transport server (planned).
- **RL training pipeline** — Genesis-Forge + rsl_rl PPO, teleop recording, behavior cloning, policy evaluation.

## Tech stack

| Layer | Technology |
|-------|------------|
| Frontend | React 18, Three.js, Socket.io client, ROSLIB (CDN), WebCodecs H.264 decoder |
| Backend | Node.js / Express, Socket.io server |
| Simulation | Genesis 0.3.13, Genesis Forge 0.3.0, MuJoCo 3.4.0 |
| ML | PyTorch 2.10 (CUDA), rsl_rl, NumPy, OpenCV |
| Infrastructure | Docker Compose (profiles), NATS + JetStream, Caddy 2 |
| CI/CD | GitHub Actions (5 workflows), MkDocs + Material + mike |
| Package management | uv (Python), pnpm (Node.js) |

## Hardware reference

| Component | Spec |
|-----------|------|
| CPU | AMD Ryzen 5 2600 (6c/12t) |
| RAM | 8 GB |
| GPU | NVIDIA GeForce RTX 2080 Ti (11 GB VRAM) |
| NVENC | 2 concurrent sessions, H.264 + HEVC |
| OS | Ubuntu 22.04.5 LTS, kernel 6.8.0 |

## Quick navigation

| Page | What you'll find |
|------|-----------------|
| [Setup](setup.md) | Environment setup (uv, pnpm, Python, PyTorch, Genesis). |
| [Architecture](architecture.md) | System topology, pipelines, multi-service design, implementation phases. |
| [Frontend](frontend.md) | React components, contexts, styles, audio, entry points. |
| [Backend](backend.md) | Express server, Socket.io events, Genesis bridge protocol. |
| [Containers](containers.md) | Dockerfiles, Compose profiles, devcontainers. |
| [CI/CD](ci.md) | Workflows, runners, test split, benchmarks, perf gates. |
| [Build log](build.md) | Local build commands and build history. |
| [Plans](plans/2026-02-05-multi-service-backend-design.md) | Multi-service backend design and Phase 1 implementation plan. |

## Repository layout

```
SDR_OS/
├── src/client/           # React frontend (40+ components, 5 contexts)
├── server.js             # Express + Socket.io backend
├── containers/           # CUDA / ROCm / MLX Dockerfiles
├── configs/              # Robot YAMLs (planned: NATS, Caddy, Prometheus)
├── docs/                 # This MkDocs site
├── documents/            # Internal architecture and planning docs
├── tests/benchmarks/     # GPU benchmark scripts (CUDA, ROCm, MLX)
├── requirements/         # Per-backend pip requirements
├── .github/workflows/    # CI, benchmarks, compat-matrix, docs, perf
├── .devcontainer/        # VS Code devcontainer configs
├── ref/                  # Reference material and legacy code backup
├── _archive/             # Archived original codebase
├── pyproject.toml        # Python project (uv)
├── package.json          # Node project (pnpm)
├── docker-compose.yml    # Multi-profile Compose
├── webpack.config.js     # Webpack → dist/bundle.js
└── mkdocs.yml            # This documentation site config
```

## License

GPL-3.0. See [LICENSE](https://github.com/Gbrothers1/SDR_OS/blob/main/LICENSE).
