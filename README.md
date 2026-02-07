<div align="center">

# SDR_OS

**Multi-backend robotics simulation and control platform**

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg?logo=python&logoColor=white)](https://python.org)
[![Rust](https://img.shields.io/badge/Rust-stable-DEA584.svg?logo=rust&logoColor=white)](https://www.rust-lang.org)
[![React 18](https://img.shields.io/badge/React-18-61DAFB.svg?logo=react&logoColor=white)](https://react.dev)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-22314E.svg?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/Docker-Compose-2496ED.svg?logo=docker&logoColor=white)](https://docs.docker.com/compose/)

Low-latency teleoperation, GPU-accelerated physics simulation, and reinforcement learning training &mdash; all from a single source tree targeting **CUDA**, **ROCm**, and **MLX**.

<!-- Add a hero screenshot or demo GIF here:
![SDR_OS Demo](assets/images/hero.png)
-->

[Documentation](https://gbrothers1.github.io/SDR_OS/) &middot; [Architecture](#architecture) &middot; [Quick Start](#quick-start) &middot; [Contributing](#contributing)

</div>

---

## What is SDR_OS?

SDR_OS connects a browser-based control interface to either real robots (via ROS 2) or a Genesis physics simulation (via a GPU-accelerated video pipeline), with a multi-service backend orchestrated through Docker Compose.

It was built for the [Steam Deck](https://store.steampowered.com/steamdeck) form factor but runs on any modern browser.

### Key Capabilities

- **Browser-based teleoperation** &mdash; Gamepad and keyboard control with real-time 3D visualization (Three.js), telemetry dashboards, and H.264 video streaming via WebCodecs
- **Genesis physics simulation** &mdash; GPU-hosted simulation with NVENC hardware encoding and zero-copy SHM frame transport
- **ROS 2 integration** &mdash; Full ROS 2 Jazzy support for real robots: sensor pipelines, `/cmd_vel`, joint states, transforms
- **3-layer safety stack** &mdash; Frontend, transport, and sim-level safety with HOLD/ESTOP modes and deadman switch
- **RL training pipeline** &mdash; Genesis-Forge + rsl_rl PPO with teleop recording, behavior cloning, and live policy blending
- **Multi-GPU backend support** &mdash; NVIDIA CUDA, AMD ROCm, and Apple Silicon MLX from one codebase

## Architecture

<!-- Add architecture diagram here:
![Architecture Diagram](assets/images/architecture.png)
-->

```
Browser (React + Three.js + WebCodecs)
 ├── ROSLIB WebSocket ──→ rosbridge_server :9090 ──→ ROS 2 topics
 ├── Socket.io ──→ Node.js/Express :3000 ──→ gamepad relay
 └── Binary WS ──→ /stream/ws ──→ Rust transport-server ──→ SHM + NATS ──→ Genesis sim (GPU)
```

### Multi-Service Backend

```
┌──────────────────────────────────────────────────────────────┐
│                      DOCKER COMPOSE                          │
│                                                              │
│  ══════════ EDGE NETWORK (external) ══════════              │
│                    │                                         │
│             ┌──────┴──────┐                                  │
│             │   Caddy     │  :443 (TLS) / :80                │
│             └──────┬──────┘                                  │
│       ┌────────────┼────────────┐                            │
│       ▼            ▼            ▼                             │
│  transport    ros-bridge      static                         │
│   (Rust)      (Python)       assets                          │
│       │            │                                         │
│  ═══ shm ═══      │                                         │
│       │     ┌──────┴──────┐                                  │
│  ══════ BACKPLANE NETWORK (internal) ══════                  │
│       │     │    NATS     │                                  │
│       │     │ + JetStream │                                  │
│       │     └──────┬──────┘                                  │
│       ▼            ▼            ▼                             │
│  genesis-sim   ros-bridge   training-runner                  │
│   (GPU)        (backplane)    (Python)                       │
│                                                              │
│  Shared: /dev/shm/sdr_os_ipc (zero-copy frame transport)    │
└──────────────────────────────────────────────────────────────┘
```

| Service | Language | Role |
|---------|----------|------|
| **Caddy** | Go | Reverse proxy, static files, TLS termination |
| **Node** | Node.js | Socket.io gamepad relay, `/api` routes |
| **Transport** | Rust | SHM &rarr; WebSocket fanout, NATS relay, video gate |
| **Genesis Sim** | Python | Physics simulation, NVENC/JPEG encoding, safety layer 3 |
| **ROS Bridge** | Python | ROS 2 &harr; rosbridge WebSocket |
| **NATS** | NATS | Message bus for commands and telemetry |

### Safety Stack

| Layer | Location | HOLD Trigger | ESTOP Trigger | Authority |
|-------|----------|-------------|--------------|-----------|
| 1 | Frontend | &mdash; | 500ms video age | UI overlay |
| 2 | Transport (Rust) | 1s SHM stale | 5s SHM stale | Gate + zero velocity |
| 3 | Sim (Python) | 200ms cmd TTL | 2s cmd TTL | Canonical state |

### Binary WebSocket Protocol

| Byte | Type | Direction | Payload |
|------|------|-----------|---------|
| `0x01` | VIDEO | server &rarr; browser | 32-byte LE header + Annex-B/JPEG |
| `0x02` | TELEMETRY | server &rarr; browser | NATS subject (null-terminated) + JSON |
| `0x03` | SIGNALING | reserved | WebRTC signaling |
| `0x04` | COMMAND | browser &rarr; server | JSON `{action, cmd_seq, data, ttl_ms?}` |

## Tech Stack

| Layer | Technology |
|-------|------------|
| **Frontend** | React 18, Three.js, Socket.io, ROSLIB, WebCodecs H.264 |
| **Backend** | Node.js / Express, Rust transport-server |
| **Simulation** | Genesis 0.3.13, Genesis Forge 0.3.0, MuJoCo 3.4 |
| **ML** | PyTorch 2.8+ (CUDA/ROCm/MLX), rsl_rl, NumPy |
| **Infra** | Docker Compose, NATS + JetStream, Caddy 2 |
| **CI/CD** | GitHub Actions, MkDocs Material, mike |
| **Packages** | uv (Python), pnpm (Node.js) |

## Quick Start

### Prerequisites

- Python &ge; 3.10, < 3.14
- [uv](https://docs.astral.sh/uv/) (Python package manager)
- [pnpm](https://pnpm.io/) (Node.js package manager)
- Docker + Docker Compose (for containerized services)

### Install

```bash
git clone https://github.com/Gbrothers1/SDR_OS.git
cd SDR_OS

# Python dependencies
uv sync

# Node dependencies (web UI)
pnpm install

# Verify Genesis installation
uv run python -c "import genesis; print(genesis.__version__)"
```

### Run

```bash
# Full simulation stack (Docker)
npm run build
docker compose --profile sim up --build -d

# Development mode (bare metal)
npm run build
node server.js  # http://localhost:3000
```

### GPU Backends

| Profile | Target | Command |
|---------|--------|---------|
| `sim` | Full sim stack | `docker compose --profile sim up` |
| `dev` | Web + ROS only | `docker compose --profile dev up` |
| `cuda` | NVIDIA GPU CI | `docker compose --profile cuda run --rm app-cuda` |
| `rocm` | AMD GPU CI | `docker compose --profile rocm run --rm app-rocm` |
| `mlx` | Apple Silicon CI | `docker compose --profile mlx run --rm app-mlx` |

See [`docs/setup.md`](docs/setup.md) for full environment setup (PyTorch, Genesis, CUDA drivers, etc.).

## Repository Layout

```
SDR_OS/
├── src/client/                    # React frontend (40+ components, 5 contexts)
├── src/sdr_os/                    # Python backend (SHM ringbuffer, IPC)
├── server.js                      # Express + Socket.io (gamepad relay)
├── services/transport-server/     # Rust: SHM→WS fanout, NATS relay, video gate
├── containers/                    # CUDA / ROCm / MLX / ROS2 Jazzy Dockerfiles
├── configs/                       # Caddyfile, nats.conf, prometheus.yml
├── scripts/                       # verify.sh, validate_nvenc.py, ROS2 launcher
├── tests/                         # unit / integration / benchmarks
├── docs/                          # MkDocs documentation site
├── assets/images/                 # README and documentation images
├── docker-compose.yml             # Multi-profile Compose (12 services, 7 profiles)
├── pyproject.toml                 # Python project config (uv)
├── package.json                   # Node project config (pnpm)
└── mkdocs.yml                     # Documentation site config
```

## Documentation

Full documentation is built with [MkDocs Material](https://squidfunk.github.io/mkdocs-material/) and published via GitHub Pages.

```bash
# Serve docs locally
uv sync --group docs
uv run mkdocs serve
```

Key internal docs:

- [`documents/ARCHITECTURE_OVERVIEW.md`](documents/ARCHITECTURE_OVERVIEW.md) &mdash; Original architecture and Genesis-in-browser design
- [`documents/NEW_ARCHITECTURE_OVERVIEW.md`](documents/NEW_ARCHITECTURE_OVERVIEW.md) &mdash; Multi-target dev and CI/CD plan
- [`docs/architecture.md`](docs/architecture.md) &mdash; Current system topology and pipelines

## Roadmap

| Phase | Scope | Status |
|-------|-------|--------|
| **1** | CUDA Docker, basic pipeline, SHM ringbuffer, NVENC validation | Done |
| **2** | NATS backbone, 3-layer safety stack, transport integration, Caddy routing | Done |
| **3** | Production hardening: healthchecks, host tuning, Prometheus/Grafana, CI | Planned |
| **4** | WebRTC + control path: signaling, DataChannel, H.264 RTP | Planned |
| **5** | Training integration: JetStream streams, genesis-forge, episode recording | Planned |
| **6** | Distribution: NATS leafnodes, multi-node, capability roles | Planned |

## Contributing

Contributions are welcome. Please open an issue first to discuss what you'd like to change.

```bash
# Run tests
PYTHONPATH=src pytest tests/unit/

# Rust transport tests
cargo test --manifest-path services/transport-server/Cargo.toml

# Build frontend
npm run build
```

## Support the Project

If you find SDR_OS useful, consider supporting development:

**Bitcoin:**

<img src="assets/images/bitcoin-qr.png" alt="Bitcoin QR" width="200">

```
35gdRMt37KgAXg4c2kC14j6aadQLYPwmjU
```

**Lightning Network:**

<img src="assets/images/lightning-qr.png" alt="Lightning Network QR" width="200">

<details>
<summary>Lightning invoice (click to expand)</summary>

```
lnbc1p5cdd4pdqdgdshx6pqg9c8qpp50xfw72q99cclm6c7f5wxgz6r6v60w9r7m6ee4ltn2uf0wfpu0yeqsp5478wft23waxsndap2p7lf6wwkpezt3ta4w7aszdctnaqnum9jtxs9qrsgqcqpcxqy8ayqrzjqfzhphca8jlc5zznw52mnqxsnymltjgg3lxe4ul82g42vw0jpkgkwrvdz5qqy9cqqgqqqqqqqqqqqqqq9grzjqv06k0m23t593pngl0jt7n9wznp64fqngvctz7vts8nq4tukvtljqztad5qqrdqqqcqqqqqqqqqqqqqq9g52gaxzm3syccas52z0jdfxwwq7lmymq5tz9pt95wt94lhmvh65ny34zp8t44wn55pu8xqwp3gy399035jw5cupngka7ff3fzd2ln0espg5frpy
```

</details>

## License

[GNU General Public License v3.0](LICENSE)
