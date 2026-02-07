# Architecture

## Overview

SDR_OS connects a browser UI to either a real robot (via ROS 2) or a Genesis physics simulation (via a WebSocket bridge), with a Node.js gateway in the middle. The project is evolving from a monolithic setup toward a multi-service Docker Compose architecture.

```
Browser (React + Three.js + WebCodecs)
 ├── ROSLIB WebSocket ──→ rosbridge_server :9090 ──→ ROS 2 topics
 ├── Socket.io ──→ Node.js/Express :3000 ──→ gamepad relay (browser-only)
 └── Binary WS ──→ /stream/ws ──→ transport-server ──→ SHM + NATS ──→ Genesis sim (GPU)
```

## Dual-mode UI

The app switches between two modes via a toolbar selector:

| Mode | Visualization | Panels | Data path |
|------|---------------|--------|-----------|
| **Real Robot** | `RobotViewer` (Three.js 3D) | `TelemetryPanel`, `CameraViewer` | ROS 2 via rosbridge |
| **Genesis Sim** | `SimViewer` (H.264/JPEG stream) | `GenesisControlPanel`, `SimTelemetryPane`, `GenesisRobotSelector`, `MultiCameraViewer`, `SimulationStarter` | Binary WS → transport-server → NATS |

Both modes share: `ControlOverlay` (gamepad), `LogViewer`, `SettingsModal`, `SafetyIndicator`, `TrustStrip`.

## Pipelines

### Control

```
Browser gamepad → ControlOverlay (every animationFrame)
 → Socket.io: controller_button_states, controller_joystick_state
 → server.js broadcasts to all browser clients (gamepad relay only)
 → ROS: ROSLIB publishes to /cmd_vel

Genesis velocity commands:
 → Browser sends 0x04 COMMAND over WS → transport-server
 → NATS command.genesis.set_cmd_vel → genesis-sim
```

### Telemetry

```
Sensors / IIO / IMU / GPS → ROS 2 topics
 → rosbridge → Browser TelemetryPanel charts

Genesis sim → NATS telemetry.* subjects
 → transport-server → 0x02 WS → Browser TrainingDashboard
```

### Video

```
Genesis GPU render → NVENC H.264 (or JPEG fallback)
 → SHM ringbuffer (/dev/shm/sdr_os_ipc/frames)
 → Rust transport-server reads SHM
 → 0x01 WS fanout to browser clients
 → H.264: WebCodecs VideoDecoder → canvas
 → JPEG: Blob → objectURL → <img>
```

### Binary WebSocket Protocol

Type byte prefix on every WS binary frame:

| Byte | Type | Direction | Payload |
|------|------|-----------|---------|
| `0x01` | VIDEO | server→browser | 32-byte LE header + Annex-B/JPEG |
| `0x02` | TELEMETRY | server→browser | NATS subject (null-terminated) + JSON |
| `0x03` | SIGNALING | reserved | WebRTC signaling |
| `0x04` | COMMAND | browser→server | JSON `{action, cmd_seq, data, ttl_ms?}` |

See `docs/nats-subjects.md` for the full subject schema.

### Safety Stack (3 Layers)

| Layer | Location | HOLD | ESTOP | Authority |
|-------|----------|------|-------|-----------|
| 1 | Frontend (GenesisContext) | — | 500ms video age | Cosmetic (VIDEO LOST overlay) |
| 2 | Transport (Rust) | 1s SHM stale | 5s SHM stale | Gate + latch zero velocity |
| 3 | Sim (genesis_sim_runner) | 200ms cmd TTL | 2s cmd TTL | Canonical state authority |

- **ARMED** → normal operation
- **HOLD** → zero velocity, auto-recoverable on fresh command
- **ESTOP** → zero velocity, requires operator RE-ARM via TrustStrip button

### Training

```
Genesis env wrapper (genesis_vecenv.py) ↔ rsl_rl PPO
 → teleop recording, behavior cloning, policy evaluation
 → checkpoints, episode replay, live policy rollout
ActionRouter blends: applied = alpha * policy + (1-alpha) * teleop
```

## ROS 2 topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/cmd_vel` | geometry_msgs/Twist | Publish |
| `/robot/imu` | sensor_msgs/Imu | Subscribe |
| `/joint_states` | sensor_msgs/JointState | Subscribe |
| `/tf`, `/tf_static` | tf2_msgs/TFMessage | Subscribe |
| `/odom` | nav_msgs/Odometry | Subscribe |
| `/webcam/image_raw` | sensor_msgs/Image | Subscribe |
| `/controller/joystick_state` | — | Publish |
| `/controller/button_states` | — | Publish |

## Backend options (decision record)

Three options were evaluated. See [ARCHITECTURE_REASONING.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/ARCHITECTURE_REASONING.md) for full analysis.

| Option | Stack | Verdict |
|--------|-------|---------|
| **A: Hybrid** | Python ROS 2 + Node gateway | **Chosen** — matches existing code, minimal refactor risk |
| B: Python-only | FastAPI + rclpy + aiortc | Single language but heavier for media |
| C: Node-dominant | ts-node/Nest + rclnodejs | JS GC jitter risk in ROS loop |

## Multi-service backend

The [Multi-Service Backend Design](plans/2026-02-05-multi-service-backend-design.md) defines the target architecture:

```
┌────────────────────────────────────────────────────────────────┐
│                        DOCKER COMPOSE                          │
│                                                                │
│  ═══════════ EDGE NETWORK (external) ═══════════              │
│                     │                                          │
│              ┌──────┴──────┐                                   │
│              │   Caddy     │  :443 (Tailscale TLS)             │
│              └──────┬──────┘                                   │
│        ┌────────────┼────────────┐                             │
│        ▼            ▼            ▼                              │
│  transport    ros-bridge      web app                          │
│   (Rust)      (Python)       (static)                          │
│        │            │                                          │
│  ═══ shm+uds ═     │                                          │
│        │     ┌──────┴──────┐                                   │
│  ═══════ BACKPLANE NETWORK (internal) ═══════                  │
│        │     │    NATS     │                                   │
│        │     │ + JetStream │                                   │
│        │     └──────┬──────┘                                   │
│        ▼            ▼            ▼                              │
│  genesis-sim   ros-bridge   training-runner                    │
│   (GPU)        (backplane)    (Python)                         │
│                                                                │
│  Shared: sdr_ipc (tmpfs) at /dev/shm/sdr_os_ipc               │
└────────────────────────────────────────────────────────────────┘
```

### Services

| Service | Language | Role | Network |
|---------|----------|------|---------|
| Caddy (webserver) | Go | Reverse proxy, static files, Tailscale TLS | edge + backplane |
| node | Node.js | Socket.io gamepad relay, /api routes | backplane |
| transport-server | Rust | SHM→WS fanout, 0x04→NATS, video gate | edge + backplane |
| genesis-sim | Python | Genesis simulation, NVENC/JPEG encoding, safety L3 | host network |
| ros-bridge | Python | ROS 2 ↔ rosbridge WebSocket | host network (DDS) |
| training-runner | Python | genesis-forge RL, rsl_rl PPO | backplane |
| NATS | NATS | Message bus (command + telemetry) | backplane |

### Docker Compose Profiles

| Profile | Services | Use |
|---------|----------|-----|
| `dev` | webserver, ros-bridge | Development |
| `sim` | webserver (Caddy), node, transport-server, genesis-sim, nats, ros-bridge | Full simulation stack |
| `train` | training-runner, nats | RL training |
| `obs` | prometheus, grafana | Observability |
| `cuda` | app-cuda | CUDA CI target |
| `rocm` | app-rocm | ROCm CI target |
| `mlx` | app-mlx | MLX CI target |

### Data plane

| Data type | Transport | Rationale |
|-----------|-----------|-----------|
| Video frames (encoded) | SHM ringbuffer | Zero-copy, ~50–500 KB/frame |
| Events / lifecycle | NATS (plain) | Simple pub/sub |
| Training data | NATS JetStream | Durability, replay |
| Control commands | WebRTC DataChannel | Lowest latency |

### SHM Ringbuffer (Phase 1)

Zero-copy frame transport between genesis-sim and transport-server:

- **Layout**: 16-byte control region + frame data
- **Frame header**: 32 bytes (frame_id, frame_seq, size, flags, codec, crc32)
- **Atomic commit**: sequence number written last as commit marker
- **Lap detection**: reader tracks frame_seq, resyncs to IDR on gaps
- **Drop policy**: "latest wins" — writer always overwrites with freshest frame

### NVENC Video Pipeline

```
Genesis Render (CUDA) → NVENC encoder → SHM ringbuffer → transport-server → clients
```

- H.264/HEVC encoding on RTX 2080 Ti
- 2 concurrent NVENC sessions max
- Target: 1080p60 @ 6-12 Mbps CBR

### Containers

| Container | Base Image | Purpose |
|-----------|-----------|---------|
| `sdr_os-cuda` | `nvidia/cuda:12.4.1-runtime-ubuntu22.04` | CUDA CI + sim |
| `sdr_os-node` | `node:20-alpine` | Socket.io gamepad relay + /api |
| `sdr_os-transport` | Custom Rust | SHM→WS fanout, NATS relay |
| `sdr_os-rocm` | `rocm/rocm-terminal:6.1.2` | ROCm CI |
| `sdr_os-mlx` | `python:3.11-slim` | MLX Linux parity |
| `sdr_os-ros-jazzy` | `ros:jazzy-ros-base` | ROS2 Jazzy + rosbridge |

## Implementation phases

| Phase | Scope | Status |
|-------|-------|--------|
| **1** | CUDA Docker + basic pipeline: NATS, Caddy, ROS 2 Jazzy container, SHM ringbuffer, pytest, justfile, NVENC validation, docs. | **Complete** |
| **2** | NATS backbone, safety stack, transport integration, Caddy routing, node service, docs alignment. | **Complete** |
| **3** | Production hardening: healthchecks, host tuning, Prometheus/Grafana, CI. | Planned |
| **4** | WebRTC + control path: signaling, DataChannel, H.264 RTP, WebCodecs. | Planned |
| **5** | Training integration: JetStream streams, genesis-forge, episode recording. | Planned |
| **6** | Distribution (future): NATS leafnodes, multi-node, capability roles. | Planned |

See [Phase 1 Implementation Plan](plans/2026-02-05-phase1-cuda-docker-basic-pipeline.md) and [NATS Safety Transport Plan](plans/2026-02-06-nats-safety-transport-plan.md) for task breakdowns.

## Key design decisions

- **roslib loaded from CDN** in `index.html`, not bundled — marked as external in webpack.
- **Gamepad polling** every animation frame in `ControlOverlay`; state published to both ROS and Socket.io.
- **Blob URL management**: `GenesisContext` tracks Blob URLs and revokes old ones to prevent memory leaks.
- **Safety**: Deadman switch (hold L1), velocity/accel/jerk clamps, confidence gating, auto-stop on anomaly.
- **Action blending**: `applied = alpha * policy + (1-alpha) * teleop` with alpha ramp-limited for smooth transitions.
- **Sound effects** synthesized at runtime using Web Audio API oscillators (no audio files).
- **Shared memory / UDS** preferred when sim and bridge are co-located to eliminate memcpy overhead.

## References

- [ARCHITECTURE_OVERVIEW.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/ARCHITECTURE_OVERVIEW.md) — Original architecture (Genesis-in-browser goal).
- [ARCHITECTURE_REASONING.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/ARCHITECTURE_REASONING.md) — Backend tradeoffs and decision record.
- [NEW_ARCHITECTURE_OVERVIEW.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/NEW_ARCHITECTURE_OVERVIEW.md) — Multi-target dev and CI/CD plan.
- [TECH_SPEC.md](https://github.com/Gbrothers1/SDR_OS/blob/main/documents/TECH_SPEC.md) — Hardware and software versions.
- [Multi-Service Backend Design](plans/2026-02-05-multi-service-backend-design.md) — Full design doc.
- [Phase 1 Solutions](solutions/phase1-architecture.md) — Architecture diagrams and memory layouts.
