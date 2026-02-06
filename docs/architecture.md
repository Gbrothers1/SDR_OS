# Architecture

## Overview

SDR_OS connects a browser UI to either a real robot (via ROS 2) or a Genesis physics simulation (via a WebSocket bridge), with a Node.js gateway in the middle. The project is evolving from a monolithic setup toward a multi-service Docker Compose architecture.

```
Browser (React + Three.js + WebCodecs)
 ├── ROSLIB WebSocket ──→ rosbridge_server :9090 ──→ ROS 2 topics
 ├── Socket.io ──→ Node.js/Express :3000 ──→ multi-client sync
 └── GenesisContext WS ──→ Genesis bridge :9091 ──→ Genesis sim (GPU)
```

## Dual-mode UI

The app switches between two modes via a toolbar selector:

| Mode | Visualization | Panels | Data path |
|------|---------------|--------|-----------|
| **Real Robot** | `RobotViewer` (Three.js 3D) | `TelemetryPanel`, `CameraViewer` | ROS 2 via rosbridge |
| **Genesis Sim** | `SimViewer` (JPEG stream) | `GenesisControlPanel`, `SimTelemetryPane`, `GenesisRobotSelector`, `MultiCameraViewer`, `SimulationStarter` | Socket.io → Genesis bridge |

Both modes share: `ControlOverlay` (gamepad), `LogViewer`, `SettingsModal`, `SafetyIndicator`.

## Pipelines

### Control

```
Browser gamepad → ControlOverlay (every animationFrame)
 → Socket.io: controller_button_states, controller_joystick_state
 → server.js broadcasts to all clients + Genesis bridge
 → ROS: ROSLIB publishes to /cmd_vel
```

### Telemetry

```
Sensors / IIO / IMU / GPS → ROS 2 topics
 → rosbridge → Browser TelemetryPanel charts
Genesis sim → genesis_training_metrics, genesis_obs_breakdown
 → Socket.io → Browser TrainingDashboard
```

### Video

Current: Genesis bridge streams JPEG frames over WebSocket (port 9091), displayed in `SimViewer`.

Planned (Phase 2+): Genesis GPU render → NVENC H.264/HEVC encode → SHM ringbuffer → Rust transport server → WebSocket/WebRTC fanout → Browser WebCodecs decode (`H264Decoder.js`).

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

## Multi-service backend (planned)

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
| Caddy | Go | Reverse proxy, Tailscale TLS | edge + backplane |
| transport-server | Rust | Frame fanout, WebSocket/WebRTC | backplane |
| genesis-sim | Python | Genesis simulation, NVENC encoding | backplane |
| ros-bridge | Python | ROS 2 ↔ NATS relay, topic allowlist | backplane |
| training-runner | Python | genesis-forge RL, rsl_rl PPO | backplane |
| NATS | NATS | Message bus + JetStream | backplane |

### Data plane

| Data type | Transport | Rationale |
|-----------|-----------|-----------|
| Video frames (encoded) | SHM ringbuffer | Zero-copy, ~50–500 KB/frame |
| Events / lifecycle | NATS (plain) | Simple pub/sub |
| Training data | NATS JetStream | Durability, replay |
| Control commands | WebRTC DataChannel | Lowest latency |

## Implementation phases

| Phase | Scope | Status |
|-------|-------|--------|
| **1** | CUDA Docker + basic pipeline: NATS, Caddy, ROS 2 Jazzy container, SHM ringbuffer, pytest, justfile, NVENC validation, docs. | **Complete** |
| **2** | Rust transport server: SHM reader, WebSocket fanout, NATS telemetry. | Planned |
| **3** | Production hardening: healthchecks, host tuning, Prometheus/Grafana, CI. | Planned |
| **4** | WebRTC + control path: signaling, DataChannel, H.264 RTP, WebCodecs. | Planned |
| **5** | Training integration: JetStream streams, genesis-forge, episode recording. | Planned |
| **6** | Distribution (future): NATS leafnodes, multi-node, capability roles. | Planned |

See [Phase 1 Implementation Plan](plans/2026-02-05-phase1-cuda-docker-basic-pipeline.md) for the 12-task breakdown.

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
