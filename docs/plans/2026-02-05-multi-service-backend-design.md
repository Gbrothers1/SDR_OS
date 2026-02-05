# SDR_OS Multi-Service Backend Architecture

**Date:** 2026-02-05
**Status:** Approved
**Author:** Claude + Ethan

## Overview

This document defines the architecture for SDR_OS's backend services: simulation, video streaming, ROS2 integration, and RL training. The design prioritizes low-latency video delivery, zero-copy data paths, and explicit failure handling.

### Goals

- 60fps H.264/HEVC video from Genesis simulation to browser (WebCodecs)
- Sub-100ms end-to-end control latency
- Clean separation: simulation, transport, ROS bridge, training
- Sparse RAM/CPU usage, leverage NVENC for encoding
- Distributable architecture (single box now, multi-node later)

### Non-Goals

- AV1 encoding (RTX 2080 Ti doesn't support it)
- Kubernetes (use Docker Compose + profiles)
- Browser-side rendering (server-side NVENC is the path)

---

## 1. Service Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                  DOCKER COMPOSE                                  │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│  ══════════════════════════ EDGE NETWORK (external) ════════════════════════    │
│                                      │                                           │
│                              ┌───────┴───────┐                                  │
│                              │    webserver   │  :443 (Tailscale TLS)           │
│                              │    (Caddy)     │  :80 → 443 redirect             │
│                              └───────┬───────┘                                  │
│                                      │                                           │
│         ┌────────────────────────────┼────────────────────────────┐             │
│         │ /stream/*                  │ /ros/*                     │             │
│         ▼                            ▼                            ▼             │
│  ┌──────────────────┐    ┌──────────────────┐    ┌──────────────────┐          │
│  │ transport-server │    │    ros-bridge    │    │   web app        │          │
│  │     (Rust)       │    │    (Python)      │    │   (static)       │          │
│  └────────┬─────────┘    └────────┬─────────┘    └──────────────────┘          │
│           │                       │                                              │
│  ═══════ shm+uds ════            │                                              │
│           │              ┌────────┴─────────┐                                   │
│  ══════════════════════ BACKPLANE NETWORK (internal) ═══════════════════════    │
│           │              │       NATS       │                                   │
│           │              │   + JetStream    │                                   │
│           │              └────────┬─────────┘                                   │
│           │                       │                                              │
│           ▼                       ▼                                              │
│  ┌─────────────────────┐  ┌─────────────────┐  ┌─────────────────┐             │
│  │    genesis-sim      │  │   ros-bridge    │  │ training-runner │             │
│  │      (Python)       │  │   (backplane)   │  │    (Python)     │             │
│  │  GPU: RTX 2080 Ti   │  │                 │  │                 │             │
│  └─────────────────────┘  └─────────────────┘  └─────────────────┘             │
│                                                                                  │
│  Shared volume: sdr_ipc (tmpfs) at /dev/shm/sdr_os_ipc                          │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Services

| Service | Language | Role | Network |
|---------|----------|------|---------|
| webserver | Caddy | Reverse proxy, Tailscale TLS | edge + backplane |
| transport-server | Rust | Frame fanout, WebSocket/WebRTC, signaling | backplane |
| genesis-sim | Python | Genesis simulation, NVENC encoding | backplane |
| ros-bridge | Python | ROS2 ↔ NATS relay, topic allowlist | backplane |
| training-runner | Python | genesis-forge RL, RSL-RL PPO | backplane |
| nats | NATS | Message bus + JetStream for training | backplane |

### Network Isolation

- **Edge network**: Only Caddy exposed (true DMZ)
- **Backplane network**: Internal only, not routable externally
- Caddy joins both networks to reach services

---

## 2. Data Plane Architecture

### Transport Selection

| Data Type | Transport | Rationale |
|-----------|-----------|-----------|
| Video frames (encoded) | SHM ringbuffer | Zero-copy, local only, ~50-500KB/frame |
| High-rate sensors | zenoh (future) | Zero-copy local, WAN-capable |
| Events/lifecycle | NATS (plain) | Simple pub/sub, no persistence |
| Training data | NATS JetStream | Durability, replay |
| Control commands | WebRTC DataChannel | Low latency, direct to client |

### NATS Subject Namespaces

| Namespace | Persistence | Use |
|-----------|-------------|-----|
| `telemetry.*` | None | IMU, joint states, metrics |
| `sim.metrics.*` | None | Sim FPS, physics stats |
| `control.*` | None | Prefer WebRTC DataChannel |
| `train.>` | JetStream (7d, 10GB) | Training events, rewards |
| `episodes.>` | JetStream (30d, 30GB) | Episode recordings |
| `datasets.>` | JetStream (90d, 50GB) | Curated training data |

---

## 3. Video Pipeline (Zero-Copy)

```
Genesis Render (CUDA)
      │
      ▼ cuSurfaceObject (GPU memory)
┌─────────────┐
│ CUDA tensor │  Frame lives here, never leaves GPU until encode
└──────┬──────┘
       │
       ▼ NvEncMapInputResource (zero-copy to NVENC)
┌─────────────────┐
│ NVENC encoder   │  H.264/HEVC, Baseline/Main, CBR 8Mbps
└──────┬──────────┘
       │
       ▼ Encoded NAL units (~50-500KB/frame)
┌─────────────────┐
│ SHM ringbuffer  │  /dev/shm/sdr_os_ipc/frames (4MB)
└──────┬──────────┘
       │
       ▼ Unix DGRAM socket signal (hint only)
┌─────────────────┐
│transport-server │  Rust: mmap same shm, fan out to clients
└─────────────────┘
```

### SHM Ringbuffer Protocol

**Layout:**
```
[0:8]   sequence_number (written LAST - commit marker)
[8:16]  write_index
[16:]   frame data (header + payload, wrapped)
```

**Frame Header (32 bytes):**
```
frame_id:   uint64  # Application-level ID
frame_seq:  uint64  # Monotonic, never resets (lap detection)
size:       uint32  # Payload size
flags:      uint16  # FLAG_KEYFRAME, FLAG_WRAP_MARKER, etc.
codec:      uint16  # 1=H.264, 2=HEVC
crc32:      uint32  # zlib.crc32 of payload
reserved:   uint32
```

**Atomic Commit Pattern:**
1. Writer: write header + payload, then write_index, then sequence LAST
2. Reader: read seq1 → read write_index → read seq2 → accept only if seq1 == seq2

**Lap Detection:**
- Reader tracks `last_frame_seq`
- If `header.frame_seq > expected`: writer lapped, resync to IDR
- If `header.frame_seq < expected`: corruption, resync to IDR
- On resync: set `last_frame_seq = 0`, `awaiting_keyframe = true`, request IDR

**Drop Policy:**
- genesis-sim: "Latest wins" single-slot buffer (always encode freshest frame)
- transport-server: Per-client broadcast with lag detection → drop + request keyframe

---

## 4. Transport Server (Rust)

### Crate Structure

```
services/transport-server/
├── src/
│   ├── main.rs
│   ├── config.rs
│   ├── shm/
│   │   ├── reader.rs       # mmap consumer, lap detection
│   │   └── protocol.rs     # Frame header parsing
│   ├── fanout/
│   │   ├── broadcast.rs    # tokio::sync::broadcast
│   │   └── backpressure.rs # Per-client drop + keyframe recovery
│   ├── transport/
│   │   ├── websocket.rs    # axum WebSocket
│   │   ├── webrtc.rs       # webrtc-rs DataChannel + media
│   │   └── signaling.rs    # Offer/answer exchange
│   ├── nats/
│   │   └── subscriber.rs   # Telemetry relay
│   ├── metrics.rs          # Prometheus :9100
│   └── health.rs           # /health endpoint
```

### Transport Boundaries

| Transport | Direction | Buffer | Drop Policy | Rate Limit |
|-----------|-----------|--------|-------------|------------|
| SHM (frames) | In | 32 | Drop non-keyframes | 120 fps |
| NATS (telemetry) | In | 128 | Drop oldest | 1000/s |
| WebRTC DC (control) | In | 16 | Block (never drop) | 100/client/s |
| Fanout (frames) | Out | 64/client | Drop + request keyframe | - |

### Key Dependencies

```toml
tokio = { version = "1", features = ["full"] }
axum = { version = "0.7", features = ["ws"] }
webrtc = "0.9"
async-nats = "0.33"
memmap2 = "0.9"
crc32fast = "1.3"
```

---

## 5. Genesis-Sim (Python)

### Thread Model

```
┌─────────────────┐   LatestFrameSlot    ┌─────────────────┐
│  Genesis Sim    │ ──────────────────▶  │   Async Loop    │
│    (thread)     │   (latest wins)      │   (main)        │
│                 │                      │                 │
│  50Hz physics   │                      │  NVENC encode   │
│  60fps render   │                      │  SHM write      │
└─────────────────┘                      └─────────────────┘
```

### LatestFrameSlot (Thread-Safe)

```python
class LatestFrameSlot:
    """Single-slot 'latest wins' buffer."""

    def put(self, frame):  # Called from sim thread
        with self._lock:
            self._frame = frame
        self._loop.call_soon_threadsafe(self._event.set)

    async def wait_and_take(self):  # Called from async loop
        await self._event.wait()
        with self._lock:
            frame, self._frame = self._frame, None
        return frame
```

### Keyframe Debouncer

- Max 2 IDR/sec (500ms minimum interval)
- At least 30 frames between forced IDRs
- Prevents IDR spam from multiple client connects/lags

---

## 6. ROS Bridge (Python)

### Architecture

```
┌─────────────────┐     queue.Queue      ┌─────────────────┐
│  ROS Executor   │ ──────────────────▶  │   Async Loop    │
│    (thread)     │   RosMessage         │   (main)        │
│                 │                      │                 │
│  - rate limit   │                      │  - serialize    │
│  - queue.put()  │                      │  - nats.publish │
└─────────────────┘                      └─────────────────┘
```

### Topic Allowlist (Default Deny)

```python
topic_rules = [
    # ROS2 → NATS (telemetry out)
    TopicRule("/joint_states", "ros2nats", "telemetry.joints", rate_limit_hz=100),
    TopicRule("/imu/data", "ros2nats", "telemetry.imu", rate_limit_hz=200),
    TopicRule("/odom", "ros2nats", "telemetry.odom", rate_limit_hz=50),
    TopicRule("/tf", "ros2nats", "telemetry.tf", rate_limit_hz=100),

    # NATS → ROS2 (control in)
    TopicRule("/cmd_vel", "nats2ros", "control.cmd_vel", rate_limit_hz=50),
    TopicRule("/emergency_stop", "nats2ros", "control.estop"),
]

blocked_patterns = [
    "/rosout", "/parameter_events", "/**/set_parameters", ...
]
```

### Topic Matcher Semantics

| Pattern | Matches | Doesn't Match |
|---------|---------|---------------|
| `/cmd_vel` | `/cmd_vel` | `/cmd_vel_raw`, `/foo/cmd_vel` |
| `/camera/*` | `/camera/front` | `/camera/front/image` |
| `/camera/**` | `/camera/front`, `/camera/front/image/raw` | `/camera` |
| `/**/set_parameters` | `/node/set_parameters`, `/ns/node/set_parameters` | - |

---

## 7. Host Tuning

### IRQ Pinning

```bash
# Pin NIC IRQs to cores 0-1 (away from sim on 2-5)
# Auto-detect interface, handle multi-queue NICs
# Use smp_affinity_list when available
```

### GPU Setup

```bash
nvidia-smi -pm 1                    # Persistence mode
nvidia-smi -lgc <max-100>,<max>     # Lock clocks (query supported range)
nvidia-smi -lmc <max>,<max>         # Lock memory clock
```

### CPU Pinning

```yaml
# docker-compose.yml
genesis-sim:
  cpuset: "2,3,4"
transport-server:
  cpuset: "5"
```

### Systemd Unit

```ini
[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/local/bin/sdr-irq-affinity.sh
ExecStart=-/usr/local/bin/sdr-gpu-setup.sh   # Continue on failure
ExecStart=-/usr/local/bin/sdr-cpu-setup.sh
```

---

## 8. Capability Negotiation

Nodes advertise capabilities on startup via NATS:

```yaml
node_id: "gpu-host-01"
capabilities:
  gpu:
    type: "NVIDIA GeForce RTX 2080 Ti"
    vram_gb: 11
    nvenc: true
    nvenc_sessions_max: 2
    nvenc_codecs: ["h264", "hevc"]
  cpu:
    cores: 6
    threads: 12
    hot_cores: [2, 3, 4, 5]
    irq_cores: [0, 1]
  memory:
    total_gb: 8
    available_for_workloads_gb: 6
  roles_supported: ["sim", "encode", "train"]
```

Orchestrator assigns roles based on capabilities:
- `sim`/`encode`: requires NVENC
- `train`: requires GPU + 8GB+ VRAM

---

## 9. Docker Compose Structure

### Profiles

| Profile | Services | Use |
|---------|----------|-----|
| `dev` | hot-reload variants, debug ports | Development |
| `sim` | genesis-sim, transport-server | Simulation + streaming |
| `train` | training-runner | RL training |
| `obs` | prometheus, grafana, loki | Observability |
| `tools` | lint, test, docs | CI/tooling |

### Healthchecks

All services have healthchecks. Caddy uses `depends_on: condition: service_healthy`.

### Volumes

```yaml
volumes:
  sdr_ipc:           # tmpfs for SHM ringbuffer
    driver_opts:
      type: tmpfs
      o: size=512m
  nats_data:         # JetStream persistence
  prometheus_data:
  grafana_data:
```

---

## 10. Observability

### Metrics (Prometheus)

- `frames_received_total{codec}`
- `frames_dropped_lag{client_id}`
- `encode_latency_us`
- `shm_queue_depth`
- `connected_clients`
- `nats_messages_total{subject}`

### Dashboards (Grafana)

- Frame encode latency (p50, p99)
- NATS messages/sec by subject
- GPU utilization
- SHM queue depth
- Client connection count

### Structured Logging

All services emit JSON logs with:
- `timestamp`, `level`, `service`, `message`
- Correlation IDs for request tracing

---

## 11. Implementation Phases

### Phase 1: CUDA Docker + Basic Pipeline
1. Copy docker-compose from Genesis/SDR_OS
2. Get ROS2 Jazzy container running with host network
3. Validate NVENC availability
4. Basic SHM ringbuffer (Python writer, Python reader)
5. Document in docs/ and documents/

### Phase 2: Rust Transport Server
1. Scaffold Rust crate with tokio + axum
2. Implement SHM reader with lap detection
3. WebSocket frame fanout
4. NATS telemetry relay
5. Integration test with genesis-sim

### Phase 3: Production Hardening
1. Healthchecks + proper depends_on
2. Host tuning scripts (IRQ, GPU, CPU)
3. Observability stack (Prometheus, Grafana, Loki)
4. CI pipeline (lint, test, build, scan)

### Phase 4: WebRTC + Control Path
1. WebRTC signaling server
2. DataChannel for control (< 10ms latency)
3. Media track for video (H.264 RTP)
4. Browser WebCodecs integration

### Phase 5: Training Integration
1. JetStream streams for training data
2. training-runner with genesis-forge
3. Episode recording + replay
4. Real robot data collection via ros-bridge

### Phase 6: Distribution (Future)
1. NATS leafnodes for multi-node
2. Capability-based role assignment
3. Local registry for fast image pulls

---

## Appendix A: Key Files to Create

```
/home/ethan/SDR_OS/
├── docker-compose.yml              # Multi-profile compose
├── configs/
│   ├── Caddyfile                   # Tailscale TLS + reverse proxy
│   ├── nats.conf                   # JetStream config
│   └── prometheus.yml
├── containers/
│   ├── cuda/Dockerfile             # Multi-stage, BuildKit cache
│   ├── ros-jazzy/Dockerfile
│   └── tools/Dockerfile
├── services/
│   └── transport-server/           # Rust crate
├── genesis_bridge/                 # Python package
│   ├── shm/ringbuffer.py
│   └── ...
├── ros_bridge/                     # Python package
│   ├── allowlist.py
│   └── ...
├── scripts/
│   ├── sdr-irq-affinity.sh
│   ├── sdr-gpu-setup.sh
│   └── sdr-cpu-setup.sh
└── justfile                        # Task runner
```

---

## Appendix B: Hardware Reference

| Component | Spec |
|-----------|------|
| GPU | NVIDIA GeForce RTX 2080 Ti, 11GB VRAM |
| NVENC | 2 concurrent sessions, H.264 + HEVC |
| CPU | AMD Ryzen 5 2600, 6 cores / 12 threads |
| RAM | 8 GB |
| Network | Ethernet / WiFi 6 |

---

## Appendix C: Browser Requirements

- **Chrome/Chromium**: Full WebCodecs support
- **Safari**: Newer versions, patchier
- **Firefox**: WebCodecs landed later

WebCodecs required for low-latency H.264 decode (not MSE/HLS).
