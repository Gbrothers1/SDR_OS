# Phase 1 Architecture Solutions

**Phase:** 1 — CUDA Docker + Basic Pipeline
**Branch:** `feature/phase1-cuda-docker-pipeline`
**Date:** 2026-02-05

---

## 1. Service Topology

The full SDR_OS backend is 12 services across 7 Docker Compose profiles with network isolation between edge (browser-facing) and backplane (internal) traffic.

```
                         ┌─────────────────────────────────────────────────┐
                         │                  EDGE NETWORK                   │
                         │                                                 │
                         │   ┌─────────┐                                   │
  Browser ──────────────────▶│  Caddy   │  :80 (webserver)                 │
                         │   │ static  │                                   │
                         │   └────┬────┘                                   │
                         │   ┌────┼──────────┬──────────────┐              │
                         │   │    │          │              │              │
                         │   ▼    ▼          ▼              ▼              │
                         │ ┌──────────┐ ┌──────────┐  ┌───────────────┐   │
                         │ │transport │ │  node    │  │  ros-bridge   │   │
                         │ │ server   │ │ :3000   │  │  (host net)   │   │
                         │ │ :8080    │ │ gamepad │  │  :9090        │   │
                         │ └────┬─────┘ └─────────┘  └───────────────┘   │
                         │      │                                         │
                         └──────┼─────────────────────────────────────────┘
                                │
                         ┌──────┼─────────────────────────────────────────┐
                         │      │          BACKPLANE NETWORK (internal)    │
                         │      ▼                                         │
                         │ ┌──────────┐        ┌──────────┐               │
                         │ │  SHM     │◀──────▶│  NATS    │               │
                         │ │ tmpfs    │        │ JetStream│               │
                         │ │ /dev/shm │        │ :4222    │               │
                         │ └────┬─────┘        └────┬─────┘               │
                         │      │                    │                    │
                         │ ┌────▼─────┐        ┌────▼──────┐             │
                         │ │genesis   │        │ training  │             │
                         │ │  sim     │        │  runner   │             │
                         │ │(GPU:0)   │        │ (GPU:0)   │             │
                         │ └──────────┘        └───────────┘             │
                         │                                                │
                         │ ┌──────────┐  ┌──────────┐                     │
                         │ │Prometheus│  │ Grafana  │                     │
                         │ │ :9090    │  │ :3001    │                     │
                         │ └──────────┘  └──────────┘                     │
                         └────────────────────────────────────────────────┘
```

### Service Registry

| Service | Profile | Network | Port | Purpose |
|---------|---------|---------|------|---------|
| `webserver` (Caddy) | `dev`, `sim` | edge + backplane | 80, 443 | Reverse proxy, static files, TLS |
| `node` | `dev`, `sim` | backplane | 3000 (internal) | Socket.io gamepad relay, /api |
| `transport-server` | `sim` | edge + backplane | 8080 | SHM→WS fanout, NATS relay, video gate |
| `ros-bridge` | `dev`, `sim` | host | 9090 | ROS2 rosbridge_server |
| `genesis-sim` | `sim` | host | — | Physics simulation (GPU) |
| `nats` | `sim`, `train` | backplane | 4222 | Event bus (JetStream) |
| `training-runner` | `train` | backplane | — | RL training loop (GPU) |
| `prometheus` | `obs` | backplane | 9090 | Metrics collection |
| `grafana` | `obs` | backplane | 3001 | Metrics dashboard |
| `app-cuda` | `cuda` | — | — | CUDA CI target |
| `app-rocm` | `rocm` | — | — | ROCm CI target |
| `app-mlx` | `mlx` | — | — | MLX CI target |

---

## 2. Network Isolation

Two Docker networks enforce security boundaries:

```
┌──────────────────────────────────────────────────────┐
│                    EDGE NETWORK                       │
│              (bridge, internet-facing)                │
│                                                       │
│  webserver (Caddy) ◀──▶ node ◀──▶ transport-server   │
│                                                       │
│  ros-bridge (host network — bypasses Docker for DDS)  │
└──────────────────────────────────────────────────────┘
        ▲
        │ webserver + transport-server have dual membership
        ▼
┌──────────────────────────────────────────────────────┐
│              BACKPLANE NETWORK                        │
│          (bridge, internal: true)                     │
│       No direct internet access                      │
│                                                       │
│  genesis-sim ◀──▶ SHM tmpfs ◀──▶ transport-server    │
│       │                                ▲              │
│       ▼                                │              │
│     NATS ◀──────────────────▶ training-runner         │
│       ▲                                               │
│       │                                               │
│  prometheus ──▶ grafana                               │
└──────────────────────────────────────────────────────┘
```

**Key design choice:** `webserver` (Caddy) and `transport-server` are on both networks — Caddy routes browser traffic to the appropriate backend, while transport-server bridges SHM frame data from the backplane to browser clients on the edge.

---

## 3. Video Pipeline (Zero-Copy)

The video pipeline moves rendered frames from simulation to browser with minimal copying:

```
┌────────────┐     NVENC      ┌─────────────┐    mmap     ┌─────────────┐
│ Genesis    │───(H.264/HEVC)──▶│    SHM      │◀──(zero───▶│ Transport   │
│ Simulator  │    GPU encode   │ Ringbuffer  │   copy)    │   Server    │
│            │                 │  /dev/shm   │            │  (Rust)     │
└────────────┘                 └─────────────┘            └──────┬──────┘
                                                                 │
                                                          WebSocket fanout
                                                                 │
                                                    ┌────────────┼────────────┐
                                                    ▼            ▼            ▼
                                               ┌────────┐  ┌────────┐  ┌────────┐
                                               │Browser │  │Browser │  │Browser │
                                               │Client 1│  │Client 2│  │Client 3│
                                               └────────┘  └────────┘  └────────┘
```

### Frame Flow

1. **Genesis renders** a scene on the GPU
2. **NVENC encodes** the frame to H.264 or HEVC (hardware encoder, no CPU)
3. **Writer** places the encoded frame into the SHM ringbuffer (mmap'd tmpfs)
4. **Transport server** reads the frame via mmap (zero-copy — same physical memory)
5. **WebSocket fanout** sends the frame to all connected browsers

---

## 4. SHM Ringbuffer Memory Layout

The shared memory region uses a fixed layout with an atomic commit protocol:

```
Byte Offset    Field                Size    Description
──────────────────────────────────────────────────────────────
CONTROL REGION (16 bytes)
──────────────────────────────────────────────────────────────
[0:8]          sequence_number      8B      Written LAST (commit marker)
[8:16]         write_index          8B      Current write position

──────────────────────────────────────────────────────────────
FRAME REGION (starts at byte 16)
──────────────────────────────────────────────────────────────
[16:24]        frame_id             8B      Application-level frame ID
[24:32]        frame_seq            8B      Monotonic sequence number
[32:36]        size                 4B      Payload size in bytes
[36:38]        flags                2B      KEYFRAME=0x01, WRAP=0x02
[38:40]        codec                2B      1=H.264, 2=HEVC
[40:44]        crc32                4B      Payload integrity check
[44:48]        reserved             4B      Future use
[48:48+size]   payload              var     Encoded frame data
```

### Atomic Commit Protocol

```
Writer                              Reader
──────                              ──────
1. Write frame data (payload)
2. Write frame header
3. Write write_index                1. Read sequence_number (seq1)
4. Write sequence_number ← LAST    2. Read frame header + payload
   (memory barrier)                 3. Read sequence_number (seq2)
                                    4. If seq1 ≠ seq2 → TORN READ → retry
                                    5. If seq1 = last_seen → NO NEW DATA
                                    6. Validate CRC32
                                    7. Check frame_seq for lap detection
```

### Lap Detection State Machine

```
                    ┌─────────────────────┐
                    │     NORMAL READ     │
                    │  (frame_seq check)  │
                    └──────────┬──────────┘
                               │
                    frame_seq gap detected
                    (skipped > 1 or went backwards)
                               │
                               ▼
                    ┌─────────────────────┐
                    │   RESYNC MODE       │
                    │  (waiting for       │──── non-keyframe ──▶ SKIP
                    │   next keyframe)    │
                    └──────────┬──────────┘
                               │
                        keyframe arrives
                        (flags & KEYFRAME)
                               │
                               ▼
                    ┌─────────────────────┐
                    │   NORMAL READ       │
                    │  (resume delivery)  │
                    └─────────────────────┘
```

---

## 5. CI Pipeline Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    GITHUB ACTIONS CI                          │
│                                                              │
│  Trigger: push to main/dev, all PRs                          │
│                                                              │
│  ┌─────────────────── FREE RUNNERS (ubuntu-latest) ────────┐ │
│  │                                                          │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │ │
│  │  │  validate-   │  │  lint-and-   │  │  rust-unit   │   │ │
│  │  │  compose     │  │  unit        │  │              │   │ │
│  │  │              │  │              │  │  cargo test  │   │ │
│  │  │ docker       │  │ Python 3.12  │  │  transport-  │   │ │
│  │  │ compose      │  │ pytest unit  │  │  server      │   │ │
│  │  │ config       │  │ + SHM compat │  │  (7 modules) │   │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘   │ │
│  │                                                          │ │
│  │  ┌──────────────┐  ┌──────────────┐                      │ │
│  │  │ ros-jazzy-   │  │  docs-build  │                      │ │
│  │  │ build        │  │              │                      │ │
│  │  │ Build ROS2   │  │  mkdocs      │                      │ │
│  │  │ container    │  │  build       │                      │ │
│  │  └──────────────┘  └──────────────┘                      │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                              │
│  ┌────────────── SELF-HOSTED RUNNERS (GPU) ─────────────────┐ │
│  │                                                          │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐   │ │
│  │  │  cuda-smoke  │  │  rocm-smoke  │  │  mlx-smoke   │   │ │
│  │  │              │  │              │  │              │   │ │
│  │  │  linux,gpu,  │  │  linux,gpu,  │  │  macos,arm64 │   │ │
│  │  │  nvidia      │  │  rocm        │  │  mlx         │   │ │
│  │  │              │  │              │  │              │   │ │
│  │  │  NVENC       │  │  ROCm smoke  │  │  MLX smoke   │   │ │
│  │  │  validation  │  │  tests       │  │  tests       │   │ │
│  │  └──────────────┘  └──────────────┘  └──────────────┘   │ │
│  └──────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

---

## 6. Docker Compose Profiles

Profiles group services for different use cases. Only start what you need:

```
                    ┌──────────────────────────────────────────┐
                    │           docker compose --profile       │
                    └───────────────────┬──────────────────────┘
                                        │
              ┌─────────┬───────────┬───┴───┬──────────┬──────────┬──────────┐
              ▼         ▼           ▼       ▼          ▼          ▼          ▼
         ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
         │  dev   │ │  sim   │ │ train  │ │  obs   │ │  cuda  │ │  rocm  │ │  mlx   │
         │        │ │        │ │        │ │        │ │        │ │        │ │        │
         │web     │ │web     │ │train   │ │prom    │ │app-    │ │app-    │ │app-    │
         │node    │ │node    │ │nats    │ │grafana │ │cuda    │ │rocm    │ │mlx     │
         │ros     │ │ros     │ │        │ │        │ │        │ │        │ │        │
         │        │ │genesis │ │        │ │        │ │        │ │        │ │        │
         │        │ │nats    │ │        │ │        │ │        │ │        │ │        │
         │        │ │xport   │ │        │ │        │ │        │ │        │ │        │
         └────────┘ └────────┘ └────────┘ └────────┘ └────────┘ └────────┘ └────────┘

Usage:
  docker compose --profile dev up           # Development
  docker compose --profile sim up           # Full simulation
  docker compose --profile sim --profile obs up  # Sim + monitoring
```

---

## 7. Container Build Hierarchy

```
nvidia/cuda:12.4.1-runtime-ubuntu22.04     ros:jazzy-ros-base
         │                                         │
         ▼                                         ▼
  ┌──────────────┐                          ┌──────────────┐
  │  sdr_os-cuda │                          │sdr_os-ros-   │
  │              │                          │  jazzy       │
  │  + Python    │                          │              │
  │  + PyTorch   │                          │  + rosbridge │
  │  + ffmpeg    │                          │  + rclpy     │
  │  + PyAV      │                          │  + msgs      │
  └──────────────┘                          └──────────────┘

rocm/rocm-terminal:6.1.2                   python:3.11-slim
         │                                         │
         ▼                                         ▼
  ┌──────────────┐                          ┌──────────────┐
  │  sdr_os-rocm │                          │  sdr_os-mlx  │
  │              │                          │              │
  │  + Python    │                          │  + Python    │
  │  + deps      │                          │  + deps      │
  └──────────────┘                          └──────────────┘
```

---

## 8. NVENC Validation Flow

```
┌─────────────────────────────────────────────────────────┐
│            scripts/validate_nvenc.py                     │
│                                                         │
│  ┌──────────────┐                                       │
│  │ 1. CUDA Info │  torch.cuda.is_available()            │
│  │              │  device count, names, memory           │
│  └──────┬───────┘                                       │
│         │                                               │
│  ┌──────▼───────┐                                       │
│  │ 2. GPU Info  │  nvidia-smi --query-gpu=...           │
│  │              │  driver version, active sessions       │
│  └──────┬───────┘                                       │
│         │                                               │
│  ┌──────▼───────┐                                       │
│  │ 3. Codec    │  ffmpeg -encoders | grep nvenc         │
│  │    Support   │  h264_nvenc, hevc_nvenc availability  │
│  └──────┬───────┘                                       │
│         │                                               │
│  ┌──────▼───────┐                                       │
│  │ 4. Probe    │  Attempt real NVENC encode via PyAV    │
│  │    (encode) │  320x240 test frame → H.264            │
│  └──────┬───────┘                                       │
│         │                                               │
│  ┌──────▼───────┐                                       │
│  │ 5. Output   │  Structured JSON to stdout             │
│  │    (JSON)   │  Exit: 0=OK, 1=UNAVAIL, 2=NO_CUDA     │
│  └─────────────┘                                       │
└─────────────────────────────────────────────────────────┘
```
