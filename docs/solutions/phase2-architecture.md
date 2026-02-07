# Phase 2 Architecture Solutions

**Phase:** 2 — NATS Backbone + Safety Stack
**Commits:** `c0a52ac`..`70d29bc`
**Date:** 2026-02-06
**Version:** 0.2.0

---

## 1. Production Routing (Caddy)

All browser traffic enters through Caddy on port 80. Caddy routes to the appropriate backend service:

```
Browser
  │
  ▼
┌─────────────────────────────────────────────────────────┐
│                  Caddy (:80)                             │
│                                                          │
│  /stream/ws ──────▶ transport-server:8080                │
│                     (binary WS: video + telemetry)       │
│                     flush_interval: -1                   │
│                     read_timeout: 0                      │
│                                                          │
│  /socket.io/* ────▶ node:3000                            │
│                     (gamepad relay WS)                    │
│                                                          │
│  /api/* ──────────▶ node:3000                            │
│                     (health, status JSON)                 │
│                                                          │
│  /ros/* ──────────▶ ros-bridge:9090                      │
│                     (rosbridge WS)                        │
│                                                          │
│  /* ──────────────▶ /srv/www (static)                    │
│                     (SPA bundle from dist/)               │
│                                                          │
│  /health ─────────▶ respond "OK" 200                     │
└─────────────────────────────────────────────────────────┘
```

---

## 2. Binary WebSocket Protocol

Every binary frame on `/stream/ws` has a 1-byte type prefix:

```
┌──────────────────────────────────────────────────────────┐
│ Byte 0: Type   │ Remaining bytes: Payload                │
├──────────────────────────────────────────────────────────┤
│                                                          │
│  0x01  VIDEO       server → browser                      │
│  ┌──────────────────────────────────────────────┐        │
│  │ [0]    type = 0x01                           │        │
│  │ [1:33] 32-byte LE header                     │        │
│  │        frame_id (8B), frame_seq (8B),        │        │
│  │        size (4B), flags (2B), codec (2B),    │        │
│  │        crc32 (4B), reserved (4B)             │        │
│  │ [33:]  Annex-B NALUs (H.264) or JPEG blob   │        │
│  └──────────────────────────────────────────────┘        │
│                                                          │
│  0x02  TELEMETRY   server → browser                      │
│  ┌──────────────────────────────────────────────┐        │
│  │ [0]    type = 0x02                           │        │
│  │ [1:N]  NATS subject (null-terminated string) │        │
│  │ [N+1:] JSON payload                          │        │
│  └──────────────────────────────────────────────┘        │
│                                                          │
│  0x03  SIGNALING   reserved for WebRTC (Phase 4)         │
│                                                          │
│  0x04  COMMAND     browser → server                      │
│  ┌──────────────────────────────────────────────┐        │
│  │ [0]    type = 0x04                           │        │
│  │ [1:]   JSON {action, cmd_seq, data, ttl_ms?} │        │
│  └──────────────────────────────────────────────┘        │
│                                                          │
└──────────────────────────────────────────────────────────┘
```

### Codec routing (browser side)

```
0x01 VIDEO frame received
        │
        ├── header.codec == 1 (H.264)
        │   └──▶ H264Decoder.js (WebCodecs VideoDecoder → canvas)
        │
        ├── header.codec == 3 (JPEG)
        │   └──▶ new Blob([payload]) → URL.createObjectURL → <img>
        │
        └── header.codec == 2 (HEVC)
            └──▶ future (WebCodecs HEVC support)
```

---

## 3. NATS Message Flow

```
┌────────────────────────────────────────────────────────────────────────┐
│                        NATS Message Bus                                │
│                                                                        │
│  COMMANDS (browser → sim)                                              │
│  ═══════════════════════                                               │
│                                                                        │
│  Browser ──0x04──▶ transport-server ──NATS──▶ genesis-sim              │
│                                                                        │
│  command.genesis.set_cmd_vel   {cmd_seq, data: {vx, vy, omega}}       │
│  command.genesis.pause         {cmd_seq}                               │
│  command.genesis.reset         {cmd_seq}                               │
│  command.genesis.estop         {cmd_seq}                               │
│  command.genesis.estop_clear   {cmd_seq}                               │
│  command.genesis.load_robot    {cmd_seq, data: {robot_id}}            │
│  command.genesis.settings      {cmd_seq, data: {codec, h264_bitrate,  │
│                                 h264_preset, jpeg_quality, stream_fps, │
│                                 camera_res}}                           │
│                                                                        │
│  TELEMETRY (sim → browser)                                             │
│  ═════════════════════════                                             │
│                                                                        │
│  genesis-sim ──NATS──▶ transport-server ──0x02──▶ Browser              │
│                                                                        │
│  telemetry.training.metrics    {step, fps, reward, policy_name}       │
│  telemetry.safety.state        {state_id, mode, timestamp}  (2 Hz)    │
│  telemetry.safety.video_gate   {gated, reason, timestamp}             │
│  telemetry.command.ack         {action, cmd_seq, status}              │
│  telemetry.sim.status          {loaded, robot_id, env_count}          │
│                                                                        │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 4. Safety Stack State Machine (3 Layers)

```
┌─────────────────────────────────────────────────────────────────────┐
│ LAYER 3: Sim (genesis_sim_runner) — CANONICAL AUTHORITY             │
│                                                                     │
│   ┌─────────┐     cmd TTL expires    ┌─────────┐                   │
│   │  ARMED  │──────(200ms)──────────▶│  HOLD   │                   │
│   │         │◀─────────────────────── │         │                   │
│   │ normal  │    fresh cmd arrives    │  zero   │                   │
│   │ control │                        │ velocity│                   │
│   └─────────┘                        └────┬────┘                   │
│                                           │ 2s no commands         │
│                                           ▼                        │
│                                      ┌─────────┐                   │
│                                      │  ESTOP  │                   │
│                                      │         │                   │
│                                      │  zero   │◀── RE-ARM only    │
│                                      │ latched │    via TrustStrip  │
│                                      └─────────┘                   │
│   Publishes: telemetry.safety.state @ 2 Hz                         │
│   Fields: {state_id (monotonic), mode, timestamp}                  │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ LAYER 2: Transport (Rust) — VIDEO GATE                              │
│                                                                     │
│   ┌─────────┐     SHM stale 1s      ┌─────────┐                   │
│   │ PASSING │──────────────────────▶│  HOLD   │                   │
│   │         │◀─────────────────────── │         │                   │
│   │ frames  │     SHM refreshed      │ gate    │                   │
│   │ flowing │                        │ closed  │                   │
│   └─────────┘                        └────┬────┘                   │
│                                           │ SHM stale 5s           │
│                                           ▼                        │
│                                      ┌─────────┐                   │
│                                      │  ESTOP  │                   │
│                                      │         │                   │
│                                      │ latch   │                   │
│                                      │ zero    │                   │
│                                      └─────────┘                   │
│   Publishes: telemetry.safety.video_gate                           │
│   Sends: zero-velocity command.genesis.set_cmd_vel                 │
└─────────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────────┐
│ LAYER 1: Frontend (GenesisContext) — COSMETIC                       │
│                                                                     │
│   ┌─────────┐     500ms no video     ┌─────────────────┐           │
│   │ HEALTHY │───────────────────────▶│  VIDEO LOST     │           │
│   │         │◀─────────────────────── │                 │           │
│   │ video   │     frame arrives       │ red overlay     │           │
│   │ playing │                        │ pulse animation │           │
│   └─────────┘                        │ shows mode      │           │
│                                      └─────────────────┘           │
│   Sets: videoHealthy = false                                        │
│   UI: SimViewer overlay + TrustStrip safety badge                  │
└─────────────────────────────────────────────────────────────────────┘
```

### Mode transitions

```
ARMED ──(timeout)──▶ HOLD ──(extended timeout)──▶ ESTOP
  ▲                   │                             │
  │    fresh command   │                             │
  └────────────────────┘                             │
  ▲                                                  │
  │              operator RE-ARM (TrustStrip)        │
  └──────────────────────────────────────────────────┘
```

---

## 5. End-to-End Data Flow

```
                    CONTROL PATH (dual cmd_vel)
                    ═══════════════════════════

Gamepad ──▶ ControlOverlay (30Hz)
                │
                ├──▶ ROS /cmd_vel (geometry_msgs/Twist via rosbridge)
                │    └──▶ Any ROS2 robot (universal)
                │
                ├──▶ GenesisContext.sendVelocityCommand()
                │    │
                │    ▼
                │    0x04 COMMAND {action: "set_cmd_vel", cmd_seq: N, data: {vx, vy, omega}}
                │    │
                │    ▼
                │    /stream/ws ──▶ Caddy ──▶ transport-server (Layer 2 gate)
                │    │
                │    ▼
                │    NATS: command.genesis.set_cmd_vel
                │    │
                │    ▼
                │    genesis-sim ──▶ apply velocity ──▶ reset TTL timer
                │
                └──▶ Socket.io ──▶ node ──▶ broadcast (gamepad UI sync only)


                    VIDEO PATH
                    ══════════

Genesis render (GPU) ──▶ NVENC/JPEG encode ──▶ SHM ringbuffer write
                         (codec switchable at runtime via settings command)
                                                      │
                                             transport-server reads (mmap)
                                                      │
                                                      ▼
                                               0x01 VIDEO frame
                                               [type][32B header][payload]
                                                      │
                                               /stream/ws fanout
                                                      │
                                          ┌───────────┼───────────┐
                                          ▼           ▼           ▼
                                     Browser 1   Browser 2   Browser 3
                                          │
                                    codec == 1? ──▶ H264Decoder ──▶ canvas
                                    codec == 3? ──▶ Blob URL ──▶ <img>


                    TELEMETRY PATH
                    ══════════════

genesis-sim ──▶ NATS: telemetry.* ──▶ transport-server ──▶ 0x02 TELEMETRY ──▶ Browser
                                                                                │
                                                            GenesisContext parses subject + JSON
                                                            updates: trainingMetrics, safetyState
```

---

## 6. Node Service (Docker)

```
node:20-alpine
    │
    ▼
┌──────────────────────────┐
│  sdr_os-node             │
│                          │
│  /app/server.js          │
│  /app/package.json       │
│  /app/node_modules/      │
│                          │
│  Listens: 0.0.0.0:3000   │
│  Network: backplane only │
│                          │
│  Routes:                 │
│    Socket.io → gamepad   │
│    /api/status → JSON    │
│    /stream/ws → proxy    │
│      (dev mode only)     │
│                          │
│  Healthcheck:            │
│    wget 127.0.0.1:3000   │
│    /api/status           │
│    (IPv4 only — Alpine   │
│     resolves localhost   │
│     to ::1 first)        │
└──────────────────────────┘
```

---

## 7. Test Coverage Map

```
┌────────────────────────────────────────────────────────────────────┐
│                         TEST COVERAGE                              │
│                                                                    │
│  ┌─── CI (ubuntu-latest, every PR) ────────────────────────────┐  │
│  │                                                              │  │
│  │  Python unit:    test_shm_ringbuffer.py (17 tests)          │  │
│  │  Python compat:  test_shm_cross_language.py (3 tests)       │  │
│  │  Rust unit:      cargo test (7 modules, ~20 tests)          │  │
│  │                  protocol, reader, websocket, broadcast,    │  │
│  │                  backpressure, config, health                │  │
│  │                                                              │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  ┌─── GPU runner (self-hosted, cuda-smoke) ─────────────────────┐  │
│  │                                                              │  │
│  │  NVENC validation:  validate_nvenc.py                       │  │
│  │  CUDA smoke:        tests/smoke/ (when created)             │  │
│  │                                                              │  │
│  └──────────────────────────────────────────────────────────────┘  │
│                                                                    │
│  ┌─── Integration (requires docker compose --profile sim) ──────┐  │
│  │                                                              │  │
│  │  test_ws_frame_delivery.py     (transport-server + SHM)     │  │
│  │  test_nats_command_roundtrip.py (NATS + genesis-sim)        │  │
│  │  test_safety_video_gate.py     (transport video gate)       │  │
│  │  test_cmd_ttl_decay.py         (sim TTL → HOLD)            │  │
│  │  test_safety_state_authority.py (monotonic state_id)        │  │
│  │                                                              │  │
│  └──────────────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────────────┘
```
