# SDR_OS: NATS Transport, Safety Stack & Service Connectivity

**Date:** 2026-02-06
**Status:** Approved
**Scope:** Replace Socket.io sim relay with NATS backbone, add 3-layer safety stack, fix defaults, align docs

---

## 1. Fix Default Settings & Auto-Discovery

- `genesis.enabled`: `false` → `true` (auto-connect on page load)
- `genesis.bridgeWsUrl`: keep `''` (auto-discovers `/stream/ws` on same origin)
- `genesis.streamBackend`: `'websocket'` (JPEG fallback via WS, H.264 primary)
- Settings.jsx `bridgeWsUrl`: remove hardcoded `:9091` → `''` (match SettingsContext)
- Settings.jsx `webrtcSignalingUrl`: remove stale `:9092` (signaling is 0x03 on same WS)
- webpack.config.js: remove hardcoded `192.168.12.147` IP

## 2. H.264 Primary, JPEG Fallback

### Encoder (genesis sim runner, NVENC on GPU):
- PyAV (`av` package) wrapping FFmpeg's `h264_nvenc`
- Preset: `p4` (low-latency), tune: `ull` (ultra-low-latency)
- Profile: Baseline (`avc1.42E01E`) for browser compatibility
- GOP: 30 frames (1 IDR/sec at 30fps)
- Bitrate: 2-4 Mbps (configurable)
- Output: raw Annex-B NAL units → SHM with `codec=H264`, `flags=KEYFRAME` on IDR
- Fallback: if NVENC init fails → JPEG encoder, `codec=JPEG`

### Frontend codec routing (GenesisContext `handleVideoFrame`):
- Parse header → check codec field
- `CodecType.H264`: init H264Decoder(`avc1.42E01E`), feed payload, render to canvas
- `CodecType.JPEG`: Blob → `URL.createObjectURL()` → `<img>` tag (revoke previous URL)
- Auto-detect from first frame, no negotiation needed

### SHM protocol codec values:
- `1` = H.264
- `2` = HEVC
- `3` = JPEG

## 3. Binary WS Protocol

| Byte | Type | Direction | Payload |
|------|------|-----------|---------|
| `0x01` | VIDEO | transport → browser | 32-byte LE header + encoded frame |
| `0x02` | TELEMETRY | transport → browser | u16 subject_len + subject + JSON |
| `0x03` | SIGNALING | bidirectional | JSON (WebRTC offer/answer/ICE) |
| `0x04` | COMMAND | browser → transport | JSON (action + cmd_seq + ttl_ms + data) |

### 0x04 Command format:
```json
{
  "action": "set_cmd_vel",
  "cmd_seq": 14207,
  "ttl_ms": 200,
  "data": {"linear_x": 0.5, "linear_y": 0.0, "angular_z": 0.1}
}
```

- `cmd_seq` — monotonic, browser increments per command, sim ignores out-of-order
- `ttl_ms` — command validity window, sim decays after expiry
- `action` — maps to NATS subject: `command.genesis.{action}`

## 4. NATS Subjects

### Commands (browser → transport → NATS → sim):

| Subject | Payload |
|---------|---------|
| `command.genesis.set_cmd_vel` | `{linear_x, linear_y, angular_z}` |
| `command.genesis.pause` | `{paused: bool}` |
| `command.genesis.reset` | `{}` |
| `command.genesis.load_robot` | `{robot_name}` |
| `command.genesis.unload_robot` | `{}` |
| `command.genesis.set_mode` | `{mode}` |
| `command.genesis.load_policy` | `{checkpoint_dir, model_file?}` |
| `command.genesis.set_alpha` | `{alpha}` |
| `command.genesis.estop` | `{reason?}` |
| `command.genesis.estop_clear` | `{cmd_seq}` |
| `command.genesis.camera` | `{position, lookat}` |
| `command.genesis.settings` | `{jpeg_quality, stream_fps, camera_res}` |

### Telemetry (sim → NATS → transport → browser):

| Subject | Rate | Payload |
|---------|------|---------|
| `telemetry.training.metrics` | 1 Hz | `{step, fps, policy_loaded, total_reward, episode_length}` |
| `telemetry.frame.stats` | 1 Hz | `{frame_id, encode_ms, codec, frame_bytes}` |
| `telemetry.reward.breakdown` | 1 Hz | `{per_component_rewards}` |
| `telemetry.obs.breakdown` | 1 Hz | `{per_component_observations}` |
| `telemetry.velocity.command` | 5 Hz | `{linear_x, linear_y, angular_z}` |
| `telemetry.safety.state` | 2 Hz | `{state_id, mode, reason, since_ms}` |
| `telemetry.safety.cmd_timeout` | on event | `{mode, decaying, velocity}` |
| `telemetry.command.ack` | on event | `{action, cmd_seq, status, detail}` |

### Safety (transport → NATS, informational):

| Subject | Rate | Payload |
|---------|------|---------|
| `telemetry.safety.video_gate` | on event | `{gated, mode, stale_ms}` |

### Link health (client-side computed, not NATS):

| Metric | Source | How |
|--------|--------|-----|
| WS link health | browser | track time since last 0x01 or 0x02 received |
| NATS link health | browser | infer from telemetry.safety.state arrival (2 Hz expected) |
| Video link health | browser | track time since last 0x01 VIDEO frame |

## 5. Safety Stack: HOLD & ESTOP

### Two named modes:

**HOLD** — zero velocity, system armed, auto-recoverable
- Triggered by: video stale, command timeout
- TrustStrip: yellow flash, "HOLD" badge

**ESTOP** — zero velocity, policy disabled, requires operator re-arm
- Triggered by: prolonged stale (>5s video, >2s cmd), repeated HOLD (3+ in 10s), operator button
- TrustStrip: red flash, "E-STOP" badge, re-arm prompt

### Layer 1: Frontend Video-Health Gate (500ms)

- Track `lastFrameTime` on every 0x01 received
- Stale >500ms → `videoHealthy = false`
- Stop emitting velocity commands (don't send, not just zero)
- Show "VIDEO LOST — HOLD" overlay
- Frames resume → `videoHealthy = true` → resume automatically
- **Cosmetic only** — does not control the robot, just stops the browser from sending

### Layer 2: Transport-Server Command Gate (1s / 5s)

- Track `last_video_frame_ts` from SHM reader
- Track `gate_active: bool` (latch)
- On 0x04 `set_cmd_vel` with video stale >1s AND `gate_active == false`:
  - Publish **one** zero `command.genesis.set_cmd_vel` → `{linear_x:0, linear_y:0, angular_z:0, _safety:"hold"}`
  - Set `gate_active = true`
  - Publish `telemetry.safety.video_gate` → `{gated:true, mode:"HOLD", stale_ms:N}`
- While `gate_active`: silently drop all velocity commands
- Video stale >5s:
  - Publish `command.genesis.estop` → `{reason:"video_timeout", _safety:"estop"}`
  - Publish `telemetry.safety.video_gate` → `{gated:true, mode:"ESTOP", stale_ms:N}`
  - Block all commands except `estop`, `estop_clear`, `pause`, `reset`
- Video recovers:
  - Set `gate_active = false`
  - Publish `telemetry.safety.video_gate` → `{gated:false}`

### Layer 3: Genesis-Sim Command TTL & Decay (200ms / 2s)

- Track `last_cmd_seq` — ignore any `cmd_seq <= last_cmd_seq` (out-of-order protection)
- Track `last_cmd_vel_time`
- Subscribe to `telemetry.safety.video_gate` from transport
- Command TTL expired (>200ms no fresh cmd):
  - **HOLD** — exponential decay: velocity * 0.5 per tick until < 0.01, then zero
  - Publish `telemetry.safety.cmd_timeout` → `{mode:"HOLD", decaying:true}`
- Command timeout >2s:
  - **ESTOP** — immediate zero, disable policy, reject commands
  - Publish `telemetry.safety.cmd_timeout` → `{mode:"ESTOP", reason:"command_timeout"}`

### Canonical safety state (sim is authority):

- Sim publishes `telemetry.safety.state` at 2 Hz:
  ```json
  {"state_id": 42, "mode": "ARMED|HOLD|ESTOP", "reason": "ok|cmd_timeout|video_gate|operator", "since_ms": 1234}
  ```
- `state_id` monotonic — UI ignores out-of-order updates after reconnects
- Sim determines mode from: own TTL + transport's `video_gate` + operator commands
- **UI reads only this topic** for badge/overlay — no local reconciliation

### Re-arm sequence:

1. Operator presses E-STOP toggle
2. Browser sends 0x04 `command.genesis.estop_clear` → `{cmd_seq: N}`
3. Sim checks `video_gate.gated == false` (trusts transport's view)
4. If video healthy → exit ESTOP, publish `{state_id:N, mode:"ARMED", reason:"operator_clear"}`
5. If video dead → reject, publish `{state_id:N, mode:"ESTOP", reason:"no_video"}`

### Never gated (always pass through all layers):
- `estop`
- `estop_clear`
- `pause`
- `reset`

### Stack timing:

```
Video drops
  500ms   Layer 1 (browser): stop sending cmds, show HOLD overlay
  1000ms  Layer 2 (transport): latch zero, gate cmds, notify
  200ms   Layer 3 (sim): TTL decay (no fresh cmd from any source)
  2s      Layer 3 (sim): ESTOP — full stop, require re-arm
  5s      Layer 2 (transport): ESTOP — block everything, notify
```

## 6. Socket.io — Browser-Only UX

### Stays:
- `controller_button_states` — gamepad broadcast between browser tabs
- `controller_joystick_state` — volatile joystick relay
- `controller_mapping_type` — Steam Deck vs Xbox detection
- `controller_vibration` — haptics
- `/api/status` — HTTP health

### Removed from server.js:
- Entire `GENESIS_EVENTS` array (48 events) and `forEach` broadcast loop
- `genesis_identify` handler
- `genesisBridgeConnected` tracking
- ~100 lines deleted

### Socket.io is optional:
- If it dies: video, telemetry, commands all work via NATS/WS
- Only loss: multi-client gamepad sharing
- Single-user Steam Deck: gamepad → `command.genesis.set_cmd_vel` via 0x04

### Gamepad dual-path:
- ControlOverlay → Socket.io (other browsers see joystick state)
- ControlOverlay → 0x04 WS (sim receives velocity command)
- Independent — either can fail without affecting the other

## 7. Reverse Proxy & Networking

### Production (Caddy :443):

```
handle /stream/ws   { reverse_proxy transport-server:8080 }
handle /socket.io/* { reverse_proxy node:3000 }
handle /ros/*       { reverse_proxy ros-bridge:9090 }
handle /api/*       { reverse_proxy node:3000 }
handle /health      { respond "OK" 200 }
handle              { root * /srv/www; file_server }
```

Caddy WS timeout: set `flush_interval -1` and `transport http { read_timeout 0 }` on `/stream/ws` to prevent long-running stream kills.

### Backplane (no exposed ports):
- `genesis-sim` — outbound only: SHM write, NATS connect
- `transport-server` — :8080 internal
- `nats` — :4222 internal
- `ros-bridge` — :9090 internal
- `node` — :3000 internal

### Dev mode:
- Node on host :3000, proxies `/stream/ws` → `localhost:8080`
- transport-server port-mapped `8080:8080`
- ros-bridge host network → `localhost:9090`
- Browser → `localhost:3000`

### docker-compose production changes:
- genesis-sim: remove `network_mode: host` → `networks: [backplane]`
- transport-server: remove `ports: ["8080:8080"]`
- ros-bridge: remove `network_mode: host` → `networks: [backplane]`
- Add `node` service: `networks: [edge, backplane]`
- Caddy: uncomment all proxy routes

## 8. Environment Variables

| Var | Used by | Dev default | Prod default |
|-----|---------|-------------|--------------|
| `SDR_NATS_URL` | transport, sim | `nats://localhost:4222` | `nats://nats:4222` |
| `SDR_SHM_PATH` | transport, sim | `/dev/shm/sdr_os_ipc/frames` | same |
| `SDR_SHM_SIZE` | transport, sim | `4194304` (4MB) | same |
| `SDR_LISTEN_ADDR` | transport | `0.0.0.0:8080` | same |
| `TRANSPORT_HOST` | server.js proxy | `localhost` | `transport-server` |
| `TRANSPORT_PORT` | server.js proxy | `8080` | `8080` |

## 9. Smoke Tests

| Test | What it validates |
|------|-------------------|
| `test_ws_frame_delivery` | WS upgrade to `/stream/ws`, confirm 0x01 within 2s |
| `test_nats_command_roundtrip` | Publish `command.genesis.pause`, confirm `telemetry.command.ack` |
| `test_safety_video_gate` | Stop SHM writes, confirm `telemetry.safety.video_gate` fires <2s |
| `test_cmd_ttl_decay` | Send one velocity, wait 500ms, confirm velocity near zero |
| `test_safety_state_authority` | Stale video + stale cmd, verify UI badge changes only from `telemetry.safety.state` |

## 10. Implementation Order

1. **Protocol + schemas** — lock 0x01-0x04 message formats and NATS subject table
2. **Transport gate + latch zero** — implement `gate_active`, `telemetry.safety.video_gate`
3. **Sim safety authority** — subscribe `video_gate`, enforce TTL, publish `telemetry.safety.state`
4. **Frontend** — consume `telemetry.safety.state` for badge/overlay, local HOLD cosmetic only
5. **Delete Socket.io Genesis junk** — remove GENESIS_EVENTS, keep controller sharing
6. **H.264 encoder** — NVENC in sim runner, JPEG fallback
7. **JPEG codec path** — frontend Blob → img for fallback
8. **Proxy + compose** — flip dev → prod networking
9. **Defaults + config** — fix all stale URLs, env vars, settings
10. **Docs + smoke tests** — update CLAUDE.md, add NATS schema doc, bake in tests
