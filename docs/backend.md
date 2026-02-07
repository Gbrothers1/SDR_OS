# Backend

The backend consists of a Node.js/Express server (`server.js`) for gamepad relay and API routes, a Rust transport server for binary video/telemetry streaming, and Caddy as a reverse proxy. Genesis simulation communication uses NATS messaging through the transport server — Socket.io is **not** used for sim data.

## Node server (`server.js`)

| Property | Value |
|----------|-------|
| Runtime | Node.js + Express |
| Port | 3000 (configurable via `PORT` env var) |
| Host | 0.0.0.0 (configurable via `HOST` env var) |
| Real-time | Socket.io (gamepad relay only) |
| Docker image | `sdr_os-node` (`containers/node/Dockerfile`) |

```bash
npm start          # production: node server.js
npm run dev:server # development: nodemon server.js (auto-restart)
```

### Static file serving (dev mode only)

In production, Caddy serves static files. In dev mode (bare-metal Node):

| Route | Directory | Purpose |
|-------|-----------|---------|
| `/` | `dist/` | Webpack-built SPA (bundle.js, index.html) |
| `/configs/*` | `configs/` | URDF registry, robot YAML configs |
| `/assets/*` | `assets/` | URDF files, 3D models (DAE, OBJ, MTL) |

### Socket.io events (gamepad relay)

These events are broadcast from any browser client to all connected clients for gamepad synchronization between tabs.

| Event | Payload | Transport |
|-------|---------|-----------|
| `robot_control` | Control command object | Reliable (default) |
| `controller_button_states` | Button state map (e.g. `{L1: true, R1: false, ...}`) | Reliable |
| `controller_joystick_state` | Joystick axes object | **Volatile** (drops if behind) |
| `controller_mapping_type` | `{type: string}` | Reliable |
| `controller_vibration` | Haptic feedback payload | Reliable |

The joystick state uses `io.volatile.emit` to drop stale frames rather than queue them — important for low-latency control.

### `/stream/ws` proxy

In dev mode (bare-metal Node), `server.js` proxies WebSocket connections at `/stream/ws` to `transport-server:8080`. In production, Caddy handles this route directly.

### `/api/status` endpoint

Returns server health and connection count as JSON.

## Transport server (Rust)

See `services/transport-server/` for full source. The transport server handles:

- **SHM → WS fanout**: Reads encoded video frames from the SHM ringbuffer and fans them out to browser clients over WebSocket as `0x01` VIDEO frames.
- **NATS → WS relay**: Subscribes to `telemetry.*` NATS subjects and forwards them to browsers as `0x02` TELEMETRY frames.
- **WS → NATS commands**: Receives `0x04` COMMAND frames from browsers and publishes them to `command.genesis.*` NATS subjects.
- **Video gate (Layer 2 safety)**: Monitors SHM freshness — HOLD after 1s stale, ESTOP after 5s.

Configuration is via `SDR_*` environment variables (see `services/transport-server/src/config.rs`).

```bash
cargo build --manifest-path services/transport-server/Cargo.toml
cargo test --manifest-path services/transport-server/Cargo.toml
```

## Caddy reverse proxy

Caddy (`configs/Caddyfile`) is the production entry point on port 80. Routes:

| Route | Upstream | Purpose |
|-------|----------|---------|
| `/stream/ws` | `transport-server:8080` | Binary video/telemetry WebSocket |
| `/socket.io/*` | `node:3000` | Gamepad relay WebSocket |
| `/api/*` | `node:3000` | Health/status API |
| `/ros/*` | `ros-bridge:9090` | ROS 2 rosbridge WebSocket |
| `/*` | `/srv/www` (static) | SPA bundle from `dist/` |

The `/stream/ws` route uses `flush_interval -1` and `read_timeout 0` for persistent WebSocket connections.

## How the pieces connect

```
Browser (React)
  ├── Binary WS (/stream/ws) ──→ Caddy ──→ transport-server (Rust)
  │                                           ├── SHM ←── genesis-sim (GPU)
  │                                           └── NATS ←→ genesis-sim
  ├── Socket.io ──→ Caddy ──→ node:3000 (gamepad relay only)
  └── ROSLIB WS (/ros/) ──→ Caddy ──→ ros-bridge:9090 ──→ ROS 2 topics
```

## Performance notes

- Joystick state is volatile (fire-and-forget) to avoid buffering stale control inputs.
- The node server is I/O bound (relay only); heavy compute stays in Python/CUDA/Rust.
- Video frames are zero-copy via SHM between genesis-sim and transport-server.
- NATS provides ordered, at-most-once delivery for commands and telemetry.
