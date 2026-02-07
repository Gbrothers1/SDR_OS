# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SDR_OS is a web-based robot controller for ROS2-enabled robots, optimized for Steam Deck. It provides 3D visualization, real-time control, and telemetry monitoring through a browser interface.

**Stack:** React 18 + Three.js frontend, Node.js/Express + Socket.io backend, Rust transport-server, NATS messaging, roslib for ROS2 communication, with an optional PyQt6 desktop fallback.

## Build Commands

```bash
npm install              # Install dependencies
npm run build            # Production build → dist/
npm run dev              # Development watch mode (unminified)
npm start                # Run production server (port 3000)
npm run dev:server       # Auto-restart server with nodemon
```

**Transport server (Rust):**
```bash
cargo build --manifest-path services/transport-server/Cargo.toml
cargo test --manifest-path services/transport-server/Cargo.toml
```

**Python (desktop app):**
```bash
pip install -r requirements.txt
python src/main.py
```

**ROS2 bridge (required for robot communication):**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Architecture

### Frontend (`src/client/`)
- **Entry:** `index.js` → `App.jsx`
- **State:** `contexts/SettingsContext.jsx` - React Context with localStorage persistence
- **Genesis:** `contexts/GenesisContext.jsx` - sim streaming, safety state, WS command dispatch
- **Components:** Each has paired `.jsx` and `.css` in `components/` and `styles/`
- **ROS:** ROSLIB connects to rosbridge_websocket (default `ws://localhost:9090`)
- **Socket.io:** Browser-only, used for gamepad relay between tabs (NOT for sim communication)
- **Gamepad → dual cmd_vel:** ControlOverlay publishes `geometry_msgs/Twist` on ROS `/cmd_vel` (any ROS2 robot) AND sends `set_cmd_vel` via Genesis NATS (sim safety stack) in parallel at 30Hz

### Backend (`server.js`)
- Express static server for `dist/`
- Socket.io for controller input relay only: `robot_control`, `controller_button_states`, `controller_joystick_state`
- `/stream/ws` proxy → transport-server for binary video/telemetry stream

### Transport Server (`services/transport-server/`)
- Rust: SHM frame reader → WebSocket fanout + NATS telemetry relay
- Config: All via `SDR_*` env vars (see `src/config.rs`)
- Docker: `docker compose --profile sim up transport-server`

### Binary WebSocket Protocol
Type byte prefix on every WS binary frame:
- `0x01` VIDEO — 32-byte LE header + Annex-B/JPEG payload
- `0x02` TELEMETRY — NATS subject (null-terminated) + JSON payload
- `0x03` SIGNALING — reserved for WebRTC
- `0x04` COMMAND — browser→server, JSON `{action, cmd_seq, data, ttl_ms?}`

See `src/client/utils/H264Decoder.js` for header parsing and `docs/nats-subjects.md` for the full subject schema.

### NATS Subjects
Commands (browser → transport → sim): `command.genesis.<action>`
Telemetry (sim → NATS → transport → browser): `telemetry.*`

Key subjects:
- `command.genesis.set_cmd_vel` — velocity with `cmd_seq` for ordering
- `command.genesis.settings` — codec, bitrate, preset, quality, fps, resolution
- `telemetry.training.metrics` — sim step/fps/policy state
- `telemetry.safety.state` — canonical safety mode (ARMED/HOLD/ESTOP)
- `telemetry.safety.video_gate` — transport→sim gate notifications

Full schema: `docs/nats-subjects.md`

### Safety Stack (3 Layers)
| Layer | Location | HOLD threshold | ESTOP threshold | Authority |
|-------|----------|---------------|-----------------|-----------|
| 1 | Frontend (GenesisContext) | — | 500ms video age | Cosmetic (UI overlay) |
| 2 | Transport (Rust) | 1s SHM stale | 5s SHM stale | Gate + latch zero |
| 3 | Sim (genesis_sim_runner) | 200ms cmd TTL | 2s cmd TTL | Canonical state |

- **ARMED** → normal operation
- **HOLD** → zero velocity, auto-recoverable on fresh command
- **ESTOP** → zero velocity, requires operator RE-ARM

### Genesis Sim Runner (`scripts/genesis_sim_runner.py`)
- NVENC H.264 encoder with JPEG fallback (via PyAV), runtime codec switching via `settings` command
- NATS command subscriber (`command.genesis.>`)
- Layer 3 safety: TTL decay + ESTOP on command timeout
- Publishes `telemetry.safety.state` at 2Hz (canonical authority)

### SHM Ringbuffer (`src/sdr_os/ipc/shm_ringbuffer.py`)
- 16-byte metadata + 32-byte frame header per slot
- Codec values: 1=H.264, 2=HEVC, 3=JPEG
- Writer: `ShmRingWriter`, Reader: Rust transport-server

### Python Sensor Publishers (root level)
- `iio_telemetry_publisher.py` - IMU/sensor telemetry via IIO
- `webcam_publisher.py` - Camera feed to ROS topic
- `gps.py`, `gps_sensor.py` - GPS data publishing

### Desktop App (`src/robot_controller/`)
- PyQt6 alternative to web interface
- `ros/ros_node.py` - ROS2 integration
- `ui/main_window.py` - Main window with custom widgets

## Key Patterns

### Settings Structure (SettingsContext)
Settings organized by section: `ui`, `telemetry`, `visualization`, `control`, `topics`, `voice`, `genesis`, `connection`. Access via:
```javascript
const { getSetting, updateSettings } = useSettings();
const value = getSetting('control', 'maxLinearSpeed', 1.0);
updateSettings({ control: { maxLinearSpeed: 2.0 } });
```

### ROS Topics (configured in settings)
- `/cmd_vel` - Robot velocity commands
- `/odom` - Odometry
- `/imu/data` - IMU sensor data
- `/joint_states` - Joint states
- `/tf`, `/tf_static` - Transforms
- `/webcam/image_raw` - Camera feed

### Webpack Configuration
- Entry: `src/client/index.js`
- Output: `dist/bundle.js`
- Dev server proxies `/ros` to `ws://localhost:9090`
- ROSLIB loaded as external (expects global `ROSLIB`)

### Docker Services & Networking

**Profiles:** `sim` (full stack), `dev` (web only), `train`, `obs`, `cuda`, `rocm`, `mlx`

| Service | Image | Network | Port | Purpose |
|---------|-------|---------|------|---------|
| `webserver` | caddy:2-alpine | edge+backplane | 80, 443 | Reverse proxy, static file server |
| `node` | sdr_os-node | backplane | 3000 (internal) | Socket.io gamepad relay, /api |
| `transport-server` | sdr_os-transport | edge+backplane | 8080 | SHM→WS fanout, NATS relay, video gate |
| `genesis-sim` | sdr_os-cuda | host | — | Genesis simulation, NVENC encoder, SHM writer |
| `nats` | nats:2-alpine | backplane | 4222, 8222 | Message broker |
| `ros-bridge` | sdr_os-ros-jazzy | host | 9090 | ROS2 rosbridge WebSocket |

**Caddy routes** (production entry point on `:80`):
- `/stream/ws` → `transport-server:8080` (binary video/telemetry WS)
- `/socket.io/*` → `node:3000` (gamepad relay)
- `/api/*` → `node:3000` (health/status)
- `/ros/*` → `ros-bridge:9090` (rosbridge WS)
- `/*` → `/srv/www` (static bundle from `dist/`)

## Running

### Production (Docker Compose + Caddy)
```bash
npm run build                                    # Build frontend → dist/
docker compose --profile sim up --build -d       # Build + start all services
# Access at http://<host-ip>:80
```

### Development (bare metal Node)
```bash
npm run build                                    # Build frontend → dist/
node server.js                                   # Start on :3000 (proxies /stream/ws)
# Access at http://localhost:3000
```

### Rebuilding individual services
```bash
docker compose build transport-server            # After Rust code changes
docker compose build node                        # After server.js changes
docker compose build genesis-sim                 # After Dockerfile/dependency changes
docker compose up -d <service>                   # Recreate with new image
```

## Development Notes

- rosbridge_server must be running for ROS communication
- Default rosbridge URL configurable via localStorage key `rosBridgeUrl`
- Controller state synced across multiple browser clients via Socket.io (gamepad only)
- Three.js used for 3D robot model rendering in `RobotViewer.jsx`
- Genesis sim communication is via NATS (not Socket.io)
- Transport server binary WS at `/stream/ws` (proxied through node server or Caddy)
- Genesis-sim uses `network_mode: host` in dev — revert to `[backplane]` for release
