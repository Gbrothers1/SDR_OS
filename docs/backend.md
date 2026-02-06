# Backend

The backend is a Node.js/Express server (`server.js`) that serves the web app and acts as the real-time relay between browser clients and the Genesis simulation bridge.

## Server overview

| Property | Value |
|----------|-------|
| Runtime | Node.js + Express |
| Port | 3000 (configurable via `PORT` env var) |
| Host | 0.0.0.0 (configurable via `HOST` env var) |
| Real-time | Socket.io |
| Static files | `dist/` (Webpack output), `configs/`, `assets/` |

```bash
pnpm start          # production: node server.js
pnpm run dev:server  # development: nodemon server.js (auto-restart)
```

## Static file serving

| Route | Directory | Purpose |
|-------|-----------|---------|
| `/` | `dist/` | Webpack-built SPA (bundle.js, index.html) |
| `/configs/*` | `configs/` | URDF registry, robot YAML configs |
| `/assets/*` | `assets/` | URDF files, 3D models (DAE, OBJ, MTL) |

Development headers disable caching for `.js` and `.css` files.

## Socket.io events

### Controller events (gamepad relay)

These events are broadcast from any browser client to all connected clients (including the Genesis bridge).

| Event | Payload | Transport |
|-------|---------|-----------|
| `robot_control` | Control command object | Reliable (default) |
| `controller_button_states` | Button state map (e.g. `{L1: true, R1: false, ...}`) | Reliable |
| `controller_joystick_state` | Joystick axes object | **Volatile** (drops if behind) |
| `controller_mapping_type` | `{type: string}` | Reliable |
| `controller_vibration` | Haptic feedback payload | Reliable |

The joystick state uses `io.volatile.emit` to drop stale frames rather than queue them -- important for low-latency control.

### Genesis bridge protocol

The Genesis bridge (Python, port 9091) connects as a Socket.io client and identifies itself:

1. Bridge sends `genesis_identify` with metadata.
2. Server marks the socket as bridge, broadcasts `genesis_status: {connected: true}` to all clients.
3. On disconnect, server broadcasts `genesis_status: {connected: false}`.

### Genesis events (40+ forwarded)

The server forwards these events between browser clients and the Genesis bridge using `socket.broadcast.emit`:

**Simulation control:**
`genesis_load_robot`, `genesis_unload_robot`, `genesis_reset`, `genesis_pause`, `genesis_set_mode`, `genesis_select_env`, `genesis_camera`, `genesis_set_goal`, `genesis_set_alpha`, `genesis_set_control_mode`, `genesis_velocity_command`, `genesis_estop`, `genesis_set_command_source`, `genesis_skill_command`, `genesis_mapping_type`

**Status and info:**
`genesis_status`, `genesis_env_info`, `genesis_robot_list`, `genesis_robot_info`, `genesis_robot_loaded`, `genesis_robot_unloaded`, `genesis_robot_load_failed`, `genesis_memory_estimate`, `genesis_init_status`, `genesis_get_memory_estimate`, `genesis_get_init_status`, `genesis_get_robot_info`, `genesis_scan_robots`

**Training and metrics:**
`genesis_training_metrics`, `genesis_obs_breakdown`, `genesis_reward_breakdown`, `genesis_frame_stats`, `genesis_load_policy`, `genesis_list_policies`, `genesis_policy_list`, `genesis_policy_load_status`

**Camera:**
`genesis_get_camera_list`, `genesis_camera_list`

**Scripts and profiles:**
`genesis_load_script`, `genesis_script_status`, `load_profile`, `genesis_settings`

### Logging

The server logs:

- Client connect/disconnect with socket ID and IP
- HTTP requests (method + URL)
- Button payload shape (once per socket, for debugging)
- Button/joystick ticks (throttled to 1/sec per socket)
- L1 (deadman switch) state changes
- Genesis bridge connect/disconnect
- All Genesis event names with source client ID

## Connection tracking

The server tracks:

- `connectedClients` -- count of active Socket.io connections
- `genesisBridgeConnected` -- boolean, whether the Genesis bridge is connected

## How the pieces connect

```
Browser (React)                              Genesis bridge (Python)
  |                                              |
  |-- Socket.io -----> server.js (port 3000) <---|-- Socket.io
  |                      |                       |
  |                      +-- broadcasts to all --+
  |                                              |
  |-- ROSLIB WS -----> rosbridge (port 9090) --> ROS 2 topics
  |                                              |
  |-- Genesis WS ----> Genesis sim (port 9091) --+
```

## Performance notes

- Joystick state is volatile (fire-and-forget) to avoid buffering stale control inputs.
- Button payloads are logged once per socket to avoid log spam.
- The server is I/O bound (relay only); heavy compute stays in Python/CUDA.
