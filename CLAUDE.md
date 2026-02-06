# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SDR_OS is a web-based robot controller for ROS2-enabled robots, optimized for Steam Deck. It provides 3D visualization, real-time control, and telemetry monitoring through a browser interface.

**Stack:** React 18 + Three.js frontend, Node.js/Express + Socket.io backend, roslib for ROS2 communication, with an optional PyQt6 desktop fallback.

## Build Commands

```bash
npm install              # Install dependencies
npm run build            # Production build → dist/
npm run dev              # Development watch mode (unminified)
npm start                # Run production server (port 3000)
npm run dev:server       # Auto-restart server with nodemon
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
- **Components:** Each has paired `.jsx` and `.css` in `components/` and `styles/`
- **ROS:** ROSLIB connects to rosbridge_websocket (default `ws://localhost:9090`)
- **Real-time:** Socket.io for controller state broadcasting between clients

### Backend (`server.js`)
- Express static server for `dist/`
- Socket.io for controller input relay (broadcasts `robot_control`, `controller_button_states`, `controller_joystick_state`)

### Python Sensor Publishers (root level)
- `iio_telemetry_publisher.py` - IMU/sensor telemetry via IIO
- `webcam_publisher.py` - Camera feed to ROS topic
- `gps.py`, `gps_sensor.py` - GPS data publishing

### Transport Server (`services/transport-server/`)
- Rust: SHM frame reader → WebSocket fanout + NATS telemetry relay
- Build: `cargo build --manifest-path services/transport-server/Cargo.toml`
- Test: `cargo test --manifest-path services/transport-server/Cargo.toml`
- Config: All via `SDR_*` env vars (see `src/config.rs`)
- Binary WS protocol: `0x01` = video frame, `0x02` = telemetry
- Docker: `docker compose --profile sim up transport-server`

### Desktop App (`src/robot_controller/`)
- PyQt6 alternative to web interface
- `ros/ros_node.py` - ROS2 integration
- `ui/main_window.py` - Main window with custom widgets

## Key Patterns

### Settings Structure (SettingsContext)
Settings organized by section: `ui`, `telemetry`, `visualization`, `control`, `topics`, `voice`. Access via:
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
- Dev server proxies rosbridge websocket to `ws://192.168.12.147:9090`
- ROSLIB loaded as external (expects global `ROSLIB`)

## Development Notes

- rosbridge_server must be running for ROS communication
- Default rosbridge URL configurable via localStorage key `rosBridgeUrl`
- Controller state synced across multiple browser clients via Socket.io
- Three.js used for 3D robot model rendering in `RobotViewer.jsx`
