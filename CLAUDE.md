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
- **State:** `contexts/SettingsContext.jsx` - React Context with localStorage persistence; `contexts/GenesisContext.jsx` for sim bridge; `contexts/PhaseContext.jsx` for derived stage health/authority
- **Components:** Each has paired `.jsx` and `.css` in `components/` and `styles/`
- **Viewers:** `RobotViewer.jsx` (Three.js 3D scene for ROS), `SimViewer.jsx` (Genesis sim video feed)
- **Trust Strip:** Top bar with stage cells (TELEOP/TRAIN/EVAL) on left, mode buttons (TELEM/POLICY/BLEND/LAB/E-STOP) on right; connection indicator toggles SIM/REAL viewer
- **ROS:** ROSLIB connects to rosbridge_websocket (default `ws://localhost:9090`)
- **Real-time:** Socket.io for controller state broadcasting between clients

### Backend (`server.js`)
- Express static server for `dist/`
- Socket.io for controller input relay (broadcasts `robot_control`, `controller_button_states`, `controller_joystick_state`)

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
- Three.js used for 3D robot model rendering in `RobotViewer.jsx`; sim feed rendered in `SimViewer.jsx`
- Test mode (`localStorage sdr_test_mode`) enables UI development without ROS/Genesis connections
- User-facing text uses "Sim" (not "Genesis"); internal code keeps `genesis*` naming
- CSS design system in `styles/theme.css` — use `var(--color-*)`, `var(--transition-*)`, `var(--z-*)` tokens
- Viewer source (SIM/REAL) state is lifted to `App.jsx` and shared between `ViewerLayer` and `TrustStrip`
