# SDR_OS Architecture Overview (Genesis-in-browser goal)

## Objectives
- Low-latency teleoperation from browser (gamepad/keyboard) to robot or simulated agent.
- Run Genesis sim in-browser when possible; fall back to GPU-hosted sim with video streaming.
- Unified telemetry, control, and media paths that can swap real robot vs. simulation.
- Keep ROS2/Python control surface for robotics stack compatibility; expose web-first UX.

## Topology (current→target)
- **Browser client (React/Three/WebGPU)** ↔ **Gateway** (WebSocket/HTTP) ↔ **ROS2 control plane (Python)** ↔ **Genesis runtime (GPU) or physical robot**.
- **Telemetry**: ROS2 topics → Gateway → Browser dashboards.
- **Video**: Sim/robot camera → NVENC encode → WebRTC/HLS → Browser.
- **Training**: Genesis + rsl_rl via Python; share observation/action schema with web client for live viz.
- **Bridge architecture (client/server)**:
  - *Server*: RTX 2080 Ti host runs Genesis (CUDA) and exposes gRPC API for control/state; WebRTC media
    (SFU or direct PeerConnection) with NVENC/NVDEC for video/lidar streams.
  - *Client*: Steam Deck/Mac connects via gRPC for control/state and WebRTC for media; uses libwebrtc or bindings.
  - *ROCm*: optional sim worker behind the same gRPC interface when AMD targets are added.

## Pipelines
- **Control**: Browser gamepad → socket.io/ROSBridge JSON → `/controller/{button_states,joystick_state}` → `RobotControllerNode` → `/cmd_vel` → sim/robot.
- **Telemetry**: Sensors/IIO/IMU/GPS → ROS2 topics (`/robot/telemetry/all`, `/robot/imu`, etc.) → Gateway → `TelemetryPanel` charts.
- **Video/Rendering** (dual mode):
  - *Client-side*: Genesis WASM/WebGPU build renders in browser; Gateway only syncs controls + occasional state checkpoints.
  - *Server-side*: Genesis on GPU renders offscreen → NVENC (H.264/H.265/AV1) → WebRTC SFU (mediasoup/ion-sfu) → Browser `SimViewer`.
  - *Failover*: if WebRTC fails, reduce bitrate or fall back to MJPEG over HTTP/2; keep commands on gRPC.
- **Training loop**: rsl_rl policies → Genesis env wrapper → checkpoints → optional live policy rollout streamed to browser; uses same control schema for manual override.

## Components
- **Browser UI**: `src/client/` (React 18/Three.js, socket.io, ROSLIB). Key components: `ViewerLayer` (dual SIM/REAL viewer with PiP swap), `TrustStrip` (stage cells + mode buttons), `BlendPanel` (HIL alpha control), `SimViewer` (Genesis video), `RobotViewer` (Three.js 3D scene).
- **Gateway**: Node/Express + socket.io (`server.js`) for WS/HTTP; forwards ROSBridge + Genesis events; health endpoint (`/api/status`); graceful shutdown.
- **ROS2 Control**: `src/robot_controller/ros/ros_node.py` publishes `/cmd_vel`, consumes controller topics.
- **Sensors/Telemetry**: `iio_telemetry_publisher.py`, `gps.py`, `webcam_publisher.py` emit ROS2 topics; TF broadcaster for pose.
- **Simulation Runtime**: Genesis (GPU) with render graph; optional WASM/WebGPU build for in-browser mode; action/obs parity with rsl_rl.
- **Media Stack**: NVENC on RTX 2080 Ti; encoder service outputs WebRTC/HLS/DASH; integrates with Gateway for signaling/token auth.
- **Storage/Logs**: Structured logs (JSON) from Gateway and ROS; optional TimescaleDB/Influx for metrics.

## Deployment Targets
- **Containers/Services** (minimum viable):
  1) `web-gateway` (Node or FastAPI) :3000 WS/HTTP, WebRTC signaling.
  2) `ros-control` (Python/ROS2) : uses CycloneDDS/FastDDS.
  3) `sim-runtime` (Genesis GPU) : gRPC/ZeroMQ control port; pipes frames to encoder.
  4) `media-encoder` (CUDA/NVENC) : RTP/WebRTC SFU output.
- **Local dev**: single-machine compose; browser connects to localhost; feature flags select client-side vs server-side sim.

## Modes & Feature Flags
- `SIM_MODE=browser|server`: toggle WebGPU vs GPU-hosted Genesis.
- `MEDIA_TRANSPORT=webrtc|hls`: choose stream transport.
- `BACKEND=gateway-node|gateway-python`: pick backend implementation (see reasoning file).

## Open Questions / Risks
- Genesis WASM/WebGPU maturity and feature parity (physics fidelity, threading limits in browser).
- NVENC session limits on 2080 Ti and bandwidth budgeting for 60fps streams.
- ROSBridge latency under load vs direct socket bridge.
- Security: auth/z for signaling and ROS topic exposure.

## Performance Notes
- Prefer shared memory or Unix domain sockets when sim and bridge are co-located to reduce memcpy overhead.
- Use GPU encoders (NVENC on NVIDIA, VCE/AMF on AMD) to minimize CPU usage.
- Batch high-rate sensor updates and interpolate client-side to reduce packet overhead.
