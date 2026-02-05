# ROS Cleanup + Fixes Brainstorming Plan (2026-02-04)

**Scope**
- Create a focused cleanup plan for latent or legacy code.
- Create a focused ROS fixes plan to align telemetry with the current ROS implementation.
- Keep this as a brainstorming doc that can be turned into tasks.

**Assumptions To Confirm**
- Canonical ROS backend in current commits: `genesis_ros` + `genesis_bridge/gs_ros_bridge.py` + rosbridge + web UI.
- The JSON telemetry topic `/robot/telemetry/all` is legacy unless explicitly still required.
- The Web UI should route topics via Settings where possible, not hardcoded constants.

**Backfile / Cleanup Candidates**
- `rl/checkpoints/go2-locomotion/git/SDR_OS.diff` (18MB captured diff file, not runtime).
- `src/ros/subscription_logger.py` (ROS1 `rospy`, unused).
- `src/client/simple.html` and `src/client/simple.js` (demo artifacts, not wired to app).
- `start_robot_telemetry.sh` (legacy flow; duplicates other launch/ROS scripts).
- `src/main.py` + `src/robot_controller/ui/*` if the PyQt UI is no longer shipped.
- `gps.py` and `gps_sensor.py` if GPS is not part of the current runtime story.
- `iio_telemetry_publisher.py` if moving fully to `genesis_ros` telemetry.

**Backfile Strategy (Lightweight, Reversible)**
- Move legacy scripts into `archive/` or `docs/legacy/` with a short README.
- Keep a manifest of what was moved and why.
- Preserve runnable entrypoints only if they are part of current setup docs.

**Cleanup Task Ideas**
1. Decide canonical runtime paths and update README to match.
2. Move listed legacy files into `archive/` with a short note.
3. Update `package.json`, `setup.py`, and docs to remove stale references.
4. Delete large artifacts from repo if not required.

**ROS Fixes Brainstorm**

**Goal**
- Telemetry in UI should align with the active ROS implementation and be configurable.

**Primary Fixes**
1. Use `settings.topics.*` for all ROS topics in UI components.
2. Align IMU topic to a single source (`/imu/data` or `/robot/imu`).
3. Provide a fallback path for JSON telemetry if still needed.

**UI Code Areas To Fix**
- `src/client/components/TelemetryPanel.jsx` (subscribe topic should be configurable).
- `src/client/components/RobotViewer.jsx` (IMU and TF topics should use settings).
- `src/client/components/ControlOverlay.jsx` (button + joystick topic names should use settings).
- `src/client/components/CameraViewer.jsx` (camera topic should use settings).

**Backend/ROS Nodes To Align**
- `iio_telemetry_publisher.py` to publish IMU on `/imu/data` if retained.
- `genesis_bridge/gs_ros_bridge.py` to expose a clear telemetry topic strategy.
- `launch/franka_sdr_genesis.launch.py` to ensure consistent topic mapping and ports.

**Package/Entrypoint Fixes**
- Fix `setup.py` entrypoint to point at `src/main.py` or move `main.py` under `src/robot_controller/`.
- Add `stop_spin()` to `src/robot_controller/ros/ros_node.py` or remove calls to it.

**Distro Consistency**
- Standardize ROS distro usage across scripts and docs (Jazzy vs Humble vs Foxy).
- Update `scripts/setup_genesis_ros.sh` and `start_robot_telemetry.sh` to match.

**Verification Checklist**
- Launch flow starts without manual topic edits.
- UI telemetry updates when ROS topics are active.
- Settings topic overrides actually route subscriptions.
- Genesis sim flow and real robot flow do not conflict.

**Open Questions**
- Is the JSON telemetry path still required for production use?
- Should the real-robot pipeline remain separate or share topics with genesis_ros?
- Which ROS distro is the target for the project right now?
