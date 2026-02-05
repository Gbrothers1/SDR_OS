# ROS Fixes Design (2026-02-04)

## Goal

Make the web UI's ROS telemetry work with standard ROS2 topics, driven by configurable `settings.topics` values rather than hardcoded strings. Components reactively re-subscribe when settings change.

## Decisions

- **Canonical IMU topic:** `/imu/data` (standard ROS2 convention)
- **TelemetryPanel primary source:** Standard ROS2 message types (sensor_msgs/Imu, sensor_msgs/JointState), with fallback to `/robot/telemetry/all` JSON blob for iio testing
- **Subscription wiring:** Reactive -- components re-subscribe automatically when topic settings change
- **Backend fixes included:** `stop_spin()` missing method, `setup.py` broken entrypoint

## Reactive Subscription Pattern

Each component uses `useEffect` with `settings.topics.<name>` in the dependency array. Cleanup function unsubscribes the old topic. No new abstractions -- each component wires its own topics directly.

```jsx
const { settings } = useSettings();
const imuTopic = settings.topics.imu;

useEffect(() => {
  if (!ros || !ros.isConnected) return;

  const topic = new ROSLIB.Topic({
    ros,
    name: imuTopic,
    messageType: 'sensor_msgs/Imu'
  });
  topic.subscribe(handleImuMessage);

  return () => topic.unsubscribe();
}, [ros, imuTopic]);
```

Existing `useCallback` wrappers for message handlers stay as-is (prevent re-subscription on handler identity changes).

## TelemetryPanel Rework

Biggest change. Replace single JSON blob subscription with standard topics.

**Primary subscriptions:**
- `settings.topics.imu` (sensor_msgs/Imu) -- orientation quaternion, angular velocity, linear acceleration
- `settings.topics.joint_states` (sensor_msgs/JointState) -- joint names, positions, velocities, efforts

**Fallback subscription:**
- `/robot/telemetry/all` (std_msgs/String with JSON payload) -- parse and merge into display state when active. Does not overwrite data from standard topics.

**Data merging:** Component state holds a flat telemetry object. Each subscription callback updates its slice via `setTelemetry(prev => ({ ...prev, ... }))`. Display adapts to show whatever fields are populated.

## Backend Fixes

### stop_spin() in ros_node.py

`src/main.py:32` calls `ros_node.stop_spin()` but the method doesn't exist. Fix:

```python
def stop_spin(self):
    """Stop the ROS2 spin thread and shut down."""
    self.destroy_node()
    rclpy.shutdown()
```

### setup.py entrypoint

Currently points to `robot_controller.main:main` which doesn't exist. Fix to match actual module path.

## File Changes

| File | Change |
|------|--------|
| `src/client/components/TelemetryPanel.jsx` | Replace JSON blob subscription with standard IMU + JointState topics from settings; add JSON fallback; reactive re-subscription |
| `src/client/components/RobotViewer.jsx` | Replace hardcoded `/robot/imu`, `/joint_states`, `/tf_static` with `settings.topics.*`; reactive re-subscription |
| `src/client/components/ControlOverlay.jsx` | Replace hardcoded button/joystick topic names with `settings.topics.*`; reactive re-subscription |
| `src/client/components/CameraViewer.jsx` | Replace hardcoded `/webcam/image_raw` with `settings.topics.camera`; reactive re-subscription |
| `src/robot_controller/ros/ros_node.py` | Add `stop_spin()` method |
| `setup.py` | Fix entrypoint to match actual module path |

**Not modified:** SettingsContext.jsx (defaults already correct), iio_telemetry_publisher.py, Genesis bridge, RL pipeline.

## Verification

- UI telemetry updates when standard ROS2 topics are active
- Changing a topic in Settings causes live re-subscription (no refresh needed)
- JSON fallback works when iio_telemetry_publisher is running
- Genesis sim flow unaffected
