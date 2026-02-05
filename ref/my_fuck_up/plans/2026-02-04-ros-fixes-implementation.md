# ROS Fixes Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Wire all frontend ROS subscribers to use `settings.topics.*` with reactive re-subscription, rework TelemetryPanel to use standard ROS2 message types, and fix two backend issues.

**Architecture:** Each of 4 UI components gets `useSettings()` hook access and subscribes via `useEffect` with topic names in the dependency array. TelemetryPanel is reworked to parse `sensor_msgs/Imu` and `sensor_msgs/JointState` instead of a custom JSON blob. Two Python backend fixes are standalone.

**Tech Stack:** React 18, ROSLIB (CDN global), Three.js, rclpy, setuptools

---

### Task 1: Add `stop_spin()` to RobotControllerNode

**Files:**
- Modify: `src/robot_controller/ros/ros_node.py:161-172`

**Step 1: Add stop_spin method after start_spin**

In `src/robot_controller/ros/ros_node.py`, add `stop_spin()` between `start_spin()` (line 161) and `_spin()` (line 167):

```python
def stop_spin(self):
    """Stop the ROS2 spin thread and shut down."""
    self.destroy_node()
```

Note: `src/main.py:33` already calls `rclpy.shutdown()` after `stop_spin()`, so we only need `destroy_node()` here to avoid double-shutdown.

**Step 2: Verify the fix is consistent with src/main.py**

Read `src/main.py` lines 31-33 to confirm the call sequence:
```python
ros_node.stop_spin()   # <- our new method: calls destroy_node()
rclpy.shutdown()       # <- already exists in main.py
```

This matches standard rclpy lifecycle: destroy node, then shutdown.

**Step 3: Commit**

```bash
git add src/robot_controller/ros/ros_node.py
git commit -m "fix: add missing stop_spin() method to RobotControllerNode"
```

---

### Task 2: Fix setup.py entrypoint

**Files:**
- Modify: `setup.py:22-25`

**Step 1: Fix the entrypoint**

The package uses `find_packages(where='src')` and `package_dir={'': 'src'}`, so the installed package is `robot_controller`. The main function is in `src/main.py` which is at the repo root's `src/` directory but NOT inside the `robot_controller` package directory.

Since `src/main.py` imports `from robot_controller.ui.main_window import MainWindow` and `from robot_controller.ros.ros_node import RobotControllerNode`, the entry module needs to be part of the package tree. The actual main lives at `src/main.py` which maps to the top-level `main` module under `src/`.

However, `find_packages(where='src')` only finds `robot_controller` and its subpackages. `main.py` is a standalone file in `src/`, not inside any package.

**Fix:** Create `src/robot_controller/main.py` that re-exports the main function:

```python
"""Entry point for robot_controller package."""
# Re-export main from the top-level src/main.py
# This makes the setup.py console_scripts entry work:
#   robot_controller = robot_controller.main:main
import sys
import os

# Add src directory to path so we can import the top-level main
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from main import main  # noqa: E402

__all__ = ['main']
```

Wait -- that's fragile. Simpler approach: just move the main() logic into `src/robot_controller/main.py` directly and have `src/main.py` import from it. But that's more churn than needed.

**Simplest fix:** Update setup.py to not have a broken entrypoint. Since the web app is the primary interface (started via `npm start`), and the PyQt desktop app is legacy, just comment the entrypoint with a note:

Actually, let's just make it work. Create `src/robot_controller/main.py`:

```python
"""Console entry point for robot_controller ROS2 package."""
from main import main

if __name__ == '__main__':
    main()
```

This works because `setup.py` sets `package_dir={'': 'src'}`, so at install time `src/` is the root, meaning `from main import main` resolves to `src/main.py`.

**Step 2: Create the file**

Create `src/robot_controller/main.py`:
```python
"""Console entry point for robot_controller ROS2 package."""
from main import main

if __name__ == '__main__':
    main()
```

**Step 3: Commit**

```bash
git add src/robot_controller/main.py
git commit -m "fix: add robot_controller.main entry point module for setup.py"
```

---

### Task 3: Wire RobotViewer to settings.topics

**Files:**
- Modify: `src/client/components/RobotViewer.jsx:1-478`

**Context:** RobotViewer receives `settings` as a prop (the full appSettings object from ViewerLayer). It does NOT currently use `useSettings()`. The `settings` prop contains `settings.topics.imu`, `settings.topics.joint_states`, etc. However, the prop-based approach won't trigger reactive re-subscription because the ROS subscription is inside a `useEffect([ros, tfThrottleRate])` that doesn't include topic names.

**Approach:** Import `useSettings` and read topic names directly from the hook. Add topic names to the `useEffect` dependency array. Restructure the ROS subscription block to support cleanup.

**Step 1: Add useSettings import**

At line 1, add to the imports:
```jsx
import { useSettings } from '../contexts/SettingsContext';
```

**Step 2: Add settings hook and extract topic names**

Inside the component function (after line 6), add:
```jsx
const { settings: appSettings } = useSettings();
const imuTopicName = appSettings?.topics?.imu || '/imu/data';
const jointStatesTopicName = appSettings?.topics?.joint_states || '/joint_states';
const tfTopicName = appSettings?.topics?.tf || '/tf';
const tfStaticTopicName = appSettings?.topics?.tf_static || '/tf_static';
```

**Step 3: Move ROS subscriptions to a separate useEffect**

Currently, ROS subscriptions are inside the large Three.js init `useEffect` (lines 56-478). This is a problem because re-subscribing would re-initialize the entire 3D scene. Extract the ROS subscription block (lines 410-463) into its own `useEffect`:

Remove lines 409-463 from the existing useEffect. Then add a new useEffect after the existing one:

```jsx
// ROS Subscriptions - reactive to topic settings changes
useEffect(() => {
  if (!ros) return;

  const throttleIntervalMs = tfThrottleRate > 0 ? Math.round(1000 / tfThrottleRate) : 0;

  const imuTopic = new ROSLIB.Topic({
    ros: ros,
    name: imuTopicName,
    messageType: 'sensor_msgs/Imu'
  });

  imuTopic.subscribe((message) => {
    setImuData(message);
  });

  const jointStatesTopic = new ROSLIB.Topic({
    ros: ros,
    name: jointStatesTopicName,
    messageType: 'sensor_msgs/JointState'
  });

  jointStatesTopic.subscribe((message) => {
    // Handle joint state messages
  });

  const tfTopic = new ROSLIB.Topic({
    ros: ros,
    name: tfTopicName,
    messageType: 'tf2_msgs/msg/TFMessage',
    throttle_rate: throttleIntervalMs
  });

  tfTopic.subscribe((message) => {
    // Process TF messages
  });

  const tfStaticTopic = new ROSLIB.Topic({
    ros: ros,
    name: tfStaticTopicName,
    messageType: 'tf2_msgs/msg/TFMessage',
    throttle_rate: throttleIntervalMs
  });

  tfStaticTopic.subscribe((message) => {
    // Process TF static messages
  });

  return () => {
    imuTopic.unsubscribe();
    jointStatesTopic.unsubscribe();
    tfTopic.unsubscribe();
    tfStaticTopic.unsubscribe();
  };
}, [ros, tfThrottleRate, imuTopicName, jointStatesTopicName, tfTopicName, tfStaticTopicName]);
```

**Step 4: Update the original useEffect deps**

The original useEffect (line 478) has `[ros, tfThrottleRate]`. Since we removed the ROS subscription code from it, it only needs `[ros]` for Three.js init. But keep `tfThrottleRate` if it affects anything else there (it doesn't after extraction). Change to `[ros]`.

Actually, be careful -- the entire init (scene, model, animation, resize observer) is in this effect. It should only run once per `ros` change. After removing the ROS subscription block, update the dependency:
```jsx
}, [ros]);
```

**Step 5: Commit**

```bash
git add src/client/components/RobotViewer.jsx
git commit -m "feat: wire RobotViewer ROS topics to settings with reactive re-subscription"
```

---

### Task 4: Wire ControlOverlay to settings.topics

**Files:**
- Modify: `src/client/components/ControlOverlay.jsx:182-341`

**Context:** ControlOverlay already imports `useSettings` (line 5) and uses it for UI settings. The ROS topic publishers are created inside the gamepad polling `useEffect` (lines 183-341) with hardcoded topic names at lines 190 and 196.

**Step 1: Extract topic names from settings**

Near the top of the component (after line 10), add:
```jsx
const buttonStatesTopic = getSetting('topics', 'button_states', '/controller/button_states');
const joystickStateTopic = getSetting('topics', 'joystick_state', '/controller/joystick_state');
```

Note: `getSetting` is already destructured at line 10.

**Step 2: Replace hardcoded topic names**

In the gamepad polling useEffect (line 188), replace:
```jsx
name: '/controller/button_states',
```
with:
```jsx
name: buttonStatesTopic,
```

At line 196, replace:
```jsx
name: '/controller/joystick_state',
```
with:
```jsx
name: joystickStateTopic,
```

**Step 3: Add topic names to useEffect dependency array**

At line 341, the dependency array is:
```jsx
}, [onControlChange, ros, socket, isGamepadConnected, buttonStates]);
```

Add the topic names:
```jsx
}, [onControlChange, ros, socket, isGamepadConnected, buttonStates, buttonStatesTopic, joystickStateTopic]);
```

**Step 4: Commit**

```bash
git add src/client/components/ControlOverlay.jsx
git commit -m "feat: wire ControlOverlay ROS publishers to settings.topics"
```

---

### Task 5: Wire CameraViewer to settings.topics

**Files:**
- Modify: `src/client/components/CameraViewer.jsx:1-358`

**Context:** CameraViewer receives `topic` as a prop and uses it to initialize `selectedCamera` state (line 16). The `availableCameras` list (line 22) hardcodes `/webcam/image_raw`. The component doesn't use `useSettings`.

**Step 1: Add useSettings import**

At line 1, add to imports:
```jsx
import { useSettings } from '../contexts/SettingsContext';
```

**Step 2: Use settings for default topic**

Inside the component, after the memo wrapper opens (line 6), add:
```jsx
const { settings: appSettings } = useSettings();
const cameraTopic = appSettings?.topics?.camera || '/webcam/image_raw';
```

**Step 3: Use settings-driven default for selectedCamera**

Change line 16 from:
```jsx
const [selectedCamera, setSelectedCamera] = useState(topic);
```
to:
```jsx
const [selectedCamera, setSelectedCamera] = useState(topic || cameraTopic);
```

**Step 4: Update availableCameras to use settings**

Change line 22-24 from:
```jsx
const availableCameras = [
  { id: '/webcam/image_raw', name: 'Front Camera' },
];
```
to:
```jsx
const availableCameras = [
  { id: cameraTopic, name: 'Front Camera' },
];
```

**Step 5: Add effect to sync selectedCamera when settings change**

Add after the availableCameras declaration:
```jsx
useEffect(() => {
  if (!topic) {
    setSelectedCamera(cameraTopic);
  }
}, [cameraTopic, topic]);
```

This re-subscribes when the camera topic changes in settings, but only if no explicit `topic` prop was passed.

**Step 6: Commit**

```bash
git add src/client/components/CameraViewer.jsx
git commit -m "feat: wire CameraViewer to settings.topics.camera with reactive updates"
```

---

### Task 6: Rework TelemetryPanel to use standard ROS2 topics

**Files:**
- Modify: `src/client/components/TelemetryPanel.jsx:1-450`

**Context:** This is the largest change. TelemetryPanel subscribes to `/robot/telemetry/all` (std_msgs/String with JSON payload from iio_telemetry_publisher) and expects a custom `{ imu: { accel: {x,y,z}, gyro: {x,y,z}, orientation: {roll,pitch,yaw} }, devices: {...} }` shape. We're switching to standard `sensor_msgs/Imu` and `sensor_msgs/JointState` as primary, keeping the JSON blob as fallback.

**Step 1: Add useSettings import**

Add to line 1 imports:
```jsx
import { useSettings } from '../contexts/SettingsContext';
```

**Step 2: Add settings hook inside component**

After line 35 (component function declaration), add:
```jsx
const { settings: appSettings } = useSettings();
const imuTopicName = appSettings?.topics?.imu || '/imu/data';
const jointStatesTopicName = appSettings?.topics?.joint_states || '/joint_states';
```

**Step 3: Add state for standard topic data**

After the existing `telemetryData` state (line 36), add:
```jsx
const [imuData, setImuData] = useState(null);
const [jointData, setJointData] = useState(null);
const [hasStandardTopics, setHasStandardTopics] = useState(false);
```

**Step 4: Add standard IMU subscription**

Replace the existing subscription useEffect (lines 138-172) with TWO new effects. First, the standard topics effect:

```jsx
// Subscribe to standard ROS2 IMU topic
useEffect(() => {
  if (!ros) return;

  const topic = new ROSLIB.Topic({
    ros: ros,
    name: imuTopicName,
    messageType: 'sensor_msgs/Imu'
  });

  const handleImu = (message) => {
    if (!showPanel) return;
    setHasStandardTopics(true);

    const data = {
      imu: {
        accel: {
          x: message.linear_acceleration.x,
          y: message.linear_acceleration.y,
          z: message.linear_acceleration.z
        },
        gyro: {
          x: message.angular_velocity.x,
          y: message.angular_velocity.y,
          z: message.angular_velocity.z
        },
        orientation: {
          // Convert quaternion to euler for display
          roll: Math.atan2(
            2 * (message.orientation.w * message.orientation.x + message.orientation.y * message.orientation.z),
            1 - 2 * (message.orientation.x * message.orientation.x + message.orientation.y * message.orientation.y)
          ),
          pitch: Math.asin(
            Math.max(-1, Math.min(1, 2 * (message.orientation.w * message.orientation.y - message.orientation.z * message.orientation.x)))
          ),
          yaw: Math.atan2(
            2 * (message.orientation.w * message.orientation.z + message.orientation.x * message.orientation.y),
            1 - 2 * (message.orientation.y * message.orientation.y + message.orientation.z * message.orientation.z)
          )
        }
      }
    };

    throttledUpdateState(data);
    setImuData(data);
  };

  topic.subscribe(handleImu);
  console.log(`TelemetryPanel subscribed to standard IMU: ${imuTopicName}`);

  return () => {
    topic.unsubscribe();
    console.log(`TelemetryPanel unsubscribed from IMU: ${imuTopicName}`);
  };
}, [ros, imuTopicName, showPanel, throttledUpdateState]);
```

**Step 5: Add standard JointState subscription**

```jsx
// Subscribe to standard ROS2 JointState topic
useEffect(() => {
  if (!ros) return;

  const topic = new ROSLIB.Topic({
    ros: ros,
    name: jointStatesTopicName,
    messageType: 'sensor_msgs/JointState'
  });

  const handleJointState = (message) => {
    if (!showPanel) return;
    setHasStandardTopics(true);
    setJointData({
      names: message.name,
      positions: message.position,
      velocities: message.velocity,
      efforts: message.effort
    });
  };

  topic.subscribe(handleJointState);

  return () => {
    topic.unsubscribe();
  };
}, [ros, jointStatesTopicName, showPanel]);
```

**Step 6: Keep JSON fallback subscription**

```jsx
// Fallback: Subscribe to legacy JSON telemetry topic (for iio testing)
useEffect(() => {
  if (!ros) return;

  const telemetryTopic = new ROSLIB.Topic({
    ros: ros,
    name: '/robot/telemetry/all',
    messageType: 'std_msgs/String'
  });

  telemetryTopic.subscribe((message) => {
    if (!showPanel) return;
    // Only use JSON fallback if standard topics aren't providing data
    if (hasStandardTopics) return;
    try {
      const data = JSON.parse(message.data);
      throttledUpdateState(data);
    } catch (err) {
      console.error('Error parsing JSON telemetry:', err);
    }
  });

  return () => {
    telemetryTopic.unsubscribe();
  };
}, [ros, showPanel, hasStandardTopics, throttledUpdateState]);
```

**Step 7: Update the render to show joint data when available**

After the existing IMU section in the render (after line 428), add a joint states section:

```jsx
{jointData && (
  <div className="telemetry-section">
    <h4>Joint States</h4>
    <div className="telemetry-values">
      {jointData.names && jointData.names.map((name, i) => (
        <div key={name} className="value-row">
          <span>{name}:</span>
          <span>{jointData.positions?.[i]?.toFixed(4) ?? 'N/A'} rad</span>
        </div>
      ))}
    </div>
  </div>
)}
```

**Step 8: Update the "Available Sensors" section to be conditional**

The `telemetryData.devices` section (lines 431-439) only exists in the JSON blob format. Wrap it:

```jsx
{telemetryData?.devices && (
  <div className="telemetry-section">
    <h4>Available Sensors</h4>
    <div className="sensors-list">
      {Object.entries(telemetryData.devices).map(([name, device]) => (
        <div key={name} className="sensor-item">
          <span className="sensor-name">{name}</span>
          <span className="sensor-description">{device.description}</span>
        </div>
      ))}
    </div>
  </div>
)}
```

**Step 9: Update the "no data" condition**

Change the condition at line 374 from:
```jsx
{!telemetryData && !error && (
```
to:
```jsx
{!telemetryData && !imuData && !jointData && !error && (
```

And change the data display condition at line 378 from:
```jsx
{telemetryData && (
```
to:
```jsx
{(telemetryData || imuData || jointData) && (
```

**Step 10: Remove the stableMessageHandler and old subscription refs**

Delete or leave the `messageHandlerRef`, `stableMessageHandler`, and old subscription `useEffect` (lines 112-172) since they're replaced by the three new subscription effects.

**Step 11: Commit**

```bash
git add src/client/components/TelemetryPanel.jsx
git commit -m "feat: rework TelemetryPanel to use standard ROS2 IMU and JointState topics

Primary data from settings.topics.imu (sensor_msgs/Imu) and
settings.topics.joint_states (sensor_msgs/JointState).
Falls back to /robot/telemetry/all JSON blob for iio testing."
```

---

### Task 7: Build and verify

**Step 1: Run webpack build**

```bash
cd /home/ethan/dev/Genesis/SDR_OS && npm run build
```

Expected: Build succeeds with no errors. Warnings about unused vars are OK but note them.

**Step 2: Fix any build errors**

If webpack reports errors (missing imports, syntax issues), fix them and rebuild.

**Step 3: Verify the dev server starts**

```bash
cd /home/ethan/dev/Genesis/SDR_OS && timeout 5 npm start || true
```

Expected: Server starts on port 3000 without crash.

**Step 4: Final commit if any fixes were needed**

```bash
git add -A
git commit -m "fix: address build issues from ROS topic wiring"
```
