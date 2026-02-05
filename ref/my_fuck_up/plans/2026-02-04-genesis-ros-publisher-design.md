# Genesis ROS Publisher Design

## Problem

The TelemetryPanel subscribes to ROS2 topics via roslib/rosbridge and shows "Waiting for telemetry data..." in Genesis sim mode because no ROS topics carry sim state. The Genesis bridge sends rich robot data (12 DOF positions/velocities, angular velocity, projected gravity, velocity commands, rewards, blend status) but it only reaches the SimTelemetryPane via WebSocket/Socket.io — never ROS2.

## Solution

A standalone ROS2 node (`genesis_bridge/genesis_ros_publisher.py`) that:
1. Connects to the Genesis bridge WebSocket (`ws://localhost:9091`) as a client
2. Parses incoming JSON training metrics with `obs_breakdown`
3. Publishes standard ROS2 messages so the existing TelemetryPanel works unmodified

Runs inside the ROS Jazzy Docker container with `--network=host`.

## Architecture

```
Genesis Sim
    |
bridge_server.py (WebSocket :9091)
    | JSON metrics + obs_breakdown
genesis_ros_publisher.py (ROS2 node, Jazzy Docker, --network=host)
    | publishes standard + custom topics
rosbridge_server (WebSocket :9090)
    |
TelemetryPanel (browser, existing, unmodified)
```

No changes to bridge_server.py, TelemetryPanel, or any frontend code.

### Relationship to Existing Code

- **`ros_bridge_node.py`** — Embedded in bridge_server, publishes `/cmd_vel` and `/go2_debug`, subscribes to `/cmd_vel`. Runs inside the bridge process. Different purpose.
- **`gs_ros_bridge.py`** — Subscribes TO ROS topics from a genesis_ros sim. Opposite direction.
- **New `genesis_ros_publisher.py`** — Standalone external process, publishes FROM sim state TO ROS. No overlap.

## Config File

`configs/genesis_ros_publisher.yaml`:

```yaml
bridge:
  url: "ws://localhost:9091"
  reconnect_interval: 2.0  # seconds between reconnect attempts

rates:
  physics: 10.0    # Hz - joint_states, imu, tf
  command: 10.0    # Hz - cmd_vel
  status: 5.0      # Hz - genesis/status, genesis/rewards

topics:
  joint_states: "/joint_states"
  imu: "/imu/data"
  cmd_vel: "/cmd_vel"
  tf: "/tf"
  rewards: "/genesis/rewards"
  status: "/genesis/status"
  blend: "/genesis/blend"

enable:
  joint_states: true
  imu: true
  cmd_vel: true
  tf: true
  rewards: true
  status: true
  blend: true

robot:
  name: "go2"
  joint_names:
    - "FL_hip_joint"
    - "FL_thigh_joint"
    - "FL_calf_joint"
    - "FR_hip_joint"
    - "FR_thigh_joint"
    - "FR_calf_joint"
    - "RL_hip_joint"
    - "RL_thigh_joint"
    - "RL_calf_joint"
    - "RR_hip_joint"
    - "RR_thigh_joint"
    - "RR_calf_joint"
  base_frame: "base_link"
  odom_frame: "odom"
```

## Topic Mapping

### sensor_msgs/JointState on `/joint_states`

| Field | Source |
|-------|--------|
| `name` | `robot.joint_names` from config (12 names) |
| `position` | `obs_breakdown.dof_position` (12 floats, radians) |
| `velocity` | `obs_breakdown.dof_velocity` (12 floats, rad/s) |
| `effort` | empty (not in obs_breakdown) |

### sensor_msgs/Imu on `/imu/data`

| Field | Source |
|-------|--------|
| `angular_velocity` | `obs_breakdown.angular_velocity` (3 floats, rad/s) |
| `linear_acceleration` | `obs_breakdown.projected_gravity` (gravity in body frame) |
| `orientation` | quaternion derived from projected gravity (roll/pitch; yaw=0) |

### geometry_msgs/Twist on `/cmd_vel`

| Field | Source |
|-------|--------|
| `linear.x` | `velocity_command.lin_vel_x` |
| `linear.y` | `velocity_command.lin_vel_y` |
| `angular.z` | `velocity_command.ang_vel_z` |

### tf2_msgs/TFMessage on `/tf`

Single transform: `odom_frame` -> `base_frame` with orientation from gravity-derived quaternion, position zeroed.

### Custom Topics (std_msgs/String, JSON payload)

| Topic | Content |
|-------|---------|
| `/genesis/rewards` | reward_breakdown dict |
| `/genesis/status` | blend_alpha, deadman_active, actor_tag, mode, policy_loaded, command_source |
| `/genesis/blend` | blend_alpha, confidence, safety_flags |

## Node Implementation

```
class GenesisRosPublisher(Node):
    __init__():
        - Load YAML config
        - Create publishers (one per enabled topic)
        - Create timers at configured rates (physics_timer, command_timer, status_timer)
        - Start WebSocket listener in a background thread

    ws_listener():
        - Connect to bridge WebSocket URL
        - On binary message: skip (JPEG frames)
        - On JSON message: parse and store latest state in thread-safe dict
        - On disconnect: retry with reconnect_interval

    physics_callback():  # fires at rates.physics Hz
        - Read latest obs_breakdown from shared state
        - Publish JointState, Imu, TF if enabled and data available

    command_callback():  # fires at rates.command Hz
        - Read latest velocity_command
        - Publish Twist if enabled and data available

    status_callback():  # fires at rates.status Hz
        - Read latest training_metrics
        - Publish rewards, status, blend JSON strings if enabled

main():
    - rclpy.init()
    - Spin GenesisRosPublisher
```

- WebSocket runs in a daemon thread (doesn't block rclpy spin)
- Latest state stored in a dict protected by `threading.Lock`
- Timers always publish most recent data (no queuing)
- Timestamps use `self.get_clock().now()`

## Dependencies

All standard in a Jazzy Docker image:
- `rclpy`, `sensor_msgs`, `geometry_msgs`, `tf2_msgs`, `std_msgs`
- `websockets` (pip install if not present)
- `pyyaml`

## Startup Sequence

```bash
# 1. Start Genesis bridge
python genesis_bridge/bridge_server.py

# 2. Inside Jazzy Docker
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# 3. Inside Jazzy Docker
python3 genesis_bridge/genesis_ros_publisher.py --config configs/genesis_ros_publisher.yaml

# 4. Start web UI
npm start
```

## Error Handling

- **Bridge not running**: Retries every `reconnect_interval` (2s), logs warning. No crash.
- **No obs_breakdown**: Skips physics topics, publishes status topics only.
- **Robot DOF mismatch**: Logs error, skips JointState until counts match.
- **Stale data**: `stale_timeout` (5s default) stops publishing stale values.
- **Graceful shutdown**: SIGINT/SIGTERM closes WebSocket, destroys node, calls rclpy.shutdown().
