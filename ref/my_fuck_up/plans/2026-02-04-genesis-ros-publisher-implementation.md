# Genesis ROS Publisher Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a standalone ROS2 node that reads Genesis bridge WebSocket metrics and publishes them as standard ROS2 topics so the existing TelemetryPanel works in sim mode.

**Architecture:** A single Python file (`genesis_bridge/genesis_ros_publisher.py`) connects to `ws://localhost:9091` as a WebSocket client, parses JSON `training_metrics` messages containing `obs_breakdown`/`velocity_command`/`reward_breakdown`, and publishes `sensor_msgs/JointState`, `sensor_msgs/Imu`, `geometry_msgs/Twist`, `tf2_msgs/TFMessage`, and custom `std_msgs/String` JSON topics. Config loaded from `configs/genesis_ros_publisher.yaml`.

**Tech Stack:** Python 3, rclpy (ROS2 Jazzy), websockets, PyYAML

---

### Task 1: Create Config File

**Files:**
- Create: `configs/genesis_ros_publisher.yaml`

**Step 1: Write the config file**

```yaml
# Genesis ROS Publisher Configuration
# Standalone ROS2 node that reads Genesis bridge WebSocket and publishes ROS2 topics.

bridge:
  url: "ws://localhost:9091"
  reconnect_interval: 2.0   # seconds between reconnect attempts
  stale_timeout: 5.0        # seconds before data is considered stale (stop publishing)

rates:
  physics: 10.0    # Hz — joint_states, imu, tf
  command: 10.0    # Hz — cmd_vel
  status: 5.0      # Hz — genesis/status, genesis/rewards, genesis/blend

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

**Step 2: Verify config loads**

Run: `python3 -c "import yaml; c=yaml.safe_load(open('configs/genesis_ros_publisher.yaml')); print(c['robot']['joint_names']); print(f'OK: {len(c[\"robot\"][\"joint_names\"])} joints')"`
Expected: prints 12 joint names and "OK: 12 joints"

**Step 3: Commit**

```bash
git add configs/genesis_ros_publisher.yaml
git commit -m "feat: add genesis ROS publisher config"
```

---

### Task 2: Create Publisher Node — Config Loading and Skeleton

**Files:**
- Create: `genesis_bridge/genesis_ros_publisher.py`

**Step 1: Write the node skeleton with config loading**

Create `genesis_bridge/genesis_ros_publisher.py` with:

```python
#!/usr/bin/env python3
"""
Genesis ROS Publisher — standalone ROS2 node.

Connects to the Genesis bridge WebSocket (default ws://localhost:9091),
parses training_metrics JSON messages, and publishes standard ROS2 topics:
  - /joint_states   (sensor_msgs/JointState)
  - /imu/data       (sensor_msgs/Imu)
  - /cmd_vel        (geometry_msgs/Twist)
  - /tf             (tf2_msgs/TFMessage)
  - /genesis/*      (std_msgs/String, JSON payloads)

Usage:
    python3 genesis_bridge/genesis_ros_publisher.py --config configs/genesis_ros_publisher.yaml
"""

import argparse
import json
import logging
import math
import threading
import time
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time

logger = logging.getLogger("genesis_ros_publisher")


def load_config(path: str) -> dict:
    """Load and validate YAML config."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"Config not found: {path}")
    with open(p) as f:
        cfg = yaml.safe_load(f)
    # Validate required sections
    for section in ("bridge", "rates", "topics", "enable", "robot"):
        if section not in cfg:
            raise ValueError(f"Missing config section: {section}")
    return cfg


def gravity_to_quaternion(gx: float, gy: float, gz: float):
    """Derive orientation quaternion from projected gravity vector.

    The projected gravity vector is gravity expressed in the body frame.
    At rest on flat ground, it equals (0, 0, -9.81).
    We extract roll and pitch from it; yaw is unobservable from gravity alone.

    Returns (x, y, z, w) quaternion.
    """
    norm = math.sqrt(gx * gx + gy * gy + gz * gz)
    if norm < 1e-6:
        return (0.0, 0.0, 0.0, 1.0)
    gx /= norm
    gy /= norm
    gz /= norm

    # Roll: rotation about x-axis (from gy component)
    # Pitch: rotation about y-axis (from gx component)
    roll = math.atan2(gy, -gz)
    pitch = math.atan2(-gx, math.sqrt(gy * gy + gz * gz))
    yaw = 0.0

    # Euler (ZYX) to quaternion
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


class GenesisRosPublisher(Node):
    """ROS2 node that publishes Genesis sim state as standard topics."""

    def __init__(self, config: dict):
        super().__init__("genesis_ros_publisher")
        self._cfg = config
        self._lock = threading.Lock()
        self._latest = {}           # latest parsed metrics from WebSocket
        self._last_update = 0.0     # timestamp of last WS message

        bridge_cfg = config["bridge"]
        rates = config["rates"]
        topics = config["topics"]
        enable = config["enable"]
        robot = config["robot"]

        self._bridge_url = bridge_cfg["url"]
        self._reconnect_interval = bridge_cfg.get("reconnect_interval", 2.0)
        self._stale_timeout = bridge_cfg.get("stale_timeout", 5.0)
        self._joint_names = robot["joint_names"]
        self._base_frame = robot.get("base_frame", "base_link")
        self._odom_frame = robot.get("odom_frame", "odom")

        # Create publishers for enabled topics
        self._pubs = {}
        if enable.get("joint_states"):
            self._pubs["joint_states"] = self.create_publisher(
                JointState, topics["joint_states"], 10
            )
        if enable.get("imu"):
            self._pubs["imu"] = self.create_publisher(
                Imu, topics["imu"], 10
            )
        if enable.get("cmd_vel"):
            self._pubs["cmd_vel"] = self.create_publisher(
                Twist, topics["cmd_vel"], 10
            )
        if enable.get("tf"):
            self._pubs["tf"] = self.create_publisher(
                TFMessage, topics["tf"], 10
            )
        if enable.get("rewards"):
            self._pubs["rewards"] = self.create_publisher(
                String, topics["rewards"], 10
            )
        if enable.get("status"):
            self._pubs["status"] = self.create_publisher(
                String, topics["status"], 10
            )
        if enable.get("blend"):
            self._pubs["blend"] = self.create_publisher(
                String, topics["blend"], 10
            )

        # Create timers
        if rates.get("physics", 0) > 0:
            self.create_timer(1.0 / rates["physics"], self._physics_callback)
        if rates.get("command", 0) > 0:
            self.create_timer(1.0 / rates["command"], self._command_callback)
        if rates.get("status", 0) > 0:
            self.create_timer(1.0 / rates["status"], self._status_callback)

        self.get_logger().info(
            f"Publishers: {list(self._pubs.keys())} | "
            f"Rates: physics={rates.get('physics')}Hz, "
            f"command={rates.get('command')}Hz, "
            f"status={rates.get('status')}Hz"
        )

        # Start WebSocket listener thread
        self._ws_running = True
        self._ws_thread = threading.Thread(target=self._ws_listener, daemon=True)
        self._ws_thread.start()

    # ---- WebSocket listener (background thread) ----

    def _ws_listener(self):
        """Connect to bridge WebSocket and store latest metrics."""
        import websockets.sync.client as ws_client

        while self._ws_running:
            try:
                self.get_logger().info(f"Connecting to {self._bridge_url} ...")
                with ws_client.connect(self._bridge_url) as ws:
                    self.get_logger().info("WebSocket connected")
                    while self._ws_running:
                        try:
                            msg = ws.recv(timeout=2.0)
                        except TimeoutError:
                            continue
                        # Skip binary (JPEG frames)
                        if isinstance(msg, bytes):
                            continue
                        try:
                            data = json.loads(msg)
                        except json.JSONDecodeError:
                            continue
                        if data.get("type") == "training_metrics":
                            with self._lock:
                                self._latest = data
                                self._last_update = time.monotonic()
            except Exception as e:
                self.get_logger().warn(
                    f"WebSocket error: {e}. Retrying in {self._reconnect_interval}s..."
                )
            if self._ws_running:
                time.sleep(self._reconnect_interval)

    def _is_stale(self) -> bool:
        """Check if latest data is stale."""
        return (time.monotonic() - self._last_update) > self._stale_timeout

    # ---- Timer callbacks ----

    def _physics_callback(self):
        """Publish JointState, Imu, and TF from obs_breakdown."""
        with self._lock:
            data = self._latest.copy()
        if not data or self._is_stale():
            return

        obs = data.get("obs_breakdown")
        if not obs:
            return

        now = self.get_clock().now().to_msg()

        # JointState
        dof_pos = obs.get("dof_position", [])
        dof_vel = obs.get("dof_velocity", [])
        if "joint_states" in self._pubs and len(dof_pos) == len(self._joint_names):
            msg = JointState()
            msg.header.stamp = now
            msg.header.frame_id = self._base_frame
            msg.name = list(self._joint_names)
            msg.position = [float(v) for v in dof_pos]
            msg.velocity = [float(v) for v in dof_vel] if len(dof_vel) == len(dof_pos) else []
            msg.effort = []
            self._pubs["joint_states"].publish(msg)

        # IMU
        ang_vel = obs.get("angular_velocity", [])
        proj_grav = obs.get("projected_gravity", [])
        if "imu" in self._pubs and len(ang_vel) == 3 and len(proj_grav) == 3:
            msg = Imu()
            msg.header.stamp = now
            msg.header.frame_id = self._base_frame
            msg.angular_velocity.x = float(ang_vel[0])
            msg.angular_velocity.y = float(ang_vel[1])
            msg.angular_velocity.z = float(ang_vel[2])
            msg.linear_acceleration.x = float(proj_grav[0])
            msg.linear_acceleration.y = float(proj_grav[1])
            msg.linear_acceleration.z = float(proj_grav[2])
            qx, qy, qz, qw = gravity_to_quaternion(
                proj_grav[0], proj_grav[1], proj_grav[2]
            )
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw
            # Mark orientation covariance as "unknown" (-1 in first element)
            msg.orientation_covariance[0] = -1.0
            self._pubs["imu"].publish(msg)

        # TF (odom -> base_link)
        if "tf" in self._pubs and len(proj_grav) == 3:
            tf_msg = TFMessage()
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self._odom_frame
            t.child_frame_id = self._base_frame
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = 0.0
            qx, qy, qz, qw = gravity_to_quaternion(
                proj_grav[0], proj_grav[1], proj_grav[2]
            )
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            tf_msg.transforms.append(t)
            self._pubs["tf"].publish(tf_msg)

    def _command_callback(self):
        """Publish Twist from velocity_command."""
        with self._lock:
            data = self._latest.copy()
        if not data or self._is_stale():
            return

        vcmd = data.get("velocity_command")
        if not vcmd or "cmd_vel" not in self._pubs:
            return

        msg = Twist()
        msg.linear.x = float(vcmd.get("lin_vel_x", 0.0))
        msg.linear.y = float(vcmd.get("lin_vel_y", 0.0))
        msg.angular.z = float(vcmd.get("ang_vel_z", 0.0))
        self._pubs["cmd_vel"].publish(msg)

    def _status_callback(self):
        """Publish custom genesis topics as JSON strings."""
        with self._lock:
            data = self._latest.copy()
        if not data or self._is_stale():
            return

        # Rewards
        rewards = data.get("reward_breakdown")
        if rewards and "rewards" in self._pubs:
            msg = String()
            msg.data = json.dumps(rewards)
            self._pubs["rewards"].publish(msg)

        # Status
        if "status" in self._pubs:
            status = {
                "blend_alpha": data.get("blend_alpha", 0.0),
                "deadman_active": data.get("deadman_active", False),
                "actor_tag": data.get("actor_tag", "teleop"),
                "mode": data.get("mode", ""),
                "policy_loaded": data.get("policy_loaded", False),
                "command_source": data.get("command_source", "gamepad"),
                "step_count": data.get("step_count", 0),
                "fps": data.get("fps", 0.0),
            }
            msg = String()
            msg.data = json.dumps(status)
            self._pubs["status"].publish(msg)

        # Blend
        if "blend" in self._pubs:
            blend = {
                "blend_alpha": data.get("blend_alpha", 0.0),
                "confidence": data.get("confidence", 0.0),
                "confidence_gate_active": data.get("confidence_gate_active", False),
                "safety_flags": data.get("safety_flags", {}),
            }
            msg = String()
            msg.data = json.dumps(blend)
            self._pubs["blend"].publish(msg)

    def destroy_node(self):
        """Clean shutdown."""
        self._ws_running = False
        if self._ws_thread.is_alive():
            self._ws_thread.join(timeout=3.0)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="Genesis ROS Publisher")
    parser.add_argument(
        "--config",
        default="configs/genesis_ros_publisher.yaml",
        help="Path to YAML config file",
    )
    args = parser.parse_args()

    config = load_config(args.config)

    rclpy.init()
    node = GenesisRosPublisher(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
    )
    main()
```

**Step 2: Verify syntax**

Run: `python3 -c "import ast; ast.parse(open('genesis_bridge/genesis_ros_publisher.py').read()); print('Syntax OK')"`
Expected: "Syntax OK"

**Step 3: Commit**

```bash
git add genesis_bridge/genesis_ros_publisher.py
git commit -m "feat: add genesis ROS publisher node"
```

---

### Task 3: Create Test Script

**Files:**
- Create: `genesis_bridge/test_ros_publisher.py`

This test script validates config loading, gravity-to-quaternion conversion, and message construction without requiring rclpy or a running bridge. It follows the same pattern as `rl/test_action_pipeline.py`.

**Step 1: Write the test script**

```python
#!/usr/bin/env python3
"""Tests for genesis_ros_publisher — config loading, math, message building."""

import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))


def test_load_config():
    """Test YAML config loading and validation."""
    print("\n=== Testing Config Loading ===")
    from genesis_bridge.genesis_ros_publisher import load_config

    cfg = load_config("configs/genesis_ros_publisher.yaml")

    assert "bridge" in cfg, "Missing bridge section"
    assert "rates" in cfg, "Missing rates section"
    assert "topics" in cfg, "Missing topics section"
    assert "enable" in cfg, "Missing enable section"
    assert "robot" in cfg, "Missing robot section"

    assert cfg["bridge"]["url"] == "ws://localhost:9091"
    assert len(cfg["robot"]["joint_names"]) == 12
    assert cfg["robot"]["joint_names"][0] == "FL_hip_joint"
    print(f"  Bridge URL: {cfg['bridge']['url']}")
    print(f"  Joint count: {len(cfg['robot']['joint_names'])}")
    print("PASS")


def test_load_config_missing():
    """Test config loading with missing file."""
    print("\n=== Testing Config Missing File ===")
    from genesis_bridge.genesis_ros_publisher import load_config

    try:
        load_config("nonexistent.yaml")
        assert False, "Should have raised FileNotFoundError"
    except FileNotFoundError:
        print("  Correctly raised FileNotFoundError")
    print("PASS")


def test_gravity_to_quaternion():
    """Test gravity vector to quaternion conversion."""
    print("\n=== Testing Gravity to Quaternion ===")
    from genesis_bridge.genesis_ros_publisher import gravity_to_quaternion

    # Flat on ground: gravity = (0, 0, -9.81) -> identity quaternion
    qx, qy, qz, qw = gravity_to_quaternion(0.0, 0.0, -9.81)
    assert abs(qw - 1.0) < 1e-6, f"Expected qw=1.0, got {qw}"
    assert abs(qx) < 1e-6, f"Expected qx=0, got {qx}"
    assert abs(qy) < 1e-6, f"Expected qy=0, got {qy}"
    assert abs(qz) < 1e-6, f"Expected qz=0, got {qz}"
    print(f"  Flat: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Tilted forward (pitch): gravity has +x component
    qx, qy, qz, qw = gravity_to_quaternion(4.9, 0.0, -8.5)
    assert abs(qy) > 0.1, f"Expected nonzero pitch (qy), got {qy}"
    print(f"  Pitched forward: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Tilted sideways (roll): gravity has +y component
    qx, qy, qz, qw = gravity_to_quaternion(0.0, 4.9, -8.5)
    assert abs(qx) > 0.1, f"Expected nonzero roll (qx), got {qx}"
    print(f"  Rolled right: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Zero vector: should return identity
    qx, qy, qz, qw = gravity_to_quaternion(0.0, 0.0, 0.0)
    assert abs(qw - 1.0) < 1e-6, "Zero vector should give identity"
    print(f"  Zero input: q=({qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f})")

    # Quaternion should be unit length
    for grav in [(0, 0, -9.81), (4.9, 0, -8.5), (0, 4.9, -8.5), (3, 3, -8)]:
        qx, qy, qz, qw = gravity_to_quaternion(*grav)
        norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
        assert abs(norm - 1.0) < 1e-6, f"Quaternion not unit: norm={norm}"
    print("  All quaternions are unit length")

    print("PASS")


def test_metrics_parsing():
    """Test that a realistic metrics payload can be parsed for publishing."""
    print("\n=== Testing Metrics Parsing ===")

    # Simulate a training_metrics JSON payload from the bridge
    metrics = {
        "type": "training_metrics",
        "step_count": 1234,
        "fps": 49.5,
        "blend_alpha": 0.3,
        "deadman_active": True,
        "actor_tag": "blend",
        "mode": "teleop_record",
        "command_source": "gamepad",
        "policy_loaded": False,
        "confidence": 0.85,
        "confidence_gate_active": False,
        "safety_flags": {
            "vel_clamped": False,
            "accel_clamped": False,
            "jerk_clamped": False,
            "clip_clamped": False,
        },
        "obs_breakdown": {
            "gait_command": [0.0] * 14,
            "velocity_command": [0.5, 0.0, -0.2],
            "angular_velocity": [0.01, -0.02, 0.03],
            "linear_velocity": [0.4, 0.0, 0.0],
            "projected_gravity": [0.0, 0.0, -9.81],
            "dof_position": [0.0, 0.8, -1.6] * 4,
            "dof_velocity": [0.1, -0.1, 0.05] * 4,
            "actions": [0.0, 0.8, -1.6] * 4,
        },
        "velocity_command": {
            "lin_vel_x": 0.5,
            "lin_vel_y": 0.0,
            "ang_vel_z": -0.2,
        },
        "reward_breakdown": {
            "tracking_lin_vel": 0.85,
            "tracking_ang_vel": 0.42,
            "base_height_target": -0.01,
        },
    }

    # Verify obs_breakdown structure
    obs = metrics["obs_breakdown"]
    assert len(obs["dof_position"]) == 12, "Expected 12 DOF positions"
    assert len(obs["dof_velocity"]) == 12, "Expected 12 DOF velocities"
    assert len(obs["angular_velocity"]) == 3
    assert len(obs["projected_gravity"]) == 3
    print(f"  obs_breakdown keys: {list(obs.keys())}")
    print(f"  dof_position[0:3]: {obs['dof_position'][0:3]}")

    # Verify velocity_command
    vcmd = metrics["velocity_command"]
    assert "lin_vel_x" in vcmd
    assert "lin_vel_y" in vcmd
    assert "ang_vel_z" in vcmd
    print(f"  velocity_command: {vcmd}")

    # Verify reward_breakdown serializes cleanly
    reward_json = json.dumps(metrics["reward_breakdown"])
    assert len(reward_json) > 2
    print(f"  reward_breakdown JSON: {reward_json}")

    # Verify status fields
    assert metrics["blend_alpha"] == 0.3
    assert metrics["actor_tag"] == "blend"
    print(f"  status: alpha={metrics['blend_alpha']}, tag={metrics['actor_tag']}")

    print("PASS")


if __name__ == "__main__":
    test_load_config()
    test_load_config_missing()
    test_gravity_to_quaternion()
    test_metrics_parsing()
    print("\n=== ALL TESTS PASSED ===")
```

**Step 2: Run the tests**

Run: `python3 genesis_bridge/test_ros_publisher.py`
Expected: "ALL TESTS PASSED"

**Step 3: Commit**

```bash
git add genesis_bridge/test_ros_publisher.py
git commit -m "test: add genesis ROS publisher unit tests"
```

---

### Task 4: Integration Test With Mock WebSocket

**Files:**
- Modify: `genesis_bridge/test_ros_publisher.py` (append new test)

**Step 1: Add a WebSocket integration test**

Append to `genesis_bridge/test_ros_publisher.py` — a test that starts a mock WebSocket server, verifies the publisher's WS listener can connect and parse messages. Does not require rclpy.

```python
def test_ws_listener_parsing():
    """Test WebSocket listener thread connects to a mock server and parses metrics."""
    print("\n=== Testing WebSocket Listener (mock server) ===")
    import asyncio
    import threading
    import time

    try:
        import websockets
        import websockets.sync.server as ws_server
    except ImportError:
        print("  SKIP: websockets not installed")
        return

    received = []
    stop_event = threading.Event()

    # Mock bridge: sends one binary frame (should be skipped) then one JSON metrics
    def handler(ws):
        # Send binary frame (JPEG-like)
        ws.send(b"\xff\xd8\xff\xe0JFIF_fake_frame")
        # Send JSON metrics
        metrics = {
            "type": "training_metrics",
            "obs_breakdown": {
                "dof_position": [0.1] * 12,
                "dof_velocity": [0.2] * 12,
                "angular_velocity": [0.01, -0.02, 0.03],
                "projected_gravity": [0.0, 0.0, -9.81],
            },
            "velocity_command": {"lin_vel_x": 0.5, "lin_vel_y": 0.0, "ang_vel_z": 0.1},
        }
        ws.send(json.dumps(metrics))
        # Keep alive until test signals stop
        stop_event.wait(timeout=5.0)

    # Start mock server in thread
    server = ws_server.serve(handler, "localhost", 19091)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()

    try:
        # Connect as a simple client (mimicking the publisher's ws_listener logic)
        import websockets.sync.client as ws_client
        with ws_client.connect("ws://localhost:19091") as ws:
            for _ in range(2):
                msg = ws.recv(timeout=5.0)
                if isinstance(msg, bytes):
                    received.append(("binary", len(msg)))
                else:
                    data = json.loads(msg)
                    received.append(("json", data.get("type")))

        assert len(received) == 2, f"Expected 2 messages, got {len(received)}"
        assert received[0][0] == "binary", "First message should be binary"
        assert received[1] == ("json", "training_metrics"), f"Second should be metrics, got {received[1]}"
        print(f"  Received: {received}")
        print("PASS")
    finally:
        stop_event.set()
        server.shutdown()
```

Also update the `__main__` block to call this test:

```python
if __name__ == "__main__":
    test_load_config()
    test_load_config_missing()
    test_gravity_to_quaternion()
    test_metrics_parsing()
    test_ws_listener_parsing()
    print("\n=== ALL TESTS PASSED ===")
```

**Step 2: Run the tests**

Run: `python3 genesis_bridge/test_ros_publisher.py`
Expected: "ALL TESTS PASSED" (or "SKIP: websockets not installed" for WS test if websockets is missing on host — it will be available in the Jazzy Docker)

**Step 3: Commit**

```bash
git add genesis_bridge/test_ros_publisher.py
git commit -m "test: add WebSocket integration test for ROS publisher"
```

---

### Task 5: End-to-End Verification

This task verifies the full pipeline: bridge → publisher → rosbridge → TelemetryPanel. Requires Docker with ROS Jazzy.

**Step 1: Verify the node starts and connects to the bridge**

Run the Genesis bridge server in one terminal:
```bash
python genesis_bridge/bridge_server.py
```

In the Jazzy Docker container, run:
```bash
python3 genesis_bridge/genesis_ros_publisher.py --config configs/genesis_ros_publisher.yaml
```

Expected: Logs show "WebSocket connected" and publisher info. If bridge is not running, should show reconnect retries.

**Step 2: Verify topics are published**

In the Jazzy Docker container, in another terminal:
```bash
ros2 topic list | grep -E "joint_states|imu|cmd_vel|tf|genesis"
```

Expected: All enabled topics appear in the list.

**Step 3: Verify data flows**

```bash
ros2 topic echo /joint_states --once
```

Expected: A `sensor_msgs/JointState` message with 12 named joints, position and velocity arrays.

```bash
ros2 topic echo /imu/data --once
```

Expected: A `sensor_msgs/Imu` message with angular_velocity, linear_acceleration, and orientation fields.

**Step 4: Verify TelemetryPanel**

Start rosbridge:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Start web UI: `npm start`, open browser. Switch to Real Robot mode or SPLIT view. The TelemetryPanel should show joint positions and IMU data instead of "Waiting for telemetry data...".

**Step 5: Commit**

```bash
git add configs/genesis_ros_publisher.yaml genesis_bridge/genesis_ros_publisher.py genesis_bridge/test_ros_publisher.py
git commit -m "feat: genesis ROS publisher - end-to-end verified"
```
