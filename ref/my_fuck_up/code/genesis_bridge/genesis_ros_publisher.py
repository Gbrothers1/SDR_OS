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

# ROS2 imports are deferred so pure-Python helpers (load_config, gravity_to_quaternion)
# can be imported and tested without rclpy installed.
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState, Imu
    from geometry_msgs.msg import Twist, TransformStamped
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from builtin_interfaces.msg import Time

    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False
    Node = object  # allow class definition to succeed

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
    if not _HAS_RCLPY:
        raise ImportError(
            "rclpy is required to run the Genesis ROS Publisher node. "
            "Install ROS2 or source your ROS2 workspace first."
        )

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
