#!/usr/bin/env python3
"""
sim_state_republisher.py — NATS → ROS2 bridge for sim robot state.

Subscribes to `telemetry.robot.state` on NATS (nats://127.0.0.1:4222) and
republishes as standard ROS2 messages consumed by the browser via rosbridge:

  /odom          nav_msgs/Odometry         — position + velocity
  /imu/data      sensor_msgs/Imu           — orientation + angular vel + linear accel
  /joint_states  sensor_msgs/JointState    — 12 Go2 joints

Frame conventions:
  Header frame_id: odom  (for /odom parent), base_link  (for /imu/data, /joint_states)
  Quaternion: w, x, y, z (ROS convention) — sim publishes [w, x, y, z]
  Velocities: in world frame for /odom, body frame for /imu/data (angular_velocity)

Run inside the ros-bridge container where rclpy and nats-py are available.
"""

import asyncio
import json
import logging
import sys
import time
from typing import Optional

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("sim_state_republisher")

# ── nats-py — install at startup if missing (bootstrap path for containers) ──
try:
    import nats
except ImportError:
    import subprocess
    logger.info("nats-py not found — installing now…")
    subprocess.check_call([sys.executable, "-m", "pip", "install", "nats-py"])
    import nats  # noqa: F811

# ── ROS2 — must source /opt/ros/jazzy/setup.bash before running ──
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Time as RosTime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import (
    Pose, PoseWithCovariance,
    Twist, TwistWithCovariance,
    Point, Quaternion, Vector3,
)
from std_msgs.msg import Header

NATS_URL = "nats://127.0.0.1:4222"
NATS_SUBJECT = "telemetry.robot.state"

# Publish at most every 20 ms (50 Hz cap) to avoid flooding rosbridge.
# The sim publishes at ~20 Hz so this is effectively a passthrough.
MIN_PUB_INTERVAL_S = 0.02

# Best-effort QoS for sensor streams — drop stale messages rather than queue.
_SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _now_ros_time(node: Node) -> RosTime:
    t = node.get_clock().now().to_msg()
    return t


def _make_header(node: Node, frame_id: str) -> Header:
    h = Header()
    h.stamp = _now_ros_time(node)
    h.frame_id = frame_id
    return h


class SimStateRepublisher(Node):
    """rclpy node that bridges NATS telemetry.robot.state into ROS2."""

    def __init__(self):
        super().__init__("sim_state_republisher")

        self._pub_odom = self.create_publisher(Odometry, "/odom", _SENSOR_QOS)
        self._pub_imu = self.create_publisher(Imu, "/imu/data", _SENSOR_QOS)
        self._pub_joints = self.create_publisher(JointState, "/joint_states", _SENSOR_QOS)

        self._last_pub_time = 0.0
        self._prev_lin_vel: Optional[list] = None
        self._prev_lin_vel_time: Optional[float] = None

        self.get_logger().info(
            "SimStateRepublisher ready — waiting for NATS messages on "
            f"{NATS_SUBJECT}"
        )

    def publish_state(self, state: dict) -> None:
        """Unpack a telemetry.robot.state dict and publish all three topics."""
        now_mono = time.monotonic()
        if now_mono - self._last_pub_time < MIN_PUB_INTERVAL_S:
            return
        self._last_pub_time = now_mono

        pos = state.get("pos", [0.0, 0.0, 0.0])
        quat = state.get("quat", [1.0, 0.0, 0.0, 0.0])   # [w, x, y, z]
        lin_vel = state.get("lin_vel", [0.0, 0.0, 0.0])   # world frame
        ang_vel = state.get("ang_vel", [0.0, 0.0, 0.0])   # body frame
        joint_names = state.get("joint_names", [])
        joint_pos = state.get("joint_pos", [])
        joint_vel = state.get("joint_vel", [])
        proj_grav = state.get("projected_gravity", [0.0, 0.0, -1.0])

        self._publish_odom(pos, quat, lin_vel, ang_vel)
        self._publish_imu(quat, ang_vel, lin_vel, proj_grav)
        self._publish_joints(joint_names, joint_pos, joint_vel)

    # ── /odom ──────────────────────────────────────────────────────────────────

    def _publish_odom(
        self,
        pos: list,
        quat: list,      # [w, x, y, z]
        lin_vel: list,   # world frame
        ang_vel: list,   # body frame
    ) -> None:
        msg = Odometry()
        msg.header = _make_header(self, "odom")
        msg.child_frame_id = "base_link"

        # Pose
        msg.pose.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))
        msg.pose.pose.orientation = Quaternion(
            w=float(quat[0]),
            x=float(quat[1]),
            y=float(quat[2]),
            z=float(quat[3]),
        )

        # Twist — linear velocity in body frame convention for /odom child_frame
        # child_frame_id = base_link so twist should be in base_link frame.
        # The sim publishes lin_vel in LOCAL (body) frame from robot_manager.
        msg.twist.twist.linear = Vector3(
            x=float(lin_vel[0]),
            y=float(lin_vel[1]),
            z=float(lin_vel[2]),
        )
        msg.twist.twist.angular = Vector3(
            x=float(ang_vel[0]),
            y=float(ang_vel[1]),
            z=float(ang_vel[2]),
        )

        self._pub_odom.publish(msg)

    # ── /imu/data ──────────────────────────────────────────────────────────────

    def _publish_imu(
        self,
        quat: list,
        ang_vel: list,
        lin_vel: list,   # used for finite-difference linear acceleration
        proj_grav: list,
    ) -> None:
        msg = Imu()
        msg.header = _make_header(self, "base_link")

        msg.orientation = Quaternion(
            w=float(quat[0]),
            x=float(quat[1]),
            y=float(quat[2]),
            z=float(quat[3]),
        )

        msg.angular_velocity = Vector3(
            x=float(ang_vel[0]),
            y=float(ang_vel[1]),
            z=float(ang_vel[2]),
        )

        # Linear acceleration: finite-difference lin_vel, fall back to gravity projection.
        now = time.monotonic()
        if self._prev_lin_vel is not None and self._prev_lin_vel_time is not None:
            dt = now - self._prev_lin_vel_time
            if 0.001 < dt < 0.5:
                ax = (lin_vel[0] - self._prev_lin_vel[0]) / dt
                ay = (lin_vel[1] - self._prev_lin_vel[1]) / dt
                az = (lin_vel[2] - self._prev_lin_vel[2]) / dt
            else:
                # dt out of range — use gravity projection as static estimate
                # proj_grav is [gx, gy, gz] in body frame, scale by 9.81
                ax = float(proj_grav[0]) * -9.81
                ay = float(proj_grav[1]) * -9.81
                az = float(proj_grav[2]) * -9.81
        else:
            ax = float(proj_grav[0]) * -9.81
            ay = float(proj_grav[1]) * -9.81
            az = float(proj_grav[2]) * -9.81

        self._prev_lin_vel = list(lin_vel)
        self._prev_lin_vel_time = now

        msg.linear_acceleration = Vector3(x=ax, y=ay, z=az)

        # Zero out covariance (unknown)
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self._pub_imu.publish(msg)

    # ── /joint_states ──────────────────────────────────────────────────────────

    def _publish_joints(
        self,
        names: list,
        pos: list,
        vel: list,
    ) -> None:
        if not names:
            return
        msg = JointState()
        msg.header = _make_header(self, "base_link")
        msg.name = [str(n) for n in names]
        msg.position = [float(p) for p in pos]
        msg.velocity = [float(v) for v in vel]
        # Effort not published by sim; leave as empty list (valid JointState)
        self._pub_joints.publish(msg)


# ── NATS subscriber ────────────────────────────────────────────────────────────

async def nats_loop(node: SimStateRepublisher) -> None:
    """Connect to NATS and republish messages until interrupted."""
    logger.info(f"Connecting to NATS at {NATS_URL}…")

    async def disconnected_cb():
        logger.warning("NATS disconnected")

    async def reconnected_cb():
        logger.info("NATS reconnected")

    async def error_cb(e):
        logger.error(f"NATS error: {e}")

    nc = await nats.connect(
        NATS_URL,
        disconnected_cb=disconnected_cb,
        reconnected_cb=reconnected_cb,
        error_cb=error_cb,
        max_reconnect_attempts=-1,   # retry indefinitely
        reconnect_time_wait=2,
    )
    logger.info("NATS connected")

    async def handler(msg):
        try:
            state = json.loads(msg.data.decode())
            node.publish_state(state)
        except Exception as exc:
            logger.warning(f"Failed to process NATS message: {exc}")

    await nc.subscribe(NATS_SUBJECT, cb=handler)
    logger.info(f"Subscribed to {NATS_SUBJECT}")

    # Spin forever — rclpy executor runs in a thread below
    try:
        while rclpy.ok():
            await asyncio.sleep(0.1)
    finally:
        await nc.drain()


def main() -> None:
    rclpy.init()
    node = SimStateRepublisher()

    import threading

    def spin_thread():
        rclpy.spin(node)

    t = threading.Thread(target=spin_thread, daemon=True)
    t.start()

    try:
        asyncio.run(nats_loop(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
