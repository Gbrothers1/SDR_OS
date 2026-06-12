#!/usr/bin/env python3
"""
cmd_vel_to_nats_bridge.py — ROS2 /sim/cmd_vel → NATS command.genesis.set_cmd_vel

Completes the inbound command path for ROS clients (ros-mcp-server, RViz teleop,
any rosbridge publisher). The existing sim_state_republisher covers sim → ROS
telemetry only; this bridge covers ROS → sim driving commands.

Deliberately NOT /cmd_vel: the web UI dual-publishes /cmd_vel (for real-robot
passthrough) in parallel with its direct NATS command stream. Bridging
/cmd_vel loops those duplicates back into the sim and the two cmd_seq streams
fight (observed: hundreds of "seq reset detected" per minute, gait flapping).
/sim/cmd_vel carries only deliberate sim-driving traffic.

Runs INSIDE the ros-bridge container (needs rclpy) as a native ROS node — it
must NOT subscribe through rosbridge itself: rosbridge's own /cmd_vel
subscription is created RELIABLE+TRANSIENT_LOCAL, which silently rejects all
normal VOLATILE publishers (observed on Jazzy; see "incompatible QoS" warnings
in rosbridge logs). A native subscription with VOLATILE durability is
compatible with every publisher.

Each Twist is forwarded to NATS exactly once. No latching, no republishing:
the sim's Layer-3 TTL safety (200ms HOLD / 2s ESTOP) stays fully
authoritative. To keep the robot moving, the publisher must stream
continuously (e.g. ros-mcp publish_for_durations with rate_hz=20). When the
stream stops, the sim HOLDs on its own.

Unit contract: Twist fields are NORMALIZED STICK VALUES in [-1, 1], identical
to what the web UI gamepad sends — NOT m/s. The env maps them onto its active
per-gait velocity envelope (go2_env.set_velocity_from_gamepad/_map_stick).

  Twist.linear.x  (forward, -1..1) → NATS data.linear_y  → env lin_vel_x
  Twist.linear.y  (strafe,  -1..1) → NATS data.linear_x  → env lin_vel_y
  Twist.angular.z (yaw,     -1..1) → NATS data.angular_z → env ang_vel_z

Every bridged message sets gait_enabled=true (equivalent of holding L2 on the
gamepad): publishing to /sim/cmd_vel means "drive". Every payload is tagged
source="mcp"; the sim arbitrates per-source (operator non-zero input always
wins, idle UI zeros never preempt) so driving with the web UI tab open is safe.

Raw joint control: /sim/joint_cmd (sensor_msgs/msg/JointState, name[] +
position[] in radians, partial joint sets allowed — unnamed joints hold) →
command.genesis.set_joint_targets. While the joint stream is fresh (<0.5 s)
the sim bypasses the RL policy and PD-tracks the targets directly; when it
goes stale the sim falls back to policy control.

Config via env vars:
  SDR_NATS_URL  (default nats://127.0.0.1:4222)
"""

import asyncio
import json
import logging
import os
import sys
import time

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("cmd_vel_to_nats_bridge")

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
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bridge_payloads import cmd_vel_data, joint_targets_data, wrap_command

NATS_URL = os.environ.get("SDR_NATS_URL", "nats://127.0.0.1:4222")
CMD_SUBJECT = "command.genesis.set_cmd_vel"
JOINT_SUBJECT = "command.genesis.set_joint_targets"

# Drop messages arriving faster than 100 Hz — protects NATS/sim from floods.
MIN_FORWARD_INTERVAL_S = 0.01

# VOLATILE durability accepts both VOLATILE and TRANSIENT_LOCAL publishers;
# RELIABLE matches rosbridge/ros2-cli/web-UI publishers (all RELIABLE).
_CMD_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class CmdVelToNatsBridge(Node):
    """rclpy node that forwards /cmd_vel Twists to NATS set_cmd_vel commands."""

    def __init__(self):
        super().__init__("cmd_vel_to_nats_bridge")
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=4)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._last_recv = 0.0
        self.create_subscription(Twist, "/sim/cmd_vel", self._on_twist, _CMD_QOS)
        self.get_logger().info("Subscribed to /sim/cmd_vel (RELIABLE, VOLATILE)")
        self.create_subscription(
            JointState, "/sim/joint_cmd", self._on_joint_state, _CMD_QOS
        )
        self.get_logger().info("Subscribed to /sim/joint_cmd (RELIABLE, VOLATILE)")

    def attach_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self._loop = loop

    def _on_twist(self, msg: Twist) -> None:
        """ROS executor thread → hand off to the asyncio/NATS thread."""
        # Axis swap is intentional — see module docstring.
        data = cmd_vel_data(fwd=msg.linear.x, strafe=msg.linear.y, yaw=msg.angular.z)
        self._enqueue(CMD_SUBJECT, data)

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            data = joint_targets_data(
                list(msg.name),
                list(msg.position),
                kd=list(msg.velocity),  # JointState.velocity[] carries kd
                kp=list(msg.effort),    # JointState.effort[] carries kp
            )
        except ValueError as e:
            self.get_logger().warning(f"/sim/joint_cmd rejected: {e}")
            return
        self._enqueue(JOINT_SUBJECT, data)

    def _enqueue(self, subject: str, data: dict) -> None:
        now = time.monotonic()
        if now - self._last_recv < MIN_FORWARD_INTERVAL_S or self._loop is None:
            return
        self._last_recv = now
        try:
            self._loop.call_soon_threadsafe(self._queue.put_nowait, (subject, data))
        except (asyncio.QueueFull, RuntimeError):
            pass  # drop rather than block the executor; next message replaces it


async def nats_loop(node: CmdVelToNatsBridge) -> None:
    """Connect to NATS and forward queued commands until interrupted."""
    logger.info(f"Connecting to NATS at {NATS_URL}…")

    async def on_disconnect():
        logger.warning("NATS disconnected")

    async def on_reconnect():
        logger.info("NATS reconnected")

    nc = await nats.connect(
        NATS_URL,
        max_reconnect_attempts=-1,
        reconnect_time_wait=2,
        disconnected_cb=on_disconnect,
        reconnected_cb=on_reconnect,
    )
    logger.info("NATS connected — bridge ready")

    node.attach_loop(asyncio.get_running_loop())

    # cmd_seq starts at 1: the sim treats a large backwards jump as a new
    # session and accepts it, so we cleanly take over from a stale web UI.
    cmd_seq = 0
    forwarded = 0
    try:
        while rclpy.ok():
            try:
                subject, data = await asyncio.wait_for(node._queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            cmd_seq += 1
            action = "set_cmd_vel" if subject == CMD_SUBJECT else "set_joint_targets"
            payload = wrap_command(action, cmd_seq, data)
            await nc.publish(subject, json.dumps(payload).encode())
            forwarded += 1
            if forwarded % 100 == 1:
                logger.info(f"forwarded #{forwarded} seq={cmd_seq} {action} {data}")
    finally:
        await nc.drain()


def main() -> None:
    rclpy.init()
    node = CmdVelToNatsBridge()

    import threading

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    try:
        asyncio.run(nats_loop(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
