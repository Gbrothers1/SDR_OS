"""
Pure NATS payload builders for the ROS→NATS command bridge.

No rclpy/nats imports so they unit-test on the host. The axis swap in
cmd_vel_data is the documented stick→NATS convention (see
cmd_vel_to_nats_bridge.py module docstring): Twist.linear.x (forward) →
data.linear_y, Twist.linear.y (strafe) → data.linear_x.
"""


def _clamp_stick(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def cmd_vel_data(fwd: float, strafe: float, yaw: float) -> dict:
    return {
        "linear_y": _clamp_stick(fwd),
        "linear_x": _clamp_stick(strafe),
        "angular_z": _clamp_stick(yaw),
        "angular_y": 0.0,
        "gait_enabled": True,
    }


def joint_targets_data(names: list, positions: list) -> dict:
    if len(names) != len(positions):
        raise ValueError(
            f"names/positions length mismatch: {len(names)} vs {len(positions)}"
        )
    return {"names": list(names), "positions": [float(p) for p in positions]}


def wrap_command(action: str, cmd_seq: int, data: dict) -> dict:
    return {"action": action, "cmd_seq": cmd_seq, "source": "mcp", "data": data}
