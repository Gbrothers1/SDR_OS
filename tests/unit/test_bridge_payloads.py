import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts", "ros"))
from bridge_payloads import cmd_vel_data, joint_targets_data, wrap_command


def test_cmd_vel_data_swaps_axes_and_clamps():
    d = cmd_vel_data(fwd=2.0, strafe=-0.5, yaw=0.25)
    assert d == {
        "linear_y": 1.0, "linear_x": -0.5, "angular_z": 0.25,
        "angular_y": 0.0, "gait_enabled": True,
    }


def test_joint_targets_data():
    d = joint_targets_data(["FL_hip_joint"], [0.2])
    assert d == {"names": ["FL_hip_joint"], "positions": [0.2]}


def test_joint_targets_length_mismatch_raises():
    with pytest.raises(ValueError):
        joint_targets_data(["FL_hip"], [0.1, 0.2])


def test_wrap_command_tags_mcp_source():
    p = wrap_command("set_cmd_vel", 7, {"linear_y": 1.0})
    assert p == {
        "action": "set_cmd_vel", "cmd_seq": 7, "source": "mcp",
        "data": {"linear_y": 1.0},
    }
