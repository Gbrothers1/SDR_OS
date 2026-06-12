import pytest
from src.sdr_os.control.joint_command import (
    GO2_JOINT_LAYOUT, normalize_joint_name, resolve_joint_indices,
    merge_joint_targets, targets_to_actions,
)

# Type-grouped (Genesis breadth-first DOF order): hips, thighs, calves
GO2_DEFAULTS = [0.0, 0.0, 0.0, 0.0, 0.8, 0.8, 1.0, 1.0, -1.6, -1.6, -1.6, -1.6]


def test_layout_is_12_joints_type_grouped():
    assert len(GO2_JOINT_LAYOUT) == 12
    assert GO2_JOINT_LAYOUT[0] == "FL_hip"
    assert GO2_JOINT_LAYOUT[4] == "FL_thigh"
    assert GO2_JOINT_LAYOUT[11] == "RR_calf"


def test_normalize_strips_joint_suffix():
    assert normalize_joint_name("FL_hip_joint") == "FL_hip"
    assert normalize_joint_name("FL_hip") == "FL_hip"


def test_resolve_known_names():
    assert resolve_joint_indices(["FR_thigh_joint", "FL_hip"]) == [5, 0]


def test_resolve_with_runtime_layout():
    layout = ["RR_calf_joint", "FL_hip_joint"]  # authoritative env order wins
    assert resolve_joint_indices(["FL_hip", "RR_calf_joint"], layout) == [1, 0]


def test_resolve_unknown_name_raises():
    with pytest.raises(ValueError, match="unknown joint"):
        resolve_joint_indices(["FL_elbow"])


def test_merge_partial_update_holds_others():
    latch = list(GO2_DEFAULTS)
    merged = merge_joint_targets(latch, ["FL_thigh"], [0.3])
    assert merged[4] == 0.3
    assert merged[0] == latch[0] and merged[5] == latch[5]
    assert latch[4] == 0.8  # input not mutated


def test_merge_length_mismatch_raises():
    with pytest.raises(ValueError, match="length"):
        merge_joint_targets(list(GO2_DEFAULTS), ["FL_hip"], [0.1, 0.2])


def test_targets_to_actions_is_target_minus_offset():
    targets = list(GO2_DEFAULTS)
    targets[5] = 1.3  # FR_thigh: 0.8 default → action 0.5
    actions = targets_to_actions(targets, GO2_DEFAULTS)
    assert actions[5] == pytest.approx(0.5)
    assert actions[0] == pytest.approx(0.0)
