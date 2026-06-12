"""
Raw joint-target command math for the Go2 bridge envs.

Pure functions, no Genesis/torch imports: the sim runner converts the result
to a tensor. Order matches ActuatorManager regex ordering documented in
src/sdr_os/envs/bridge_control.py:56-60. Action convention is stand mode
(scale=1.0): action = target_rad - default_offset_rad. URDF clamping happens
downstream in PositionActionManager.
"""

# Genesis enumerates DOFs breadth-first through the kinematic tree, so the
# live actuator order is type-grouped (verified empirically via /joint_states
# and the per-joint default_pos values; NOTE this contradicts the leg-grouped
# layout comment in bridge_control.py:56-60). Prefer passing the env's own
# actuator_manager.join_names as `layout` — this constant is the fallback.
GO2_JOINT_LAYOUT = [
    "FL_hip", "FR_hip", "RL_hip", "RR_hip",
    "FL_thigh", "FR_thigh", "RL_thigh", "RR_thigh",
    "FL_calf", "FR_calf", "RL_calf", "RR_calf",
]


def normalize_joint_name(name: str) -> str:
    """Accept both 'FL_hip' and URDF-style 'FL_hip_joint'."""
    return name[:-6] if name.endswith("_joint") else name


def resolve_joint_indices(
    names: list[str], layout: list[str] | None = None
) -> list[int]:
    """Map joint names to DOF indices. `layout` is the authoritative DOF-order
    name list (e.g. actuator_manager.join_names); defaults to GO2_JOINT_LAYOUT."""
    index_by_name = {
        normalize_joint_name(n): i
        for i, n in enumerate(layout if layout is not None else GO2_JOINT_LAYOUT)
    }
    indices = []
    for raw in names:
        name = normalize_joint_name(raw)
        if name not in index_by_name:
            raise ValueError(f"unknown joint name: {raw!r}")
        indices.append(index_by_name[name])
    return indices


def merge_joint_targets(
    latch: list[float],
    names: list[str],
    positions: list[float],
    layout: list[str] | None = None,
) -> list[float]:
    """New latch with named joints updated; unnamed joints hold. Immutable."""
    if len(names) != len(positions):
        raise ValueError(
            f"names/positions length mismatch: {len(names)} vs {len(positions)}"
        )
    merged = list(latch)
    for idx, pos in zip(resolve_joint_indices(names, layout), positions):
        merged[idx] = float(pos)
    return merged


def targets_to_actions(targets: list[float], offsets: list[float]) -> list[float]:
    """Stand-mode actions (scale=1.0): radian offsets from default positions."""
    return [t - o for t, o in zip(targets, offsets)]
