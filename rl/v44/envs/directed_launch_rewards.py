"""Reward terms for the v44.1 directed launch.

Re-exports the validated v44 launch terms and adds stand_track — the dense
phase reward for the STAND (pre-trigger) and RECOVER (post-arc) phases. It is
multiplicative (height-hold x stillness), the v45-validated dense-track shape.

Budget (design doc): during the LAUNCH window (trigger latched, arc not yet
closed) stand_track is gated OFF — standing through the trigger earns nothing
for the rest of the window, while jumping earns the one-shot bundle (~44) plus
post-landing stand pay. Jumping pre-trigger pays nothing (jump machinery is
trigger-gated) AND forfeits stand pay during the airtime. The only optimum is:
wait, jump on command, recover.
"""

import torch

from rl.v44.envs.launch_rewards import (  # noqa: F401
    K_APEX, K_VX, APEX_HARD_GATE_MARGIN,
    apex_match, liftoff, ascent_shaping, forward_vel_track, posture_flight,
    landing, alive_gated, liftoff_progress, crouch_load, grounded_posture,
    _LIFTOFF_FLOOR_Z, _PROGRESS_BASELINE_GROUNDED_STEPS, _GROUNDED_MIN_Z,
    _CROUCH_FLOOR_Z, _CROUCH_TRIGGER_Z, _CROUCH_SPRING_VZ,
)

_STAND_TARGET_Z = 0.31   # URDF standing CoM
K_STAND_Z = 20.0         # height-hold sharpness
K_STAND_V = 1.0          # stillness sharpness (per (m/s)^2 of total speed)


def stand_track(z, speed_sq, active):
    """Multiplicative stand reward: hold standing height x be still.

    exp(-K_STAND_Z (z - 0.31)^2) * exp(-K_STAND_V * |v|^2), gated by `active`
    (STAND phase or post-arc RECOVER phase). Multiplicative so neither a still
    crouch nor a jittery stand pays well — only a quiet, full-height stand.
    """
    r = torch.exp(-K_STAND_Z * (z - _STAND_TARGET_Z) ** 2) * torch.exp(-K_STAND_V * speed_sq)
    return r * active.to(r.dtype)
