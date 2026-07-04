"""Pure-function reward terms for the LL-Hurdle primitive (v46.0 design doc).

Reuses the VALIDATED v44 launch reward functions verbatim (re-exported below) and
adds exactly two terms, both exploit-resistant by construction:

- bar_clearance: one-shot, hard-gated on a genuine airborne crossing above the
  physical bar. The bar has collision enabled, so a below-top crossing is
  physically impossible; this term cannot be detection-gamed (the v43 failure).
- approach_progress: telescoping forward-record reward (the horizontal twin of
  v44's liftoff_progress). Only a NEW episode x-record pays, so pacing,
  oscillating, or post-crossing wandering earn 0.
"""

import torch

# Re-export the validated v44 terms so the env imports one module.
from rl.v44.envs.launch_rewards import (  # noqa: F401
    K_APEX, K_VX, APEX_HARD_GATE_MARGIN,
    apex_match, liftoff, ascent_shaping, forward_vel_track, posture_flight,
    landing, alive_gated, liftoff_progress, crouch_load, grounded_posture,
    _LIFTOFF_FLOOR_Z, _PROGRESS_BASELINE_GROUNDED_STEPS, _GROUNDED_MIN_Z,
    _CROUCH_FLOOR_Z, _CROUCH_TRIGGER_Z, _CROUCH_SPRING_VZ,
)

# Clearance hard-gate: base CoM must be at least this far above the bar top at
# the crossing edge for the one-shot to fire. Physics already prevents crossing
# below the top; the margin keeps whisker-grazes from crediting.
BAR_CLEARANCE_MARGIN = 0.03  # m


def bar_clearance(fired):
    """One-shot +1 on a genuine airborne crossing above the bar.

    `fired` is the env's one-shot edge mask:
      prev_x < bar_x <= x  (crossing edge this step)
      & airborne           (genuinely in flight — v44 detection w/ height guard)
      & z > bar_top + BAR_CLEARANCE_MARGIN
      & ~already_credited  (once per episode)
    All gating lives in the env's state machine (computed pre-physics like the
    other v44 latches); this function just converts the mask to reward.
    """
    return fired.to(torch.float)


def approach_progress(x, x_peak):
    """Telescoping per-step reward for setting a NEW episode forward record.

    reward = clamp(x - x_peak, 0); the peak then advances to x. Episode total
    telescopes to (x_final_max - x_spawn): walking the run-up once pays it once;
    pacing back and forth pays nothing more. No baseline machinery is needed
    (unlike the height twin) because spawn has no horizontal settle transient —
    x_peak is initialized to the spawn x at reset. Returns (reward, new_peak).
    """
    reward = torch.clamp(x - x_peak, min=0.0)
    new_peak = torch.maximum(x_peak, x)
    return reward, new_peak
