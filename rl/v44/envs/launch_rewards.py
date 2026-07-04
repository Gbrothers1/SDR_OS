"""Pure-function reward terms for the LL-Launch primitive (spec §5).

Each function takes explicit tensors (no env state) so it is unit-testable on
CPU. The env's reward methods are thin wrappers that pass self.* tensors here.
All terms are exploit-resistant by construction: there is no bar, and 'leave the
ground / reach the commanded apex / land doglike' is the only path to reward.
"""

import torch

# Sensitivity constants (exp(-k x^2) shaping).
# K_APEX=30 (v44.0.2, was 15): preserves climbable gradient near target while disqualifying
# low/belly apices when paired with the hard-gate (apex_match=0 if z_apex < h*-0.05).
#   0.05 m miss -> exp(-30*0.0025) = 0.928 (+, climbable near-target gradient kept)
#   0.20 m miss -> exp(-30*0.04)   = 0.301 (lower; hard gate at 0.05m is the primary block)
#   0.10 m miss -> exp(-30*0.01)   = 0.741 (reasonable gradient at 0.10m miss)
# K_VX unchanged.
K_APEX = 30.0     # apex height error (m)  -- v44.0.2: raised from 15; pairs with hard gate
K_VX = 2.0        # forward speed error (m/s)

# Hard gate: apex must reach within this margin of h* for any apex_match reward.
# Disqualifies belly-splay "apices" and micro-bounce peaks well below the target.
# v44.0.2: 0.05 m margin — a genuine attempt landing ≥5 cm below h* still gets signal.
APEX_HARD_GATE_MARGIN = 0.05  # m; apex_match = 0 if z_apex < h_star - APEX_HARD_GATE_MARGIN


def apex_match(z_apex, h_star, k=K_APEX, fired=None):
    """One-shot at apex: exp(-k (z_apex - h*)^2) with hard gate.

    v44.0.2 hard gate: returns 0 if z_apex < h_star - APEX_HARD_GATE_MARGIN.
    Disqualifies belly-splay and micro-bounce "apices" that are well below the
    target. The gradient is preserved for genuine attempts within 0.05 m of h*.
    Gated by `fired` (one-shot mask) if provided.
    """
    # Hard gate: zero out envs where the apex is more than the margin below h*.
    above_gate = (z_apex >= h_star - APEX_HARD_GATE_MARGIN).to(torch.float)
    r = torch.exp(-k * (z_apex - h_star) ** 2) * above_gate
    if fired is not None:
        r = r * fired.to(r.dtype)
    return r


def liftoff(all_feet_off, vz):
    """+1 when all four feet are off the ground AND base is ascending; else 0."""
    cond = all_feet_off & (vz > 0.0)
    return cond.to(torch.float)


def ascent_shaping(vz, airborne, below_target, flight_active=None):
    """Dense gradient toward apex: reward upward velocity while airborne & below h*.

    v44.0.3 single-flight-window gate: when `flight_active` is provided, ascent_shaping
    is only eligible during the ONE sanctioned launch arc — i.e.
    `flight_active = _liftoff_seen & ~_landing_credited`. v44.0.2 gated only on
    _liftoff_seen, which left a post-landing hole: after the first genuine
    liftoff+landing, repeated micro-bounces still farmed ascent (the latch stays
    True). Gating on `~_landing_credited` too means once the episode's genuine
    landing has fired, no further flight reward is earned — bounces pay nothing.
    Duration-independent (a 10cm bounce and a Stage-A jump have near-identical
    airborne time, ~14 vs ~15 steps, so a duration threshold cannot separate them).
    """
    gate = airborne & below_target
    if flight_active is not None:
        gate = gate & flight_active
    return torch.clamp(vz, min=0.0) * gate.to(torch.float)


def forward_vel_track(vx, vx_star, airborne, k=K_VX, flight_active=None):
    """exp(-k (vx - vx*)^2) while airborne; 0 on the ground.

    v44.0.3: optional `flight_active` single-flight-window gate (the same
    `_liftoff_seen & ~_landing_credited` mask used by ascent_shaping). Closes the
    red-team's 4.55x bounce-farm: forward_vel_track fired on EVERY airborne step,
    so ~10 post-landing hops/episode each earned forward-velocity reward. With the
    gate, forward velocity is rewarded only during the single launch arc; repeated
    bounces (which occur after the episode's genuine landing) earn nothing.
    """
    r = torch.exp(-k * (vx - vx_star) ** 2)
    gate = airborne
    if flight_active is not None:
        gate = gate & flight_active
    return r * gate.to(r.dtype)


def posture_flight(gz, foot_asymmetry, airborne):
    """Penalize non-upright (gz != -1) and L/R foot asymmetry during flight.

    gz is projected-gravity z (-1 = perfectly upright). foot_asymmetry >= 0.
    Returns a (mostly negative) shaping term, 0 on the ground.
    """
    tilt = (gz + 1.0) ** 2            # 0 when upright, grows as it tilts
    pen = -(2.0 * tilt + 1.0 * foot_asymmetry)
    return pen * airborne.to(pen.dtype)


def landing(all_feet_down, landing_pitch, z_recovered, just_landed):
    """One-shot at landing: reward 4-foot contact + low pitch + height recovery."""
    flat = all_feet_down.to(torch.float)
    pitch_ok = torch.exp(-8.0 * landing_pitch ** 2)        # <0.20 rad ~ 0.73+
    height_ok = torch.clamp(z_recovered / 0.31, 0.0, 1.0)  # v44.0.3: 0.35->0.31, actual URDF standing CoM (was saturating at 0.88)
    r = flat * (pitch_ok + height_ok)
    return r * just_landed.to(r.dtype)


# ── v44.0.6: break the alive-farming standing attractor (v44.0.5-G9 failure) ───
# G9 showed the policy converged to standing-still farming the ungated `alive`
# bonus (0.1/step ~ 20/episode), because no gradient led a grounded policy toward
# leaving the ground. Two coupled fixes: gate `alive` on a genuine liftoff (Lever 1)
# and add a telescoping height-progress gradient (Lever 2). See
# knowledge_base/experiments/v44.0.6-break-alive-attractor-spec.md.
_ALIVE_FLOOR = 0.3       # alive multiplier while NO liftoff yet (standing earns only this)
_LIFTOFF_FLOOR_Z = 0.315  # v44.0.8: 0.33->0.315. The 0.33 floor sat ~2cm above the standing CoM
                          # (~0.31), so the gradient region was UNREACHABLE from the standing basin
                          # (liftoff_progress=0 the whole v44.0.7 run). 0.315 is just above standing,
                          # so the robot's exploratory rises cross it and the on-ramp fires.
# v44.0.7: baseline the liftoff-progress running peak within this many grounded steps even if the
# robot never dips to <= _LIFTOFF_FLOOR_Z (robust fallback; the v44.0.6 settle-only trigger could
# starve if the robot stood rigidly above the floor or fell before settling).
_PROGRESS_BASELINE_GROUNDED_STEPS = 10


def alive_gated(liftoff_seen, floor=_ALIVE_FLOOR):
    """Alive bonus gated on having attempted a liftoff this episode.

    Standing-forever earns only `floor` (×env weight); once `_liftoff_seen` latches
    (a genuine all-feet-off, ascending liftoff) the full bonus is unlocked for the
    rest of the episode. This inverts the standing attractor: achieving even one
    liftoff is strongly rewarded, while standing without ever jumping is not.
    Returns a per-env scalar in [floor, 1.0].
    """
    return floor + (1.0 - floor) * liftoff_seen.to(torch.float)


def liftoff_progress(z, running_peak, baseline_set, floor=_LIFTOFF_FLOOR_Z):
    """Telescoping per-step reward for setting a NEW episode height record above `floor`.

    reward = clamp(z - max(running_peak, floor), 0) gated on `baseline_set`; the
    running peak is then raised to z. Farm-resistant by construction: only height
    ABOVE the running peak (and above the floor) pays, so bobbing, crouch-pump, and
    post-landing bounces earn 0 (the peak holds). Episode total telescopes to
    weight × (episode_peak - max(baseline, floor)). `baseline_set` is False until the
    robot's first ground contact, so the spawn height (0.4) and the RSI seed (0.65)
    never pay. Returns (reward, new_running_peak).
    """
    bar = torch.clamp(running_peak, min=floor)   # elementwise max(running_peak, floor), no alloc
    reward = torch.clamp(z - bar, min=0.0) * baseline_set.to(z.dtype)
    # Only advance the running peak once baselined, so the spawn/RSI descent (pre-
    # baseline) never raises the bar.
    new_peak = torch.where(baseline_set, torch.maximum(running_peak, z), running_peak)
    return reward, new_peak


# Minimum expected CoM height while the robot is standing/grounded (m).
# Below this threshold the robot is belly-collapsed or splayed.
# v44.0.3: 0.25 -> 0.22. URDF FK + smoke telemetry show the actual settled/crouch
# CoM floor is ~0.244 m, so 0.25 was penalizing LEGITIMATE crouch-load (suppressing
# liftoff -> liftoff=0 in the v44.0.2 smoke). 0.22 sits between the belly-splay
# ceiling (~0.19 m, firmly penalized) and the crouch floor (~0.244 m, unpenalized):
#   belly-splay z=0.17, weight 2.0, 180 steps -> 2.0*180*0.227 = -82 (closed)
#   shallow splay z=0.19                       -> 2.0*180*0.136 = -49 (closed)
#   legit crouch z=0.244 (> 0.22)              -> height_pen 0 (NOT penalized)
_GROUNDED_MIN_Z = 0.22


# v44.0.9: crouch-load on-ramp (Atanassov 2024, arXiv:2401.16337 — same Go1 hardware). The
# v44.0.8 longer run reverted to standing because the robot leapt WITHOUT loading (extend-and-
# topple) and the uncontrolled jumps fell. This term rewards a genuine pre-jump load crouch so the
# policy learns load-and-spring (controlled, landable take-off). One-shot, banded above the belly
# floor, upright-gated, and gated on a prior stand — see v44.0.9 spec §2/§4 for the farm analysis.
_CROUCH_TARGET_Z = 0.25    # m; centre of the load band (mild reward peak)
_CROUCH_TRIGGER_Z = 0.27   # m; upper band edge (below standing CoM 0.31)
_CROUCH_FLOOR_Z = 0.24     # m; lower band edge (2 cm ABOVE the belly-splay floor 0.22 — no collapse reward)
_CROUCH_SPRING_VZ = 0.15   # m/s; v44.0.9b: credit only while RISING out of the load (the spring),
                           # not on the descent/dip — couples the crouch to the upward launch.
K_CROUCH = 60.0            # band shaping (mild centring on the 0.25 target; the band+spring select the load)


def crouch_load(z, fired, target=_CROUCH_TARGET_Z, k=K_CROUCH):
    """One-shot load-and-spring reward: exp(-k*(z-target)^2) gated by `fired`, ~flat across the band.

    `fired` is the env's one-shot mask (grounded AND stood-first AND upright AND z in the
    [_CROUCH_FLOOR_Z, _CROUCH_TRIGGER_Z] band AND RISING out of it (vz>_CROUCH_SPRING_VZ) AND before
    the first liftoff AND not yet credited). It rewards SPRINGING UP out of a genuine load crouch —
    not the dip itself — so it cannot be farmed by stand→dip→re-stand (the v44.0.8 attractor in
    disguise) and a post-landing bounce (after liftoff) earns nothing. The band selects the load; the
    exp only mildly centres on 0.25 (it is NOT a depth grade — K=60 is ~flat over the 3 cm band).
    """
    return torch.exp(-k * (z - target) ** 2) * fired.to(z.dtype)


def grounded_posture(z, gz, airborne):
    """Dense per-step penalty for belly-collapse while grounded.

    Fires whenever the robot is NOT airborne and either:
    - base CoM is below _GROUNDED_MIN_Z (belly-splay / chest-on-floor)
    - projected gravity z deviates from -1 (non-upright) while grounded

    Returns a negative scalar in [-1.5, 0] per env per step (env weight is 2.0 as
    of v44.0.2). Accumulates over the many grounded steps in a belly-splay episode,
    making the belly-flop attractor strongly negative vs the alive bonus.
    """
    grounded = (~airborne).to(torch.float)
    # Penalty for low height: 0 at or above threshold, -1 at z=0
    height_pen = torch.clamp((_GROUNDED_MIN_Z - z) / _GROUNDED_MIN_Z, 0.0, 1.0)
    # Penalty for tilt: 0 when upright (gz=-1), up to 1 when horizontal (gz=0)
    tilt_pen = (gz + 1.0) ** 2   # 0 when gz=-1, 1 when gz=0, 4 when gz=+1
    tilt_pen = torch.clamp(tilt_pen, 0.0, 1.0)
    return -(height_pen + 0.5 * tilt_pen) * grounded
