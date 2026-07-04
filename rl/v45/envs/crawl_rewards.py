"""Pure-function reward terms for the LL-Crawl primitive (v45.0 spec §3).

Height-commanded locomotion: track commanded body height h* AND forward speed vx*
simultaneously. The tracking reward is MULTIPLICATIVE (spec §2) so neither component
can be farmed alone — standing tall at the right speed, or holding height while
stationary, both earn ~0. Constraints (virtual ceiling, belly-drag) are enforced by
CaT terminations in the env (arXiv:2403.18765), NOT penalties — the v43 postmortem's
"penalty stacking relocates the exploit" lesson, formalized by the field.

Each function takes explicit tensors (no env state) so it is unit-testable on CPU.
Sources: Zhuang et al. 2023 (arXiv:2309.05665) crawl Table 5 (energy alpha_4=2e-5);
Atanassov et al. 2024 (arXiv:2401.16337) exp height-target form.
"""

import torch

# Tracking sharpness (spec §3). sigma_h = 8e-4 => |z-h*|=3cm gives exp(-1.125)~0.32;
# 1cm gives ~0.88. sigma_v = 0.25 => |vx-vx*|=0.5 gives exp(-1)~0.37.
SIGMA_H = 8e-4    # m^2 — height tracking tolerance scale
SIGMA_V = 0.25    # (m/s)^2 — velocity tracking tolerance scale

# CaT thresholds (read by the env's termination fns; here for unit-testing).
_CEILING_MARGIN = 0.06        # m above h* before the ceiling CaT arms
_CEILING_GRACE_STEPS = 5      # consecutive violation steps before termination
_BELLY_FORCE_N = 5.0          # N on the base link = belly contact
_BELLY_GRACE_STEPS = 10       # consecutive belly-contact steps (while moving) before termination
_BELLY_MIN_VX = 0.2           # m/s — belly-drag is only an exploit when sliding forward

# Posture: crawling pitches the body; allow more tilt than the launch env's gz<=-0.9.
_POSTURE_GZ_FREE = -0.85      # no penalty while gz <= this (mostly upright)


def track(z, h_star, vx, vx_star, sigma_h=SIGMA_H, sigma_v=SIGMA_V):
    """Multiplicative height x velocity tracking: exp(-|z-h*|^2/sh) * exp(-(vx-vx*)^2/sv).

    Multiplicative by design: a stand-tall walk at commanded speed earns ~0 (height factor
    kills it), and holding height while stationary earns ~0 (velocity factor kills it).
    Only genuine low locomotion pays. Max 1.0 per step.
    """
    r_h = torch.exp(-((z - h_star) ** 2) / sigma_h)
    r_v = torch.exp(-((vx - vx_star) ** 2) / sigma_v)
    return r_h * r_v


def energy(torques, dof_vel, coef=2e-5):
    """Parkour crawl energy penalty (alpha_4 = 2e-5, 10x the climb value — Zhuang Table 5).

    Returns NEGATIVE sum(|tau * qdot|) * coef per env. Suppresses flailing without
    shaping the gait by hand.
    """
    return -coef * torch.sum(torch.abs(torques * dof_vel), dim=-1)


def posture(gz, free=_POSTURE_GZ_FREE):
    """Soft uprightness penalty, looser than the launch env (crawl pitches the body).

    0 while gz <= free (-0.85); grows quadratically as the body tilts past it. Returns <= 0.
    """
    viol = torch.clamp(gz - free, min=0.0)
    return -(viol ** 2)


def ceiling_violation(z, h_star, margin=_CEILING_MARGIN):
    """Bool: body above the virtual ceiling (z > h* + margin). The env counts consecutive
    violations and CaT-terminates after _CEILING_GRACE_STEPS — a structural constraint,
    not a farmable penalty."""
    return z > (h_star + margin)


def belly_drag(base_contact_force, vx, force_n=_BELLY_FORCE_N, min_vx=_BELLY_MIN_VX):
    """Bool: belly (base link) in contact while sliding forward — the crawl exploit family
    (drag chest, paddle). The env counts consecutive hits and CaT-terminates after
    _BELLY_GRACE_STEPS. Stationary belly contact (resting) is NOT flagged."""
    return (base_contact_force > force_n) & (vx.abs() > min_vx)
