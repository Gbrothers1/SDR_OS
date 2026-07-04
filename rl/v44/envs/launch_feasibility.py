"""Physical feasibility envelope for the Go2 ballistic-launch primitive.

All numbers are from the math-expert feasibility pass (2026-06-08), recorded in
docs/superpowers/specs/2026-06-08-v44-ll-launch-primitive-design.md §5.1, derived
from the repo's physical params (mass 15.02 kg, leg reach 0.426 m, all joints
capped at 23.7 N·m, kp=20/kv=0.5).

The apex ceiling falls as forward speed rises because the fixed leg-energy budget
(~40 J net) is split between vertical and forward kinetic energy. We model the
ceiling as a linear interpolation between the measured vx=0 and vx=1.5 endpoints,
which is conservative (the true curve is slightly convex), and apply a hard cap.
"""

import torch

STANDING_COM_HEIGHT = 0.31      # m, v44.0.3: 0.35->0.31, actual URDF default-pose CoM (sdr-dynamics/sdr-simulation FK). Documentation constant — apex_ceiling() does not use it; landing() now divides by 0.31 directly.
APEX_CEILING_VERTICAL = 0.54    # m, achievable CoM apex at vx=0
APEX_CEILING_AT_VX15 = 0.43     # m, achievable CoM apex at vx=1.5 m/s
HARD_APEX_CAP = 0.57            # m, never command above this regardless of vx
VX_REF = 1.5                    # m/s, the high endpoint used for interpolation
APEX_TOLERANCE = 0.02           # m, meaningful command/eval resolution


def apex_ceiling(vx: torch.Tensor) -> torch.Tensor:
    """Per-element feasible apex ceiling (m) as a function of forward speed (m/s)."""
    frac = torch.clamp(vx / VX_REF, 0.0, 1.0)
    ceil = APEX_CEILING_VERTICAL + frac * (APEX_CEILING_AT_VX15 - APEX_CEILING_VERTICAL)
    return torch.clamp(ceil, max=HARD_APEX_CAP)


def clamp_apex_command(h: torch.Tensor, vx: torch.Tensor) -> torch.Tensor:
    """Clamp a commanded apex height to the feasible envelope for its forward speed.

    Args:
        h:  commanded apex heights (m), any shape.
        vx: commanded forward speeds (m/s), broadcastable to h.
    Returns:
        Feasibility-clamped apex heights, same shape as h.
    """
    return torch.minimum(h, apex_ceiling(vx))
