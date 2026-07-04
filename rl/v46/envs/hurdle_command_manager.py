"""HurdleCommandManager — samples per-env (apex_height h*, forward_speed vx*)
coupled to the current bar height.

Clone of the validated v44 LaunchCommandManager lifecycle (allocate/reset/step/
observation). The difference: h* is derived from the stage's bar top plus a
clearance margin (the commanded apex is always consistent with the obstacle),
and every sample passes the §5.1 feasibility clamp so an unreachable apex is
never commanded.

Decoupled from Genesis for unit-testing: pass device="cpu" and call allocate().
"""

import torch
from rl.v44.envs.launch_feasibility import clamp_apex_command

# Design §curriculum: bar top per stage; h* = bar_top + U(margin band); vx band.
BAR_STAGES = {
    1: {"bar_top": 0.10},
    2: {"bar_top": 0.15},
    3: {"bar_top": 0.20},
}
_H_MARGIN_LOW = 0.22   # m above bar_top — CoM apex needs body + tucked-leg clearance
_H_MARGIN_HIGH = 0.28
_VX_LOW = 0.5          # m/s — enough forward momentum to carry the crossing
_VX_HIGH = 1.0


class HurdleCommandManager:
    def __init__(self, env, device: str | None = None):
        self.env = env
        self.num_envs = env.num_envs
        self._device = device  # None -> resolved to gs.device at allocate()
        self._bar_top = BAR_STAGES[1]["bar_top"]
        self.command = None  # (num_envs, 2): [h*, vx*]

    @property
    def bar_top(self) -> float:
        return self._bar_top

    def allocate(self):
        dev = self._device
        if dev is None:
            import genesis as gs
            dev = gs.device
        self._device = dev
        self.command = torch.zeros((self.num_envs, 2), device=dev)
        self.resample(torch.arange(self.num_envs, device=dev))

    def set_curriculum_stage(self, stage: int):
        self._bar_top = BAR_STAGES[stage]["bar_top"]

    def resample(self, env_ids: torch.Tensor):
        n = len(env_ids)
        if n == 0:
            return
        dev = self.command.device
        h = self._bar_top + torch.empty(n, device=dev).uniform_(_H_MARGIN_LOW, _H_MARGIN_HIGH)
        vx = torch.empty(n, device=dev).uniform_(_VX_LOW, _VX_HIGH)
        h = clamp_apex_command(h, vx)
        self.command[env_ids, 0] = h
        self.command[env_ids, 1] = vx

    def reset(self, env_ids: torch.Tensor | None = None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.command.device)
        self.resample(env_ids)

    def step(self):
        # Command held for the episode (lifecycle parity with other managers).
        pass

    def observation(self, env) -> torch.Tensor:
        return self.command
