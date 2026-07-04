"""DirectedLaunchCommandManager — (apex h*, forward vx*, jump trigger).

v44.1: adds the TRIGGER channel that makes the jump controller-commandable
(design: artifacts/knowledge_base/experiments/v44.1-directed-launch-design.md).
Per episode: trigger is 0 for a random 0.5–2.5 s STAND phase, then flips to 1
and latches. The policy is rewarded for standing before the trigger, jumping
on it, and recovering to a stand after — random timing is the controllability
teacher.

(h*, vx*) sampling and the feasibility clamp are inherited from the validated
v44 LaunchCommandManager semantics.
"""

import torch
from rl.v44.envs.launch_feasibility import clamp_apex_command

# Trigger timing band (steps at dt=1/50): 0.5–2.5 s into the episode.
TRIGGER_STEP_LOW = 25
TRIGGER_STEP_HIGH = 125


class DirectedLaunchCommandManager:
    def __init__(self, env, device: str | None = None):
        self.env = env
        self.num_envs = env.num_envs
        self._device = device
        # Curriculum band (set_curriculum_range, as v44).
        self._h_low = 0.38
        self._h_high = 0.42
        self._vx_high = 0.0
        self.command = None        # (num_envs, 3): [h*, vx*, trigger]
        self._trigger_step = None  # (num_envs,) long

    def allocate(self):
        dev = self._device
        if dev is None:
            import genesis as gs
            dev = gs.device
        self._device = dev
        self.command = torch.zeros((self.num_envs, 3), device=dev)
        self._trigger_step = torch.zeros(self.num_envs, dtype=torch.long, device=dev)
        self.resample(torch.arange(self.num_envs, device=dev))

    def set_curriculum_range(self, h_low: float, h_high: float, vx_high: float):
        self._h_low = float(h_low)
        self._h_high = float(h_high)
        self._vx_high = float(vx_high)

    def resample(self, env_ids: torch.Tensor):
        n = len(env_ids)
        if n == 0:
            return
        dev = self.command.device
        h = torch.empty(n, device=dev).uniform_(self._h_low, self._h_high)
        vx = torch.empty(n, device=dev).uniform_(0.0, max(self._vx_high, 0.0))
        h = clamp_apex_command(h, vx)
        self.command[env_ids, 0] = h
        self.command[env_ids, 1] = vx
        self.command[env_ids, 2] = 0.0
        self._trigger_step[env_ids] = torch.randint(
            TRIGGER_STEP_LOW, TRIGGER_STEP_HIGH + 1, (n,), device=dev)

    def reset(self, env_ids: torch.Tensor | None = None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.command.device)
        self.resample(env_ids)

    def step(self):
        # Latch the trigger once the per-env random step is reached.
        ep = self.env.episode_length
        if ep.dtype != self._trigger_step.dtype:
            ep = ep.to(self._trigger_step.dtype)
        self.command[:, 2] = torch.maximum(
            self.command[:, 2], (ep >= self._trigger_step).to(self.command.dtype))

    @property
    def triggered(self) -> torch.Tensor:
        """(num_envs,) bool — trigger latched."""
        return self.command[:, 2] > 0.5

    def observation(self, env) -> torch.Tensor:
        return self.command
