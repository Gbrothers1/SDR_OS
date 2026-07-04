"""LaunchCommandManager — samples per-env (apex_height h*, forward_speed vx*).

Mirrors the genesis_forge command-manager lifecycle (allocate/reset/step/
observation) used by GaitCommandManager, but for the launch task's 2-D command.
The curriculum widens the apex band over time via set_curriculum_range(); every
sample is passed through the §5.1 feasibility clamp so an unreachable target is
never commanded (the v39.5 failure mode).

Decoupled from Genesis for unit-testing: pass device="cpu" and call allocate()
directly. Inside the env, the env calls allocate() in build() and resample()
from reset().
"""

import torch
from rl.v44.envs.launch_feasibility import clamp_apex_command


class LaunchCommandManager:
    def __init__(self, env, device: str | None = None, resample_time_sec: float = 10.0):
        self.env = env
        self.num_envs = env.num_envs
        self._device = device  # None -> resolved to gs.device at allocate() in-sim
        self._resample_time_sec = resample_time_sec
        # Curriculum band (overwritten by set_curriculum_range in build()). v44.0.8: defaults
        # match APEX_STAGES[1] (0.38/0.42) so the first allocate() resample is not stale.
        self._h_low = 0.38
        self._h_high = 0.42
        self._vx_high = 0.0
        self.command = None  # (num_envs, 2): [h*, vx*]

    def allocate(self):
        dev = self._device
        if dev is None:
            import genesis as gs
            dev = gs.device
        self._device = dev
        self.command = torch.zeros((self.num_envs, 2), device=dev)
        self.resample(torch.arange(self.num_envs, device=dev))

    def set_curriculum_range(self, h_low: float, h_high: float, vx_high: float):
        """Set the apex band [h_low, h_high] and the max forward speed vx_high."""
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

    def reset(self, env_ids: torch.Tensor | None = None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.command.device)
        self.resample(env_ids)

    def step(self):
        # Command is held for the whole episode (single launch); no mid-episode
        # resample. Method exists for lifecycle parity with other managers.
        pass

    def observation(self, env) -> torch.Tensor:
        return self.command
