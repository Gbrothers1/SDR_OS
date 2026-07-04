"""CrawlCommandManager — samples per-env (body_height h*, forward_speed vx*).

Clone of the validated v44 LaunchCommandManager lifecycle (allocate/reset/step/
observation), for the crawl task's 2-D command. The curriculum LOWERS the height
band over stages (crawl gets harder downward — the reachable-gradient lesson:
stage 1 starts a slight crouch from the 0.31 stand).

Decoupled from Genesis for unit-testing: pass device="cpu" and call allocate().
"""

import torch

# Spec §2: descending height curriculum. Floor edge 0.18 stays above belly-on-floor
# (~0.16); stage-1 low edge 0.26 is directly reachable from the 0.31 stand.
CRAWL_STAGES = {
    1: {"h_low": 0.26, "h_high": 0.28, "vx_low": 0.3, "vx_high": 0.6},
    2: {"h_low": 0.22, "h_high": 0.26, "vx_low": 0.3, "vx_high": 0.8},
    3: {"h_low": 0.18, "h_high": 0.24, "vx_low": 0.3, "vx_high": 1.0},
}


class CrawlCommandManager:
    def __init__(self, env, device: str | None = None):
        self.env = env
        self.num_envs = env.num_envs
        self._device = device  # None -> resolved to gs.device at allocate()
        s = CRAWL_STAGES[1]
        self._h_low, self._h_high = s["h_low"], s["h_high"]
        self._vx_low, self._vx_high = s["vx_low"], s["vx_high"]
        self.command = None  # (num_envs, 2): [h*, vx*]

    def allocate(self):
        dev = self._device
        if dev is None:
            import genesis as gs
            dev = gs.device
        self._device = dev
        self.command = torch.zeros((self.num_envs, 2), device=dev)
        self.resample(torch.arange(self.num_envs, device=dev))

    def set_curriculum_stage(self, stage: int):
        s = CRAWL_STAGES[stage]
        self._h_low, self._h_high = s["h_low"], s["h_high"]
        self._vx_low, self._vx_high = s["vx_low"], s["vx_high"]

    def resample(self, env_ids: torch.Tensor):
        n = len(env_ids)
        if n == 0:
            return
        dev = self.command.device
        self.command[env_ids, 0] = torch.empty(n, device=dev).uniform_(self._h_low, self._h_high)
        self.command[env_ids, 1] = torch.empty(n, device=dev).uniform_(self._vx_low, self._vx_high)

    def reset(self, env_ids: torch.Tensor | None = None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.command.device)
        self.resample(env_ids)

    def step(self):
        # Command held for the episode (lifecycle parity with other managers).
        pass

    def observation(self, env) -> torch.Tensor:
        return self.command
