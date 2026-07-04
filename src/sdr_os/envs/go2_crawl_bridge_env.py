"""
Go2 crawl-mode bridge environment for the SDR_OS sim runner.

Wraps the VALIDATED v45 LL-Crawl training env (rl/v45/envs/go2_crawl_env.py)
for live gamepad driving. The observation pipeline — manager classes, term
ordering, scaling, 5-frame history (45 dims/frame -> 225 total) — is inherited
UNCHANGED so the policy obs is bit-exact with training; the env classes are
reused, never re-implemented.

Bridge deltas (control/UX only — nothing that feeds the policy obs is altered):
- streaming camera at the runner's resolution alongside the 480x360 training cams
- a gamepad latch drives (h*, vx*) instead of per-episode random resampling
- CaT terminations (ceiling / belly-drag) are disabled: they shape training,
  but in a live demo a height-command change transiently violates the ceiling
  and must not hard-reset the robot
- episode timeout stretched to an hour so the robot doesn't reset mid-drive
"""

import torch
import genesis as gs

from rl.v45.envs.go2_crawl_env import Go2CrawlEnv
from rl.v45.envs.crawl_command_manager import CRAWL_STAGES
from src.sdr_os.envs.bridge_control import BridgeControlMixin

# Gamepad command envelope — the full TRAINED envelope (stage-3 lows to stage-1
# highs). Commands outside these bands are out-of-distribution for model_799.
_S1, _S3 = CRAWL_STAGES[1], CRAWL_STAGES[3]
H_MIN, H_MAX = _S3["h_low"], _S1["h_high"]      # 0.18 .. 0.28 m
VX_MIN, VX_MAX = _S3["vx_low"], _S3["vx_high"]  # 0.3 .. 1.0 m/s

# Per-frame layout of the v45 policy obs (see Go2CrawlEnv.config) — used by
# get_obs_breakdown for UI telemetry.
OBS_GROUPS_CRAWL = [
    ("command", 2),
    ("angle_velocity", 3),
    ("projected_gravity", 3),
    ("dof_position", 12),
    ("dof_velocity", 12),
    ("actions", 12),
    ("base_height", 1),
]


class Go2CrawlBridgeEnv(BridgeControlMixin, Go2CrawlEnv):
    """v45 crawl env configured for bridge streaming and gamepad command."""

    def __init__(
        self,
        num_envs: int = 1,
        dt: float = 1 / 50,
        headless: bool = True,
        camera_res: tuple = (1280, 720),
        max_episode_length_s: float = 3600.0,
    ):
        super().__init__(
            num_envs=num_envs,
            dt=dt,
            max_episode_length_s=max_episode_length_s,
            headless=headless,
        )
        # Full trained envelope; build() propagates this to the command manager.
        self._curriculum_stage = 3
        self.obs_groups = OBS_GROUPS_CRAWL
        # Streaming camera at the bridge resolution; build() (via Go2CrawlEnv)
        # makes self.camera follow the robot.
        self.camera = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0),
            lookat=(0.0, 0.0, 0.0),
            res=camera_res,
            fov=40,
            env_idx=0,
        )
        # Gamepad latch — re-applied after every reset (reset resamples randomly).
        self._gamepad_h = H_MAX   # gentlest trained height (closest to stand)
        self._gamepad_vx = VX_MIN

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        self._apply_gamepad_command()  # override the training-time random resample
        return result

    # ── CaT disabled for live driving (see module docstring) ─────────────────
    def _t_ceiling(self, env):
        return torch.zeros(self.num_envs, dtype=torch.bool, device=gs.device)

    def _t_belly_drag(self, env):
        return torch.zeros(self.num_envs, dtype=torch.bool, device=gs.device)

    # ── bridge command interface (same contract as Go2BridgeEnv) ─────────────
    def set_velocity_from_gamepad(self, cmd_data: dict):
        """Map set_cmd_vel stick axes to the crawl command (h*, vx*).

        linear_y  (forward stick, positive half) -> vx* in [VX_MIN, VX_MAX]
        angular_y (height axis, -1..1)           -> h*  in [H_MIN, H_MAX]
                                                    (stick up = taller body)

        Both are clamped to the trained envelope. The crawl policy has no
        yaw/strafe command, so linear_x / angular_z are ignored in crawl mode.
        Stick at rest commands the slowest in-distribution crawl (VX_MIN).
        """
        ly = max(-1.0, min(1.0, float(cmd_data.get("linear_y", 0.0))))
        ay = max(-1.0, min(1.0, float(cmd_data.get("angular_y", 0.0))))
        h = (H_MIN + H_MAX) / 2 + ay * (H_MAX - H_MIN) / 2
        vx = VX_MIN + max(0.0, ly) * (VX_MAX - VX_MIN)
        # Stage-band coupling: tall crawls only trained at lower speeds
        # (stage 1: h<=0.28 @ vx<=0.6; stage 2: h<=0.26 @ vx<=0.8). Cap vx by
        # the commanded height so the (tall, fast) OOD corner is unreachable.
        if h > CRAWL_STAGES[2]["h_high"]:        # > 0.26
            vx = min(vx, CRAWL_STAGES[1]["vx_high"])   # <= 0.6
        elif h > CRAWL_STAGES[3]["h_high"]:      # > 0.24
            vx = min(vx, CRAWL_STAGES[2]["vx_high"])   # <= 0.8
        self._gamepad_h = h
        self._gamepad_vx = vx
        self._apply_gamepad_command()

    def _apply_gamepad_command(self):
        # crawl_command is created in config(); guard against pre-config calls.
        cmd_mgr = getattr(self, "crawl_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return
        cmd_mgr.command[:, 0] = self._gamepad_h
        cmd_mgr.command[:, 1] = self._gamepad_vx

    def zero_velocity(self):
        """HOLD/ESTOP zero-latch. vx*=0 is below the trained band — acceptable
        because the runner switches to the stand branch (policy not queried
        while held)."""
        self._gamepad_vx = 0.0
        self._apply_gamepad_command()

    def get_velocity_command(self) -> dict:
        """Current crawl command for env 0 (telemetry.training.metrics)."""
        cmd_mgr = getattr(self, "crawl_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return {}
        return {
            "mode": "crawl",
            "body_height": float(cmd_mgr.command[0, 0]),
            "lin_vel_x": float(cmd_mgr.command[0, 1]),
        }
