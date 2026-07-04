"""
Go2 hurdle-mode bridge environment for the SDR_OS sim runner.

Wraps the VALIDATED v46.2 LL-Hurdle training env (rl/v46/envs/go2_hurdle_env.py,
eval 9/9 PASS) for live driving. The observation pipeline (50 dims/frame x 5 =
250) is inherited UNCHANGED — bit-exact with training.

Live behavior: the robot spawns on the run-up, approaches and clears the
physical bar, and the episode times out ~7 s later for an automatic respawn —
a continuous approach-jump-clear loop you can steer:

- left stick Y  -> approach speed vx* in the VALIDATED band [0.6, 1.0]
- right stick Y -> bar height: thirds map to the validated stages
                   (down = 0.10 m, center = 0.15 m, up = 0.20 m)

Bridge deltas (control/UX only): streaming camera; gamepad latch instead of
random command resampling; the training fall_over (50° instantaneous) replaced
by the launch-bridge grounded-only sustained down-detection so airborne
maneuvers never reset mid-jump.
"""

import torch
import genesis as gs

from rl.v46.envs.go2_hurdle_env import Go2HurdleEnv
from rl.v46.envs.hurdle_command_manager import BAR_STAGES
from src.sdr_os.envs.bridge_control import BridgeControlMixin

VX_MIN, VX_MAX = 0.6, 1.0   # validated eval band

_DOWN_GZ_THRESHOLD = -0.342   # cos(70°)
_DOWN_GRACE_STEPS = 25        # 0.5 s at 50 Hz

OBS_GROUPS_HURDLE = [
    ("command", 2),
    ("velocity_cmd", 3),
    ("angle_velocity", 3),
    ("projected_gravity", 3),
    ("dof_position", 12),
    ("dof_velocity", 12),
    ("actions", 12),
    ("base_height", 1),
    ("bar_obs", 2),
]


class Go2HurdleBridgeEnv(BridgeControlMixin, Go2HurdleEnv):
    """v46.2 hurdle env configured for bridge streaming and gamepad command."""

    def __init__(self, num_envs: int = 1, dt: float = 1 / 50, headless: bool = True,
                 camera_res: tuple = (1280, 720), max_episode_length_s: float = 12.0):
        super().__init__(num_envs=num_envs, dt=dt,
                         max_episode_length_s=max_episode_length_s, headless=headless)
        self._curriculum_stage = 3  # default: the hardest validated bar (0.20 m)
        self.obs_groups = OBS_GROUPS_HURDLE
        self.camera = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0), lookat=(-0.5, 0.0, 0.2),
            res=camera_res, fov=40, env_idx=0)
        self._gamepad_vx = VX_MIN
        self._gamepad_stage = 3

    def config(self):
        super().config()
        # Pin the vestigial velocity_cmd obs to the gamepad vx (v44-bridge pattern).
        self._vel_cmd_buf = torch.zeros((self.num_envs, 3), device=gs.device)
        self.velocity_command.use_external_controller(lambda step: self._vel_cmd_buf)
        # Loosened live reset (same rationale as the launch bridge): grounded-only,
        # sustained down-detection; mid-air maneuvers never reset.
        from genesis_forge.managers.config.config_item import TerminationConfigItem

        self._down_run = torch.zeros(self.num_envs, dtype=torch.long, device=gs.device)
        self.termination_manager.term_cfg["fall_over"] = TerminationConfigItem(
            {"fn": self._t_down_sustained}, self
        )

    def _t_down_sustained(self, env):
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        down = (gz > _DOWN_GZ_THRESHOLD) & ~self._airborne
        self._down_run = torch.where(
            down, self._down_run + 1, torch.zeros_like(self._down_run))
        return self._down_run >= _DOWN_GRACE_STEPS

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        if hasattr(self, "_down_run"):
            if envs_idx is None:
                self._down_run.zero_()
            else:
                self._down_run[envs_idx] = 0
        self._apply_gamepad_command()
        return result

    # ── bridge command interface ──────────────────────────────────────────────
    def set_velocity_from_gamepad(self, cmd_data: dict):
        """linear_y (positive half) -> vx* in [0.6, 1.0] (validated band);
        angular_y thirds -> bar stage 1/2/3 (0.10/0.15/0.20 m)."""
        ly = max(-1.0, min(1.0, float(cmd_data.get("linear_y", 0.0))))
        ay = max(-1.0, min(1.0, float(cmd_data.get("angular_y", 0.0))))
        self._gamepad_vx = VX_MIN + max(0.0, ly) * (VX_MAX - VX_MIN)
        stage = 1 if ay < -0.33 else (3 if ay > 0.33 else 2)
        if stage != self._gamepad_stage:
            self._gamepad_stage = stage
            self.set_curriculum_stage(stage)   # moves the physical bar
        self._apply_gamepad_command()

    def _apply_gamepad_command(self):
        cmd_mgr = getattr(self, "hurdle_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return
        # h* stays bar-coupled (manager resample handles it); pin vx.
        cmd_mgr.command[:, 1] = self._gamepad_vx
        if getattr(self, "_vel_cmd_buf", None) is not None:
            self._vel_cmd_buf[:, 0] = self._gamepad_vx

    def zero_velocity(self):
        self._gamepad_vx = VX_MIN  # slowest validated approach (HOLD uses stand branch)
        self._apply_gamepad_command()

    def get_velocity_command(self) -> dict:
        cmd_mgr = getattr(self, "hurdle_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return {}
        return {
            "mode": "hurdle",
            "bar_top": float(self.bar_top),
            "apex_height": float(cmd_mgr.command[0, 0]),
            "lin_vel_x": float(cmd_mgr.command[0, 1]),
        }
