"""
Go2 directed-launch bridge environment — jump on the X button.

Wraps the v44.1 trained env (rl/v44/envs/go2_directed_launch_env.py) for live
driving. Obs pipeline (49 dims/frame x 5 = 245) inherited unchanged.

Validated envelope (eval_directed_launch, model_1999): P(jump|trigger) 96-100%,
P(spurious) 1-4%, recovery ~100%; apex tracking cm-level for h* >= ~0.46 and
OVERSHOOTS below that — so the gamepad apex axis is bounded to [0.46, 0.56].

Controls: hold L2 to run the policy (it stands quietly); X button = jump
(trigger pulses high for 1 s via set_jump_intent); right stick Y = apex within
the validated band; left stick Y = forward carry vx* in [0, 0.5] (validated).
"""

import torch
import genesis as gs

from rl.v44.envs.go2_directed_launch_env import Go2DirectedLaunchEnv
from src.sdr_os.envs.bridge_control import BridgeControlMixin

H_MIN, H_MAX = 0.46, 0.56    # validated apex band (below 0.46 it overshoots)
VX_MIN, VX_MAX = 0.0, 0.5    # validated vx cells
TRIGGER_PULSE_STEPS = 50     # 1 s trigger pulse per X press

_DOWN_GZ_THRESHOLD = -0.342  # cos(70°) — loosened live down-detection
_DOWN_GRACE_STEPS = 25

OBS_GROUPS_DIRECTED = [
    ("command", 3),
    ("velocity_cmd", 3),
    ("angle_velocity", 3),
    ("projected_gravity", 3),
    ("dof_position", 12),
    ("dof_velocity", 12),
    ("actions", 12),
    ("base_height", 1),
]


class Go2DirectedLaunchBridgeEnv(BridgeControlMixin, Go2DirectedLaunchEnv):
    def __init__(self, num_envs: int = 1, dt: float = 1 / 50, headless: bool = True,
                 camera_res: tuple = (1280, 720), max_episode_length_s: float = 3600.0):
        super().__init__(num_envs=num_envs, dt=dt,
                         max_episode_length_s=max_episode_length_s, headless=headless)
        self._curriculum_stage = 3
        self.obs_groups = OBS_GROUPS_DIRECTED
        self.camera = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0), lookat=(0.0, 0.0, 0.2),
            res=camera_res, fov=40, env_idx=0)
        self._gamepad_h = (H_MIN + H_MAX) / 2
        self._gamepad_vx = 0.0
        self._trigger_steps_left = 0

    def config(self):
        super().config()
        self._vel_cmd_buf = torch.zeros((self.num_envs, 3), device=gs.device)
        self.velocity_command.use_external_controller(lambda step: self._vel_cmd_buf)
        # Live driving owns the trigger — disable the training-time random
        # schedule entirely (it would fire spontaneous jumps mid-demo).
        self.launch_command._trigger_step[:] = 10 ** 9
        from genesis_forge.managers.config.config_item import TerminationConfigItem

        self._down_run = torch.zeros(self.num_envs, dtype=torch.long, device=gs.device)
        self.termination_manager.term_cfg["fall_over"] = TerminationConfigItem(
            {"fn": self._t_down_sustained}, self)

    def _t_down_sustained(self, env):
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        down = (gz > _DOWN_GZ_THRESHOLD) & ~self._airborne
        self._down_run = torch.where(
            down, self._down_run + 1, torch.zeros_like(self._down_run))
        return self._down_run >= _DOWN_GRACE_STEPS

    def step(self, actions):
        # Drive the trigger pulse (replaces the training random schedule, which
        # config() disabled). On pulse end the launch-arc latches reset so the
        # NEXT press is a fresh commanded jump.
        if self._trigger_steps_left > 0:
            self._trigger_steps_left -= 1
            self.launch_command.command[:, 2] = 1.0
            if self._trigger_steps_left == 0:
                self.launch_command.command[:, 2] = 0.0
                self._rearm_jump()
        return super().step(actions)

    def _rearm_jump(self):
        """Clear the per-episode one-shot latches so each X press is a fresh jump."""
        for t in (self._apex_fired, self._apex_credited, self._liftoff_seen,
                  self._liftoff_credited, self._landing_credited,
                  self._trigger_liftoff_seen, self._flight_closed,
                  self._crouch_credited):
            t.zero_()

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        if hasattr(self, "_down_run"):
            if envs_idx is None:
                self._down_run.zero_()
            else:
                self._down_run[envs_idx] = 0
        # Reset re-randomizes the trigger schedule — re-disable it.
        if getattr(self.launch_command, "_trigger_step", None) is not None:
            self.launch_command._trigger_step[:] = 10 ** 9
            self.launch_command.command[:, 2] = 1.0 if self._trigger_steps_left > 0 else 0.0
        self._apply_gamepad_command()
        return result

    # ── bridge command interface ──────────────────────────────────────────────
    def set_jump_intent(self, intensity: float = 1.0):
        """X button: pulse the trigger for 1 s (the policy jumps on command)."""
        if intensity > 0.1:
            self._trigger_steps_left = TRIGGER_PULSE_STEPS
            self.launch_command.command[:, 2] = 1.0

    def set_velocity_from_gamepad(self, cmd_data: dict):
        ly = max(-1.0, min(1.0, float(cmd_data.get("linear_y", 0.0))))
        ay = max(-1.0, min(1.0, float(cmd_data.get("angular_y", 0.0))))
        self._gamepad_vx = max(0.0, ly) * VX_MAX
        self._gamepad_h = (H_MIN + H_MAX) / 2 + ay * (H_MAX - H_MIN) / 2
        self._apply_gamepad_command()

    def _apply_gamepad_command(self):
        cmd_mgr = getattr(self, "launch_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return
        cmd_mgr.command[:, 0] = self._gamepad_h
        cmd_mgr.command[:, 1] = self._gamepad_vx
        if getattr(self, "_vel_cmd_buf", None) is not None:
            self._vel_cmd_buf[:, 0] = self._gamepad_vx

    def zero_velocity(self):
        self._gamepad_vx = 0.0
        self._trigger_steps_left = 0
        cmd_mgr = getattr(self, "launch_command", None)
        if cmd_mgr is not None and cmd_mgr.command is not None:
            cmd_mgr.command[:, 2] = 0.0
        self._apply_gamepad_command()

    def get_velocity_command(self) -> dict:
        cmd_mgr = getattr(self, "launch_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return {}
        return {
            "mode": "directed_launch",
            "apex_height": float(cmd_mgr.command[0, 0]),
            "lin_vel_x": float(cmd_mgr.command[0, 1]),
            "trigger": float(cmd_mgr.command[0, 2]),
        }
