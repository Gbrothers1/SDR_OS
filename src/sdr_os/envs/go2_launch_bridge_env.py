"""
Go2 jump-mode bridge environment for the SDR_OS sim runner.

Wraps the VALIDATED v44 LL-Launch training env (rl/v44/envs/go2_launch_env.py)
for live gamepad driving. The observation pipeline — manager classes, term
ordering, scaling, 5-frame history (48 dims/frame -> 240 total) — is inherited
UNCHANGED so the policy obs is bit-exact with training; the env classes are
reused, never re-implemented.

Jump UX (navigation-suite spike): holding the gait trigger runs the launch
policy, which crouch-loads and launches to the commanded apex h*, then lands.
Releasing the trigger returns the runner to the stand branch. The apex height
is commanded from the same height axis used for crawl depth, so one stick
gesture means "how high/low" across skills.

Bridge deltas (control/UX only — nothing that feeds the policy obs is altered):
- streaming camera at the runner's resolution alongside the training cams
- a gamepad latch drives (h*, vx*) instead of per-episode random resampling;
  every command passes the same launch_feasibility clamp used in training
- the vestigial velocity_cmd obs (randomly resampled in training, uncorrelated
  with the reward target) is pinned to [vx*, 0, 0] via the external-controller
  hook instead of resampling
- episode timeout stretched to an hour so the robot doesn't teleport mid-drive
  (training used 4 s one-jump episodes); fall_over still auto-resets
"""

import torch
import genesis as gs

from rl.v44.envs.go2_launch_env import Go2LaunchEnv
from rl.v44.envs.launch_feasibility import clamp_apex_command
from src.sdr_os.envs.bridge_control import BridgeControlMixin

# Gamepad command envelope — the VALIDATED model_799 envelope, NOT the full
# training band. Per the checkpoint eval (artifacts/knowledge_base/experiments/
# v44.0.10-long-eval-results.md): apex err <5 cm and 100% landing hold for
# h* in [0.42, 0.48] x vx* in [0, 0.75]; vx*=1.5 collapses landing to 19-35%
# and h*=0.55 undershoots (above the 0.48-0.54 feasibility ceiling). The
# feasibility clamp additionally caps h* by vx*.
H_MIN, H_MAX = 0.40, 0.48    # m apex (0.40 = stage-3 low edge, in-band)
VX_MIN, VX_MAX = 0.0, 0.75   # m/s (0 is in-distribution)

# Live-drive down-detection (replaces the training-time 50° instantaneous
# bad_orientation, which fired MID-AIR during flips/twirls and reset the env
# before the maneuver finished). Terminate only when the robot is genuinely
# down: not airborne, tipped past 70 deg, sustained for half a second.
_DOWN_GZ_THRESHOLD = -0.342   # cos(70 deg) — projected-gravity z above this = tipped >70 deg
_DOWN_GRACE_STEPS = 25        # 0.5 s at 50 Hz

# Per-frame layout of the v44 policy obs (see Go2LaunchEnv.config) — used by
# get_obs_breakdown for UI telemetry.
OBS_GROUPS_LAUNCH = [
    ("command", 2),
    ("velocity_cmd", 3),
    ("angle_velocity", 3),
    ("projected_gravity", 3),
    ("dof_position", 12),
    ("dof_velocity", 12),
    ("actions", 12),
    ("base_height", 1),
]


class Go2LaunchBridgeEnv(BridgeControlMixin, Go2LaunchEnv):
    """v44 launch env configured for bridge streaming and gamepad command."""

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
        self.obs_groups = OBS_GROUPS_LAUNCH
        # Streaming camera at the bridge resolution; build() (via Go2LaunchEnv)
        # makes self.camera follow the robot.
        self.camera = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0),
            lookat=(0.0, 0.0, 0.0),
            res=camera_res,
            fov=40,
            env_idx=0,
        )
        # Gamepad latch — re-applied after every reset (reset resamples randomly).
        self._gamepad_h = H_MIN   # gentlest trained apex
        self._gamepad_vx = 0.0    # vertical jump by default
        self._vel_cmd_buf = None  # allocated in config()

    def config(self):
        super().config()
        # Pin the vestigial velocity_cmd obs (random in training) to the gamepad vx.
        self._vel_cmd_buf = torch.zeros((self.num_envs, 3), device=gs.device)
        self.velocity_command.use_external_controller(lambda step: self._vel_cmd_buf)
        # Loosen the reset for live driving: mid-air orientation must never
        # terminate (jump/flip/twirl maneuvers tilt well past the training 50°
        # limit), and a landing wobble gets a settle grace before reset.
        from genesis_forge.managers.config.config_item import TerminationConfigItem

        self._down_run = torch.zeros(self.num_envs, dtype=torch.long, device=gs.device)
        self.termination_manager.term_cfg["fall_over"] = TerminationConfigItem(
            {"fn": self._t_down_sustained}, self
        )

    def _t_down_sustained(self, env):
        """True only when genuinely down: not airborne, tipped >70°, for 0.5 s."""
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        down = (gz > _DOWN_GZ_THRESHOLD) & ~self._airborne
        self._down_run = torch.where(
            down, self._down_run + 1, torch.zeros_like(self._down_run)
        )
        return self._down_run >= _DOWN_GRACE_STEPS

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        if hasattr(self, "_down_run"):
            if envs_idx is None:
                self._down_run.zero_()
            else:
                self._down_run[envs_idx] = 0
        self._apply_gamepad_command()  # override the training-time random resample
        return result

    # ── bridge command interface (same contract as Go2BridgeEnv) ─────────────
    def set_velocity_from_gamepad(self, cmd_data: dict):
        """Map set_cmd_vel stick axes to the launch command (apex h*, vx*).

        linear_y  (forward stick, positive half) -> vx* in [0, VX_MAX]
        angular_y (height axis, -1..1)           -> h*  in [H_MIN, H_MAX]
                                                    (stick up = higher apex)

        Every command passes the training feasibility clamp (an unreachable
        apex is never commanded). The launch policy has no yaw/strafe command,
        so linear_x / angular_z are ignored in jump mode.
        """
        ly = max(-1.0, min(1.0, float(cmd_data.get("linear_y", 0.0))))
        ay = max(-1.0, min(1.0, float(cmd_data.get("angular_y", 0.0))))
        self._gamepad_vx = max(0.0, ly) * VX_MAX
        self._gamepad_h = (H_MIN + H_MAX) / 2 + ay * (H_MAX - H_MIN) / 2
        self._apply_gamepad_command()

    def _apply_gamepad_command(self):
        # launch_command is created in config(); guard against pre-config calls.
        cmd_mgr = getattr(self, "launch_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return
        dev = cmd_mgr.command.device
        h = torch.tensor([self._gamepad_h], device=dev)
        vx = torch.tensor([self._gamepad_vx], device=dev)
        h = clamp_apex_command(h, vx)
        cmd_mgr.command[:, 0] = h
        cmd_mgr.command[:, 1] = vx
        if self._vel_cmd_buf is not None:
            self._vel_cmd_buf[:, 0] = vx
            self._vel_cmd_buf[:, 1] = 0.0
            self._vel_cmd_buf[:, 2] = 0.0

    def zero_velocity(self):
        """HOLD/ESTOP zero-latch. vx*=0 is in-distribution for launch (training
        sampled vx uniformly from 0)."""
        self._gamepad_vx = 0.0
        self._apply_gamepad_command()

    def get_velocity_command(self) -> dict:
        """Current launch command for env 0 (telemetry.training.metrics)."""
        cmd_mgr = getattr(self, "launch_command", None)
        if cmd_mgr is None or cmd_mgr.command is None:
            return {}
        return {
            "mode": "launch",
            "apex_height": float(cmd_mgr.command[0, 0]),
            "lin_vel_x": float(cmd_mgr.command[0, 1]),
        }
