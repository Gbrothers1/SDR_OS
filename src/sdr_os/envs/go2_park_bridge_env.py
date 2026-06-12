"""
Go2 park-mode bridge environment for the SDR_OS sim runner.

Wraps the v48 unified park env (rl/v48/envs/go2_park_env.py) for live
operator driving.  The observation pipeline (79 dims/frame x 5 = 395) is
reproduced EXACTLY from the training env — bit-compatible with any checkpoint
trained against Go2ParkEnv.

Obs layout (79/frame × 5 = 395 actor):
  gait_command      14  (GaitCommandManager, rl/v42 canonical)
  velocity_command   3  (sticks -> vx/vy/wz)
  skill_cmd          3  (X/B/Y -> jump/crouch/climb, external=True)
  angular_velocity   3
  projected_gravity  3
  dof_pos/vel/act   36
  base_height        1  (terrain-relative z - h_ground; FLAT plane -> ~HEIGHT_OFFSET)
  obstacle_obs       8  (next-2 slots: dx,z_low,z_high,depth; NO_OBSTACLE padding)
  terrain_heights    5  (5-probe local heightmap; zeros on flat plane)
  is_physical        3  (per-kind flags; [1,1,1] at deploy)
  TOTAL             79  × 5 = 395

Scene: one flat-plane park (FLAT ONLY — no heightfield roughness, no entity
pools raised by default).  Entities are sunken/absent so the world is light for
1-env bridge use.  A small obstacle strip (2-3 ghost obstacles) may be raised
on demand for HUD testing (set_debug_course).

Controls:
  left stick Y       -> vx* (forward speed, [0.0, 2.0])
  right stick X      -> wz* (yaw rate, [-1.0, 1.0])
  L2 held            -> policy walk; L2 released -> IK stand
  D-pad left/right   -> set_gait cycling (runner re-applies every step)
  X button           -> jump pulse 40 steps
  B button (hold)    -> crouch hold
  Y button           -> climb pulse 75 steps

Gate A-0 note: the bridge is wired for ANY park checkpoint.  Until a real
park checkpoint exists (training is live), driving with walk-policy obs (310)
or a test checkpoint verifies the harness end-to-end.

QUIRKS from gate-a0-20260612-083007/report.md (applied here):
  Q1: set_fixed_gait re-applied every step (runner handles; bridge exposes
      gait_command_manager for the runner's step_sim loop).
  Q2: velocity_command re-pinned in reset (done in _apply_gamepad_command).
  Q3: skill_command.external=True disables all auto-sources.
  Q4: obs is a TensorDict from genesis_forge; bridge reads policy key directly.
"""

import sys
import os
from pathlib import Path

import torch
import genesis as gs

# ── Ensure training env is importable (same sys.path trick other bridges use)
_project_root = str(Path(__file__).resolve().parent.parent.parent.parent)
if _project_root not in sys.path:
    sys.path.insert(0, _project_root)

from rl.v48.envs.go2_park_env import (
    Go2ParkEnv,
    HEIGHT_OFFSET,
    NO_OBSTACLE_OBS,
    LEAD_IN,
    COURSE_LEN,
    HF_CELL,
    _VERT_SCALE,
)
from rl.v48.envs.park_skill_commands import (
    JUMP_PULSE_STEPS,
    CLIMB_PULSE_STEPS,
)
from src.sdr_os.envs.bridge_control import BridgeControlMixin

# Validated velocity range (same as training env config block).
VX_MIN, VX_MAX = 0.0, 2.0
VY_MIN, VY_MAX = -0.3, 0.3
WZ_MIN, WZ_MAX = -1.0, 1.0

_DOWN_GZ_THRESHOLD = -0.342   # cos(70 deg) — loosened down-detection
_DOWN_GRACE_STEPS = 25        # 0.5 s at 50 Hz

# D-pad gait cycle order (matches Gate A-0 contract).
_GAIT_CYCLE = ["walk", "trot", "pace", "bound", "pronk"]
_GAIT_DEFAULT_PERIODS = {
    "walk": 0.50, "trot": 0.45, "pace": 0.50, "bound": 0.45, "pronk": 0.45,
}
_GAIT_DEFAULT_CLEARANCE = 0.08

OBS_GROUPS_PARK = [
    ("gait_command",      14),
    ("velocity_command",   3),
    ("skill_cmd",          3),
    ("angle_velocity",     3),
    ("projected_gravity",  3),
    ("dof_position",      12),
    ("dof_velocity",      12),
    ("actions",           12),
    ("base_height",        1),
    ("obstacle_obs",       8),
    ("terrain_heights",    5),
    ("is_physical",        3),
]
assert sum(s for _, s in OBS_GROUPS_PARK) == 79, "OBS_GROUPS_PARK frame size != 79"


class Go2ParkBridgeEnv(BridgeControlMixin, Go2ParkEnv):
    """v48 park env configured for bridge streaming and gamepad command.

    Inherits from Go2ParkEnv to share the EXACT obs pipeline (79/frame × 5
    = 395).  BridgeControlMixin adds set_stand_gains / set_walk_gains / IK
    stand / get_obs_breakdown.

    Key bridge deltas vs training env:
      - num_envs=1, flat scene (Go2ParkEnv._build_park_heightfield produces a
        flat map because seed produces near-zero noise on flat bands; Z2 patch
        only adds rough in Z2 extent which the robot never reaches at drive time).
      - skill_command.external=True: X/B/Y write command directly; the
        auto-source step() is a no-op.
      - max_episode_length_s=3600: effectively infinite — operator drives until
        they choose to stop; the normal curriculum timeouts are irrelevant.
      - phase="G0" (ghost phase): no entity is raised by default → clean flat
        arena.  set_debug_course() can raise 2-3 ghost obstacles for HUD testing.
      - Velocity re-pinned every reset (quirk Q2).
      - IK stand mode (L2 released) via BridgeControlMixin.
    """

    def __init__(self, num_envs: int = 1, dt: float = 1 / 50,
                 headless: bool = True, camera_res: tuple = (1280, 720),
                 max_episode_length_s: float = 3600.0):
        super().__init__(
            num_envs=num_envs,
            dt=dt,
            max_episode_length_s=max_episode_length_s,
            headless=headless,
            phase="G0",      # ghost: no physical obstacles at boot
            zone_key="FULL",
            seed=42,
        )
        # Bridge obs group descriptor (for get_obs_breakdown).
        self.obs_groups = OBS_GROUPS_PARK

        # Add streaming camera (replacing training-multi-cam).
        self.camera_bridge = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0), lookat=(0.0, 0.0, 0.2),
            res=camera_res, fov=40, env_idx=0)

        # Gamepad state.
        self._gamepad_vx = 0.0
        self._gamepad_vy = 0.0
        self._gamepad_wz = 0.0
        self._gait_idx = 1           # boot gait = trot (index 1)
        self._jump_steps_left = 0
        self._climb_steps_left = 0
        self._crouch_on = False

        # Sustained down-detection (loosened live reset — same as other bridges).
        self._down_run = None        # allocated in config()

    # Override the camera property so the runner's follow_entity / render calls
    # use the streaming camera, not the training multi-cam cameras.
    @property
    def camera(self):
        return self.camera_bridge

    @camera.setter
    def camera(self, value):
        # Go2ParkEnv __init__ assigns self.camera before camera_bridge exists;
        # store it as _training_camera and let camera_bridge take priority.
        self._training_camera = value

    def config(self):
        super().config()

        # Mark skill commands as operator-driven: auto-sources go dead (Q3).
        self.skill_command.external = True

        # Down-run counter for sustained fall detection.
        self._down_run = torch.zeros(self.num_envs, dtype=torch.long,
                                     device=gs.device)

        # Override fall_over with grounded-only sustained detection so airborne
        # maneuvers never reset mid-jump (same pattern as launch/hurdle bridges).
        from genesis_forge.managers.config.config_item import TerminationConfigItem
        self.termination_manager.term_cfg["fall_over"] = TerminationConfigItem(
            {"fn": self._t_down_sustained}, self)

    def _t_down_sustained(self, env):
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        down = (gz > _DOWN_GZ_THRESHOLD) & ~self._airborne
        self._down_run = torch.where(
            down, self._down_run + 1,
            torch.zeros_like(self._down_run))
        return self._down_run >= _DOWN_GRACE_STEPS

    # ── build / reset ─────────────────────────────────────────────────────────

    def build(self):
        super().build()
        # Follow entity with streaming camera.
        self.camera_bridge.follow_entity(self.robot)

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)

        # Reset down-run counter.
        if self._down_run is not None:
            if envs_idx is None:
                self._down_run.zero_()
            else:
                self._down_run[envs_idx] = 0

        # Re-pin gamepad command after reset (quirk Q2 — velocity_command
        # resamples randomly on reset; the bridge must overwrite it).
        self._apply_gamepad_command()
        return result

    # ── bridge command interface ──────────────────────────────────────────────

    def set_velocity_from_gamepad(self, cmd_data: dict):
        """Map gamepad cmd_data to velocity_command and cache for re-application.

        cmd_data keys (from genesis_sim_runner set_cmd_vel dispatch):
          linear_y  -> vx* (left stick Y, positive = forward)
          linear_x  -> vy* (left stick X; lateral; clamped to vy range)
          angular_z -> wz* (right stick X; yaw rate)
        """
        ly = float(cmd_data.get("linear_y", 0.0))
        lx = float(cmd_data.get("linear_x", 0.0))
        az = float(cmd_data.get("angular_z", 0.0))

        self._gamepad_vx = max(VX_MIN, min(VX_MAX, ly * VX_MAX))
        self._gamepad_vy = max(VY_MIN, min(VY_MAX, lx * abs(VY_MAX)))
        self._gamepad_wz = max(WZ_MIN, min(WZ_MAX, az))
        self._apply_gamepad_command()

    def _apply_gamepad_command(self):
        """Write cached gamepad state into velocity_command (pinned — no resample)."""
        cmd = getattr(self, "velocity_command", None)
        if cmd is None or cmd.command is None:
            return
        cmd.command[:, 0] = self._gamepad_vx
        cmd.command[:, 1] = self._gamepad_vy
        cmd.command[:, 2] = self._gamepad_wz

    def set_jump_intent(self, intensity: float = 1.0):
        """X button: pulse jump for JUMP_PULSE_STEPS steps (0.8 s)."""
        if intensity > 0.1:
            self._jump_steps_left = JUMP_PULSE_STEPS
            self.skill_command.command[:, 0] = 1.0

    def set_crouch_intent(self, on: bool):
        """B button (held): crouch command on/off."""
        self._crouch_on = on
        self.skill_command.command[:, 1] = 1.0 if on else 0.0

    def set_climb_intent(self, intensity: float = 1.0):
        """Y button: pulse climb for CLIMB_PULSE_STEPS steps (1.5 s)."""
        if intensity > 0.1:
            self._climb_steps_left = CLIMB_PULSE_STEPS
            self.skill_command.command[:, 2] = 1.0

    def set_gait_by_name(self, gait_name: str, period: float | None = None,
                         clearance: float | None = None):
        """D-pad cycling: sets gait via GaitCommandManager.set_fixed_gait.

        The runner's step_sim re-applies this every step so the resample timer
        cannot overwrite it (Gate A-0 quirk Q1).
        """
        if gait_name not in _GAIT_CYCLE:
            return
        period = period or _GAIT_DEFAULT_PERIODS.get(gait_name, 0.45)
        clearance = clearance or _GAIT_DEFAULT_CLEARANCE
        self._gait_idx = _GAIT_CYCLE.index(gait_name)
        gcm = getattr(self, "gait_command", None)
        if gcm is not None:
            gcm.set_fixed_gait(gait_name, period, clearance)

    def zero_velocity(self):
        """Called by runner on HOLD/ESTOP — zero velocity commands."""
        self._gamepad_vx = 0.0
        self._gamepad_vy = 0.0
        self._gamepad_wz = 0.0
        self._apply_gamepad_command()

    def get_velocity_command(self) -> dict:
        """Telemetry: current velocity command for the HUD."""
        cmd = getattr(self, "velocity_command", None)
        if cmd is None or cmd.command is None:
            return {}
        return {
            "mode": "park",
            "vx": float(cmd.command[0, 0]),
            "vy": float(cmd.command[0, 1]),
            "wz": float(cmd.command[0, 2]),
            "gait": _GAIT_CYCLE[self._gait_idx],
            "skill_cmd": self.skill_command.command[0].tolist()
                         if self.skill_command.command is not None else [0, 0, 0],
        }

    # ── step: manage pulse countdowns ────────────────────────────────────────

    def step(self, actions):
        # Tick jump pulse countdown.
        if self._jump_steps_left > 0:
            self._jump_steps_left -= 1
            self.skill_command.command[:, 0] = 1.0
            if self._jump_steps_left == 0:
                self.skill_command.command[:, 0] = 0.0

        # Tick climb pulse countdown.
        if self._climb_steps_left > 0:
            self._climb_steps_left -= 1
            self.skill_command.command[:, 2] = 1.0
            if self._climb_steps_left == 0:
                self.skill_command.command[:, 2] = 0.0

        # Crouch: held via set_crouch_intent (no countdown needed).

        return super().step(actions)

    # ── debug course (HUD testing) ────────────────────────────────────────────

    def set_debug_course(self, enable: bool = True):
        """Raise/sink a small 2-slot ghost obstacle strip for HUD testing.

        When enable=True: 2 bar-type slots are filled into _slot_* tensors with
        hardcoded positions (3 m and 7 m ahead of spawn) at ghost z.  The
        obstacle_obs block will show them without any physical entity collision.
        When enable=False: clears the slot tensors back to empty.

        This is HUD-only — no collision, no reward.  Useful for testing the
        obstacle_obs projection and zone strip rendering before training completes.
        """
        dev = gs.device
        if enable:
            # Two ghost bars: 3 m and 7 m ahead of spawn at x=0.
            self._slot_active[0] = False
            self._slot_kind[0] = -1   # KIND_NONE
            for j, slot_x in enumerate([3.0, 7.0]):
                self._slot_x[0, j] = slot_x
                self._slot_low[0, j] = 0.0
                self._slot_high[0, j] = 0.15    # 15 cm bar
                self._slot_depth[0, j] = 0.05
                self._slot_kind[0, j] = 0        # KIND_BAR
                self._slot_active[0, j] = True
        else:
            self._slot_active[0] = False
            self._slot_kind[0] = -1

    # ── obs breakdown (bridge_control compatibility) ──────────────────────────
    # BridgeControlMixin.get_obs_breakdown uses self.obs_groups + extras["policy"]
    # The parent Go2ParkEnv stores obs under "policy" via ObservationManager.
