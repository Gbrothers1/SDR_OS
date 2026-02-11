"""
Go2 locomotion environment for the SDR_OS bridge server.

Uses genesis-forge ManagedEnvironment with:
- PositionActionManager for joint PD control
- VelocityCommandManager for gamepad-driven locomotion commands
- ObservationManager for structured obs (48 dim)
- RewardManager with locomotion reward terms
- TerminationManager (timeout + fall detection)
- Camera for JPEG frame streaming
"""

import torch
import genesis as gs

from genesis_forge import ManagedEnvironment
from genesis_forge.managers import (
    RewardManager,
    TerminationManager,
    EntityManager,
    ObservationManager,
    ActuatorManager,
    PositionActionManager,
)
from genesis_forge.managers.command import VelocityCommandManager
from genesis_forge.mdp import reset, rewards, terminations
from rl.envs.gait_command_manager import GaitCommandManager


INITIAL_BODY_POSITION = [0.0, 0.0, 0.4]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]

# PD gains — walk must match policy training; stand is higher for passive stability
WALK_KP = 20.0
WALK_KV = 0.5
STAND_KP = 50.0
STAND_KV = 2.0

# Observation group names — used by bridge to decompose obs tensor
# Matches training env order: gait_command, velocity_cmd, then proprioception
# Note: with history_len=5 the full obs is 62*5=310 dim; this is per-frame layout
OBS_GROUPS = [
    ("gait_command", 14),
    ("velocity_command", 3),
    ("angular_velocity", 3),
    ("linear_velocity", 3),
    ("projected_gravity", 3),
    ("dof_position", 12),
    ("dof_velocity", 12),
    ("actions", 12),
]


class Go2BridgeEnv(ManagedEnvironment):
    """
    Go2 locomotion environment configured for bridge streaming.

    Adds VelocityCommandManager for gamepad-driven velocity targets,
    and exposes camera + observation group metadata for the web UI.
    """

    def __init__(
        self,
        num_envs: int = 1,
        dt: float = 1 / 50,
        max_episode_length_s: float = 20.0,
        headless: bool = True,
        camera_res: tuple = (1280, 720),
    ):
        super().__init__(
            num_envs=num_envs,
            dt=dt,
            max_episode_length_sec=max_episode_length_s,
            max_episode_random_scaling=0.1,
        )

        self.scene = gs.Scene(
            show_viewer=not headless,
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            viewer_options=gs.options.ViewerOptions(
                max_FPS=int(0.5 / self.dt),
                camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
            vis_options=gs.options.VisOptions(
                rendered_envs_idx=list(range(min(num_envs, 1)))
            ),
            rigid_options=gs.options.RigidOptions(
                dt=self.dt,
                constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True,
                enable_joint_limit=True,
                max_collision_pairs=60,
            ),
        )

        self.terrain = self.scene.add_entity(gs.morphs.Plane())

        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file="urdf/go2/urdf/go2.urdf",
                pos=INITIAL_BODY_POSITION,
                quat=INITIAL_QUAT,
                links_to_keep=["FL_foot", "FR_foot", "RL_foot", "RR_foot"],
            ),
        )

        self.camera = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0),
            lookat=(0.0, 0.0, 0.0),
            res=camera_res,
            fov=40,
            env_idx=0,
        )

        # External velocity command buffer — written by gamepad, read by VelocityCommandManager
        self._cmd_buf = torch.zeros((num_envs, 3), device=gs.device)

        # Will be set in config()
        self.velocity_command = None
        self.gait_command_manager = None
        self.robot_manager = None
        self.actuator_manager = None
        self.action_manager = None

    def config(self):
        # Robot reset manager
        self.robot_manager = EntityManager(
            self,
            entity_attr="robot",
            on_reset={
                "position": {
                    "fn": reset.position,
                    "params": {
                        "position": INITIAL_BODY_POSITION,
                        "quat": INITIAL_QUAT,
                        "zero_velocity": True,
                    },
                },
            },
        )

        # Joint actuators
        self.actuator_manager = ActuatorManager(
            self,
            joint_names=[
                "FL_.*_joint",
                "FR_.*_joint",
                "RL_.*_joint",
                "RR_.*_joint",
            ],
            default_pos={
                ".*_hip_joint": 0.0,
                "FL_thigh_joint": 0.8,
                "FR_thigh_joint": 0.8,
                "RL_thigh_joint": 1.0,
                "RR_thigh_joint": 1.0,
                ".*_calf_joint": -1.6,
            },
            kp=20,
            kv=0.5,
        )

        # Position action manager — clip defaults to URDF joint limits (matches training)
        self.action_manager = PositionActionManager(
            self,
            scale=0.25,
            use_default_offset=True,
            actuator_manager=self.actuator_manager,
        )

        # Velocity command manager for locomotion — ranges match training env
        self.velocity_command = VelocityCommandManager(
            self,
            range={
                "lin_vel_x": (-3.5, 3.5),
                "lin_vel_y": (-0.5, 0.5),
                "ang_vel_z": (-5.0, 5.0),
            },
            standing_probability=0.0,
            resample_time_sec=1.0,
        )
        self.velocity_command.enabled = True
        self.velocity_command.use_external_controller(lambda step: self._cmd_buf)

        # Gait command manager — provides 14-dim gait timing observations to the policy
        self.gait_command_manager = GaitCommandManager(
            self,
            foot_names={
                "FL": "FL_foot",
                "FR": "FR_foot",
                "RL": "RL_foot",
                "RR": "RR_foot",
            },
            resample_time_sec=4.0,
        )

        # Rewards
        RewardManager(
            self,
            logging_enabled=True,
            cfg={
                "base_height_target": {
                    "weight": -50.0,
                    "fn": rewards.base_height,
                    "params": {
                        "target_height": 0.3,
                        "entity_attr": "robot",
                    },
                },
                "tracking_lin_vel": {
                    "weight": 1.0,
                    "fn": rewards.command_tracking_lin_vel,
                    "params": {
                        "command": self.velocity_command.command[:, :2],
                        "entity_manager": self.robot_manager,
                    },
                },
                "tracking_ang_vel": {
                    "weight": 0.2,
                    "fn": rewards.command_tracking_ang_vel,
                    "params": {
                        "commanded_ang_vel": self.velocity_command.command[:, 2],
                        "entity_manager": self.robot_manager,
                    },
                },
                "lin_vel_z": {
                    "weight": -1.0,
                    "fn": rewards.lin_vel_z_l2,
                    "params": {
                        "entity_manager": self.robot_manager,
                    },
                },
                "action_rate": {
                    "weight": -0.005,
                    "fn": rewards.action_rate_l2,
                },
                "similar_to_default": {
                    "weight": -0.1,
                    "fn": rewards.dof_similar_to_default,
                    "params": {
                        "action_manager": self.action_manager,
                    },
                },
            },
        )

        # Termination — relaxed for live bridge control (not training)
        TerminationManager(
            self,
            logging_enabled=True,
            term_cfg={
                "fall_over": {
                    "fn": terminations.bad_orientation,
                    "params": {
                        "limit_angle": 60.0,
                        "entity_manager": self.robot_manager,
                    },
                },
            },
        )

        # Policy observations (62 dim per frame, history_len=5 -> 310 dim)
        # Must match training env (Go2GaitTrainingEnv) structure exactly
        ObservationManager(
            self,
            name="policy",
            history_len=5,
            cfg={
                "gait_command": {
                    "fn": self.gait_command_manager.observation,
                },
                "velocity_cmd": {
                    "fn": self.velocity_command.observation,
                },
                "angle_velocity": {
                    "fn": lambda env: self.robot_manager.get_angular_velocity(),
                },
                "linear_velocity": {
                    "fn": lambda env: self.robot_manager.get_linear_velocity(),
                },
                "projected_gravity": {
                    "fn": lambda env: self.robot_manager.get_projected_gravity(),
                },
                "dof_position": {
                    "fn": lambda env: self.action_manager.get_dofs_position(),
                },
                "dof_velocity": {
                    "fn": lambda env: self.action_manager.get_dofs_velocity(),
                    "scale": 0.05,
                },
                "actions": {
                    "fn": lambda env: self.action_manager.get_actions(),
                },
            },
        )

        # Privileged critic observations (16 dim per frame, history_len=5 -> 80 dim)
        # Combined with policy via obs_groups -> critic total = (62+16)*5 = 390
        ObservationManager(
            self,
            name="critic",
            history_len=5,
            cfg={
                "foot_contact_force": {
                    "fn": lambda env: torch.zeros(self.num_envs, 4, device=gs.device),
                },
                "dof_force": {
                    "fn": lambda env: torch.zeros(self.num_envs, 12, device=gs.device),
                    "scale": 0.1,
                },
            },
        )

    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)
        # Set default trot gait so the policy receives valid gait timing observations
        self.gait_command_manager.set_fixed_gait("trot", period=0.45, clearance=0.08)

    def get_obs_breakdown(self) -> dict:
        """
        Decompose the flat observation tensor into named groups.
        Returns dict of {group_name: list_of_floats} for env 0.
        Uses only the most recent frame (last 62 dims of the 310-dim history).
        Used by bridge_server to emit structured obs to the UI.
        """
        obs = self.get_observations()
        if obs is None:
            return {}

        # With history_len=5, obs is 310 dim (5 * 62). Take last frame.
        frame_size = sum(size for _, size in OBS_GROUPS)
        latest_frame = obs[0, -frame_size:]

        breakdown = {}
        offset = 0
        for name, size in OBS_GROUPS:
            values = latest_frame[offset:offset + size]
            breakdown[name] = values.cpu().tolist()
            offset += size
        return breakdown

    def get_reward_breakdown(self) -> dict:
        """
        Get per-term reward values from the RewardManager.
        Returns dict of {term_name: float} for the latest step.
        """
        rm = self.managers.get("reward")
        if rm is None:
            return {}
        # RewardManager with logging_enabled stores per-term values in extras
        terms = {}
        if hasattr(rm, '_last_reward_terms'):
            for name, val in rm._last_reward_terms.items():
                terms[name] = float(val[0]) if hasattr(val, '__getitem__') else float(val)
        return terms

    def get_velocity_command(self) -> dict:
        """Get current velocity command values for env 0."""
        if self.velocity_command is None:
            return {}
        cmd = self.velocity_command.command
        return {
            "lin_vel_x": float(cmd[0, 0]),
            "lin_vel_y": float(cmd[0, 1]),
            "ang_vel_z": float(cmd[0, 2]),
        }

    def set_velocity_from_gamepad(self, cmd_data: dict):
        """
        Set velocity command from NATS set_cmd_vel command data.

        Expected cmd_data format (from GenesisContext.jsx sendVelocityCommand):
        {
            "linear_x": float (-1 to 1),   # left stick X axis
            "linear_y": float (-1 to 1),   # left stick Y axis
            "angular_z": float (-1 to 1),  # shoulder buttons / right stick
        }

        Axis mapping:
        - linear_y  -> lin_vel_x (forward/back)
        - linear_x  -> lin_vel_y (strafe)
        - angular_z -> ang_vel_z (turn)

        Writes directly to _cmd_buf which the VelocityCommandManager reads
        via use_external_controller, bypassing random resampling.
        """
        lx = cmd_data.get("linear_x", 0.0)
        ly = cmd_data.get("linear_y", 0.0)
        rz = cmd_data.get("angular_z", 0.0)

        # Map stick values (-1..1) to velocity ranges
        ranges = self.velocity_command.range
        self._cmd_buf[0, 0] = self._map_stick(ly, *ranges["lin_vel_x"])
        self._cmd_buf[0, 1] = self._map_stick(lx, *ranges["lin_vel_y"])
        self._cmd_buf[0, 2] = self._map_stick(rz, *ranges["ang_vel_z"])

    def zero_velocity(self):
        """Zero the velocity command buffer (for HOLD/ESTOP)."""
        self._cmd_buf.zero_()

    def set_stand_gains(self):
        """Switch to high-stiffness PD gains and widen action scale for standing.

        Training uses scale=0.25 (caps joint offsets to ±0.25 rad). Standing
        body-pose control needs far more range, so we set scale=1.0 during
        stand mode — actions map directly to radian offsets from default.
        """
        n = len(self.actuator_manager.dofs_idx)
        kp = torch.full((n,), STAND_KP, device=gs.device)
        kv = torch.full((n,), STAND_KV, device=gs.device)
        self.robot.set_dofs_kp(kp, self.actuator_manager.dofs_idx)
        self.robot.set_dofs_kv(kv, self.actuator_manager.dofs_idx)
        # Widen action scale: actions are now radian offsets from default
        self.action_manager._scale_values.fill_(1.0)

    def set_walk_gains(self):
        """Restore training PD gains and action scale for policy locomotion."""
        n = len(self.actuator_manager.dofs_idx)
        kp = torch.full((n,), WALK_KP, device=gs.device)
        kv = torch.full((n,), WALK_KV, device=gs.device)
        self.robot.set_dofs_kp(kp, self.actuator_manager.dofs_idx)
        self.robot.set_dofs_kv(kv, self.actuator_manager.dofs_idx)
        # Restore training scale
        self.action_manager._scale_values.fill_(0.25)

    def compute_stand_actions(self, pitch, roll, yaw, height):
        """IK-style body pose from joystick axes. All inputs [-1, 1].

        Joint index layout (12-dim, from ActuatorManager regex ordering):
          [0] FL_hip  [1] FL_thigh  [2] FL_calf
          [3] FR_hip  [4] FR_thigh  [5] FR_calf
          [6] RL_hip  [7] RL_thigh  [8] RL_calf
          [9] RR_hip  [10] RR_thigh [11] RR_calf

        In stand mode, set_stand_gains() sets action scale=1.0, so actions
        map directly to radian offsets from default joint positions:
          target = default_pos + action * 1.0
        Clamped to URDF limits by PositionActionManager.

        URDF joint ranges (for reference):
          hip:           [-1.047, 1.047]   default 0.0
          FL/FR thigh:   [-1.571, 3.491]   default 0.8
          RL/RR thigh:   [-0.524, 4.538]   default 1.0
          calf:          [-2.723, -0.838]   default -1.6
        """
        actions = torch.zeros(1, 12, device=gs.device)

        # ── Gravity compensation (small radian bias, kp=50 handles most) ──
        GRAV_THIGH = -0.05  # rad — slightly more vertical legs
        GRAV_CALF  =  0.04  # rad — slightly stiffer knee

        for i in (1, 4, 7, 10):   # all thighs
            actions[0, i] += GRAV_THIGH
        for i in (2, 5, 8, 11):   # all calves
            actions[0, i] += GRAV_CALF

        # ── Joystick body pose offsets (radians at full stick) ────
        PITCH_RANGE  = 0.4   # ±0.4 rad (±23°) front/rear thigh differential
        ROLL_RANGE   = 0.3   # ±0.3 rad (±17°) left/right hip differential
        YAW_RANGE    = 0.25  # ±0.25 rad (±14°) diagonal hip twist
        HEIGHT_RANGE = 0.2   # ±0.2 rad (±11°) all thighs uniform

        # Pitch: front thighs vs rear thighs
        actions[0, 1]  += pitch * PITCH_RANGE    # FL_thigh
        actions[0, 4]  += pitch * PITCH_RANGE    # FR_thigh
        actions[0, 7]  -= pitch * PITCH_RANGE    # RL_thigh
        actions[0, 10] -= pitch * PITCH_RANGE    # RR_thigh

        # Roll: left hips vs right hips
        actions[0, 0] -= roll * ROLL_RANGE       # FL_hip
        actions[0, 3] += roll * ROLL_RANGE       # FR_hip
        actions[0, 6] -= roll * ROLL_RANGE       # RL_hip
        actions[0, 9] += roll * ROLL_RANGE       # RR_hip

        # Yaw: diagonal hip twist
        actions[0, 0] += yaw * YAW_RANGE        # FL_hip
        actions[0, 3] -= yaw * YAW_RANGE        # FR_hip
        actions[0, 6] -= yaw * YAW_RANGE        # RL_hip
        actions[0, 9] += yaw * YAW_RANGE        # RR_hip

        # Height: all thighs uniform (stick up = raise body = less thigh flexion)
        actions[0, 1]  += height * HEIGHT_RANGE  # FL_thigh
        actions[0, 4]  += height * HEIGHT_RANGE  # FR_thigh
        actions[0, 7]  += height * HEIGHT_RANGE  # RL_thigh
        actions[0, 10] += height * HEIGHT_RANGE  # RR_thigh

        # No artificial clamp — PositionActionManager clamps to URDF limits
        return actions

    @staticmethod
    def _map_stick(value: float, range_min: float, range_max: float) -> float:
        """Map stick value (-1..1) to a velocity range."""
        return (value + 1.0) * (range_max - range_min) / 2.0 + range_min
