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


INITIAL_BODY_POSITION = [0.0, 0.0, 0.4]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]

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
                max_collision_pairs=30,
            ),
        )

        self.terrain = self.scene.add_entity(gs.morphs.Plane())

        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file="urdf/go2/urdf/go2.urdf",
                pos=INITIAL_BODY_POSITION,
                quat=INITIAL_QUAT,
            ),
        )

        self.camera = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0),
            lookat=(0.0, 0.0, 0.0),
            res=camera_res,
            fov=40,
            env_idx=0,
        )

        # Will be set in config()
        self.velocity_command = None
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
                ".*_calf_joint": -1.5,
            },
            kp=20,
            kv=0.5,
        )

        # Position action manager
        self.action_manager = PositionActionManager(
            self,
            scale=0.25,
            clip=(-100.0, 100.0),
            use_default_offset=True,
            actuator_manager=self.actuator_manager,
        )

        # Velocity command manager for locomotion
        self.velocity_command = VelocityCommandManager(
            self,
            range={
                "lin_vel_x": (-1.0, 1.0),
                "lin_vel_y": (-0.5, 0.5),
                "ang_vel_z": (-1.0, 1.0),
            },
            resample_time_sec=5.0,
            standing_probability=0.1,
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

        # Termination
        TerminationManager(
            self,
            logging_enabled=True,
            term_cfg={
                "timeout": {
                    "fn": terminations.timeout,
                    "time_out": True,
                },
                "fall_over": {
                    "fn": terminations.bad_orientation,
                    "params": {
                        "limit_angle": 10.0,
                        "entity_manager": self.robot_manager,
                    },
                },
            },
        )

        # Policy observations (62 dim per frame, history_len=5 → 310 dim)
        # Must match training env (Go2GaitTrainingEnv) structure exactly
        ObservationManager(
            self,
            name="policy",
            history_len=5,
            cfg={
                "gait_command": {
                    "fn": lambda env: torch.zeros(self.num_envs, 14, device=gs.device),
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

        # Privileged critic observations (16 dim per frame, history_len=5 → 80 dim)
        # Combined with policy via obs_groups → critic total = (62+16)*5 = 390
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

    def set_velocity_from_gamepad(self, joystick_state: dict):
        """
        Set velocity command from web UI gamepad state.

        Expected joystick_state format from SDR_OS:
        {
            "leftStickX": float (-1 to 1),
            "leftStickY": float (-1 to 1),
            "rightStickX": float (-1 to 1),
            "rightStickY": float (-1 to 1),
        }

        Mapping:
        - Left stick Y (inverted) -> lin_vel_x (forward/back)
        - Left stick X -> lin_vel_y (strafe)
        - Right stick X -> ang_vel_z (turn)
        """
        if self.velocity_command is None:
            return

        lx = joystick_state.get("leftStickX", 0.0)
        ly = joystick_state.get("leftStickY", 0.0)
        rx = joystick_state.get("rightStickX", 0.0)

        # Map stick values (-1..1) to velocity ranges
        ranges = self.velocity_command.range
        lin_x = self._map_stick(ly, *ranges["lin_vel_x"])  # Y-axis inverted
        lin_y = self._map_stick(lx, *ranges["lin_vel_y"])
        ang_z = self._map_stick(rx, *ranges["ang_vel_z"])

        self.velocity_command.set_command("lin_vel_x", torch.tensor(lin_x, device=gs.device))
        self.velocity_command.set_command("lin_vel_y", torch.tensor(lin_y, device=gs.device))
        self.velocity_command.set_command("ang_vel_z", torch.tensor(ang_z, device=gs.device))

    @staticmethod
    def _map_stick(value: float, range_min: float, range_max: float) -> float:
        """Map stick value (-1..1) to a velocity range."""
        return (value + 1.0) * (range_max - range_min) / 2.0 + range_min
