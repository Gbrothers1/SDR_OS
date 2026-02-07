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
    VelocityCommandManager,
    ContactManager,
)
from genesis_forge.mdp import reset, rewards, terminations, observations

from rl.envs.gait_command_manager import GaitCommandManager


HEIGHT_OFFSET = 0.4
INITIAL_BODY_POSITION = [0.0, 0.0, HEIGHT_OFFSET]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]

STAND_DEFAULT_POS = {
    ".*_hip_joint": 0.0,
    "FL_thigh_joint": 0.8,
    "FR_thigh_joint": 0.8,
    "RL_thigh_joint": 1.0,
    "RR_thigh_joint": 1.0,
    ".*_calf_joint": -1.6,
}

SIT_DEFAULT_POS = {
    ".*_hip_joint": 0.0,
    "FL_thigh_joint": 1.3,
    "FR_thigh_joint": 1.3,
    "RL_thigh_joint": 1.3,
    "RR_thigh_joint": 1.3,
    ".*_calf_joint": -2.1,
}


class Go2SkillEnv(ManagedEnvironment):
    """
    Base environment for Go2 skill policies (stand/sit/freeze).
    Uses a fixed gait command and zero velocity commands.
    """

    def __init__(
        self,
        skill: str,
        num_envs: int = 1,
        dt: float = 1 / 50,
        max_episode_length_s: int | None = 10,
        headless: bool = True,
    ):
        super().__init__(
            num_envs=num_envs,
            dt=dt,
            max_episode_length_sec=max_episode_length_s,
            max_episode_random_scaling=0.1,
        )

        self.skill = skill
        self._skill_cfg = self._get_skill_cfg(skill)

        # Construct the scene
        self.scene = gs.Scene(
            show_viewer=not headless,
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            viewer_options=gs.options.ViewerOptions(
                max_FPS=int(0.5 / self.dt),
                camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=40,
            ),
            vis_options=gs.options.VisOptions(rendered_envs_idx=list(range(1))),
            rigid_options=gs.options.RigidOptions(
                dt=self.dt,
                constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True,
                enable_joint_limit=True,
                max_collision_pairs=60,
            ),
        )

        # Create terrain
        self.terrain = self.scene.add_entity(gs.morphs.Plane())

        # Robot
        self.robot = self.scene.add_entity(
            gs.morphs.URDF(
                file="urdf/go2/urdf/go2.urdf",
                pos=INITIAL_BODY_POSITION,
                quat=INITIAL_QUAT,
                links_to_keep=["FL_foot", "FR_foot", "RL_foot", "RR_foot"],
            ),
        )

        # Camera, for headless video recording
        self.camera = self.scene.add_camera(
            pos=(2.5, 1.5, 1.0),
            lookat=(0.0, 0.0, 0.0),
            res=(1280, 720),
            fov=40,
            env_idx=0,
            debug=True,
        )

    def config(self):
        # Robot manager
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

        # Joint Actions
        self.actuator_manager = ActuatorManager(
            self,
            joint_names=[
                "FL_.*_joint",
                "FR_.*_joint",
                "RL_.*_joint",
                "RR_.*_joint",
            ],
            default_pos=self._skill_cfg["default_pos"],
            kp=20,
            kv=0.5,
        )
        self.action_manager = PositionActionManager(
            self,
            scale=0.25,
            use_default_offset=True,
            actuator_manager=self.actuator_manager,
        )

        # Contact manager
        self.foot_contact_manager = ContactManager(
            self,
            link_names=[".*_foot"],
            air_time_contact_threshold=1.0,
        )
        self.body_contact_manager = ContactManager(
            self,
            link_names=["base"],
            air_time_contact_threshold=1.0,
        )
        self.bad_contact_manager = ContactManager(
            self,
            link_names=[".*_thigh", ".*_calf"],
        )

        # Velocity command manager (zero commands)
        self.velocity_command = VelocityCommandManager(
            self,
            range={
                "lin_vel_x": (0.0, 0.0),
                "lin_vel_y": (0.0, 0.0),
                "ang_vel_z": (0.0, 0.0),
            },
            standing_probability=1.0,
            resample_time_sec=5.0,
        )

        # Gait command manager (fixed walk gait)
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
        self.reward_manager = RewardManager(
            self,
            logging_enabled=True,
            cfg={
                "base_height_target": {
                    "weight": self._skill_cfg["base_height_weight"],
                    "fn": rewards.base_height,
                    "params": {
                        "target_height": self._skill_cfg["target_height"],
                        "entity_attr": "robot",
                    },
                },
                "pose_target": {
                    "weight": self._skill_cfg["pose_weight"],
                    "fn": rewards.dof_similar_to_default,
                    "params": {
                        "action_manager": self.action_manager,
                    },
                },
                "lin_vel_xy": {
                    "weight": self._skill_cfg["lin_vel_weight"],
                    "fn": self._linear_velocity_penalty,
                },
                "ang_vel": {
                    "weight": self._skill_cfg["ang_vel_weight"],
                    "fn": self._angular_velocity_penalty,
                },
                "action_rate": {
                    "weight": self._skill_cfg["action_rate_weight"],
                    "fn": rewards.action_rate_l2,
                },
                "dof_velocity": {
                    "weight": self._skill_cfg["dof_vel_weight"],
                    "fn": self._dof_velocity_penalty,
                },
                "stillness_bonus": {
                    "weight": self._skill_cfg["stillness_bonus_weight"],
                    "fn": self._stillness_bonus,
                },
                "bad_contact": {
                    "weight": -1.0,
                    "fn": rewards.contact_force,
                    "params": {
                        "contact_manager": self.bad_contact_manager,
                    },
                },
            },
        )

        # Termination conditions
        self.termination_manager = TerminationManager(
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
                        "limit_angle": 20.0,
                        "entity_manager": self.robot_manager,
                    },
                },
                "body_contact": {
                    "fn": terminations.contact_force,
                    "params": {
                        "contact_manager": self.body_contact_manager,
                        "threshold": 1.0,
                    },
                },
            },
        )

        # Observations
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

        ObservationManager(
            self,
            name="critic",
            history_len=5,
            cfg={
                "foot_contact_force": {
                    "fn": observations.contact_force,
                    "params": {
                        "contact_manager": self.foot_contact_manager,
                    },
                },
                "dof_force": {
                    "fn": observations.entity_dofs_force,
                    "params": {
                        "action_manager": self.action_manager,
                    },
                    "scale": 0.1,
                },
            },
        )

    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)
        self.gait_command_manager.set_fixed_gait("walk", period=0.8, clearance=0.05)

    def _linear_velocity_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        vel = self.robot_manager.get_linear_velocity()
        return torch.norm(vel[:, :2], dim=-1)

    def _angular_velocity_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        ang = self.robot_manager.get_angular_velocity()
        return torch.norm(ang, dim=-1)

    def _dof_velocity_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        dof_vel = self.action_manager.get_dofs_velocity()
        return torch.norm(dof_vel, dim=-1)

    def _stillness_bonus(
        self,
        env: ManagedEnvironment,
        lin_thresh: float = 0.05,
        ang_thresh: float = 0.15,
        dof_thresh: float = 0.5,
    ) -> torch.Tensor:
        lin_vel = torch.norm(self.robot_manager.get_linear_velocity()[:, :2], dim=-1)
        ang_vel = torch.norm(self.robot_manager.get_angular_velocity(), dim=-1)
        dof_vel = torch.norm(self.action_manager.get_dofs_velocity(), dim=-1)
        still = (lin_vel < lin_thresh) & (ang_vel < ang_thresh) & (dof_vel < dof_thresh)
        return still.float()

    def _get_skill_cfg(self, skill: str) -> dict:
        if skill == "sit":
            return {
                "default_pos": SIT_DEFAULT_POS,
                "target_height": 0.22,
                "base_height_weight": -35.0,
                "pose_weight": -3.5,
                "lin_vel_weight": -1.5,
                "ang_vel_weight": -0.5,
                "action_rate_weight": -0.02,
                "dof_vel_weight": -0.08,
                "stillness_bonus_weight": 1.0,
            }
        if skill == "freeze":
            return {
                "default_pos": STAND_DEFAULT_POS,
                "target_height": 0.35,
                "base_height_weight": -25.0,
                "pose_weight": -1.5,
                "lin_vel_weight": -4.0,
                "ang_vel_weight": -1.0,
                "action_rate_weight": -0.08,
                "dof_vel_weight": -0.10,
                "stillness_bonus_weight": 3.0,
            }
        # stand (default)
        return {
            "default_pos": STAND_DEFAULT_POS,
            "target_height": 0.35,
            "base_height_weight": -30.0,
            "pose_weight": -3.0,
            "lin_vel_weight": -1.5,
            "ang_vel_weight": -0.5,
            "action_rate_weight": -0.02,
            "dof_vel_weight": -0.05,
            "stillness_bonus_weight": 1.0,
        }


class Go2StandEnv(Go2SkillEnv):
    def __init__(self, *args, **kwargs):
        super().__init__("stand", *args, **kwargs)


class Go2SitEnv(Go2SkillEnv):
    def __init__(self, *args, **kwargs):
        super().__init__("sit", *args, **kwargs)


class Go2FreezeEnv(Go2SkillEnv):
    def __init__(self, *args, **kwargs):
        super().__init__("freeze", *args, **kwargs)
