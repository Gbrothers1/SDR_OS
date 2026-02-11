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

REAR_STAND_DEFAULT_POS = {
    ".*_hip_joint": 0.0,
    "FL_thigh_joint": 1.6,
    "FR_thigh_joint": 1.6,
    "RL_thigh_joint": 0.6,
    "RR_thigh_joint": 0.6,
    "FL_calf_joint": -2.2,
    "FR_calf_joint": -2.2,
    "RL_calf_joint": -1.3,
    "RR_calf_joint": -1.3,
}

JUMP_CROUCH_DEFAULT_POS = {
    ".*_hip_joint": 0.0,
    "FL_thigh_joint": 1.1,
    "FR_thigh_joint": 1.1,
    "RL_thigh_joint": 1.3,
    "RR_thigh_joint": 1.3,
    ".*_calf_joint": -2.0,
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
        self._jump_progress = torch.zeros(self.num_envs, device=gs.device)

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
            cfg=self._build_reward_cfg(),
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

    def reset(self, envs_idx: list[int] | torch.Tensor | None = None):
        reset = super().reset(envs_idx)
        if self.skill == "jump":
            if envs_idx is None:
                self._jump_progress.zero_()
            else:
                if isinstance(envs_idx, torch.Tensor):
                    env_ids = envs_idx.to(device=gs.device, dtype=torch.long)
                else:
                    env_ids = torch.tensor(envs_idx, device=gs.device, dtype=torch.long)
                self._jump_progress[env_ids] = 0.0
        return reset

    def step(self, actions: torch.Tensor):
        obs, rewards, terminated, truncated, extras = super().step(actions)
        if self.skill == "jump":
            vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
            progress = self._jump_progress + torch.clamp(vel[:, 0], min=0.0) * self.dt
            self._jump_progress = torch.clamp(progress, min=0.0)
        return obs, rewards, terminated, truncated, extras

    def _linear_velocity_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
        return torch.norm(vel[:, :2], dim=-1)

    def _angular_velocity_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        ang = self._flatten_vec(self.robot_manager.get_angular_velocity())
        return torch.norm(ang, dim=-1)

    def _dof_velocity_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        dof_vel = self.action_manager.get_dofs_velocity()
        return torch.norm(dof_vel, dim=-1)

    def _lateral_velocity_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
        return torch.abs(vel[:, 1])

    def _forward_velocity_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
        return torch.clamp(vel[:, 0], min=0.0)

    def _get_foot_contact_forces(self) -> torch.Tensor:
        if not self.gait_command_manager.foot_links:
            return torch.zeros(self.num_envs, 4, device=gs.device)
        forces = []
        for link in self.gait_command_manager.foot_links:
            force = torch.norm(self.foot_contact_manager.get_contact_forces(link.idx), dim=-1)
            forces.append(force)
        return torch.stack(forces, dim=1).squeeze(-1)

    def _flatten_vec(self, vec: torch.Tensor) -> torch.Tensor:
        if vec.ndim > 2:
            return vec.view(vec.shape[0], -1)
        return vec

    def _flight_bonus(self, env: ManagedEnvironment, threshold: float = 1.0) -> torch.Tensor:
        contact = self._get_foot_contact_forces()
        all_air = (contact < threshold).all(dim=1)
        return all_air.float()

    def _front_air_bonus(self, env: ManagedEnvironment, threshold: float = 1.0) -> torch.Tensor:
        contact = self._get_foot_contact_forces()
        front_air = (contact[:, :2] < threshold).all(dim=1)
        return front_air.float()

    def _rear_contact_bonus(self, env: ManagedEnvironment, threshold: float = 1.0) -> torch.Tensor:
        contact = self._get_foot_contact_forces()
        rear_contact = (contact[:, 2:] > threshold).float().mean(dim=1)
        return rear_contact

    def _avg_foot_height_bonus(self, env: ManagedEnvironment, min_height: float = 0.08) -> torch.Tensor:
        if not self.gait_command_manager.foot_links:
            return torch.zeros(self.num_envs, device=gs.device)
        link_idx = [f.idx_local for f in self.gait_command_manager.foot_links]
        foot_pos = self.robot.get_links_pos(links_idx_local=link_idx)
        avg_height = foot_pos[:, :, 2].mean(dim=1)
        return torch.clamp(avg_height - min_height, min=0.0)

    def _jump_distance_progress(self, env: ManagedEnvironment) -> torch.Tensor:
        target = self._skill_cfg.get("target_distance", 1.0)
        vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
        progress = self._jump_progress + torch.clamp(vel[:, 0], min=0.0) * self.dt
        return torch.clamp(progress / target, max=1.0)

    def _stillness_bonus(
        self,
        env: ManagedEnvironment,
        lin_thresh: float = 0.05,
        ang_thresh: float = 0.15,
        dof_thresh: float = 0.5,
    ) -> torch.Tensor:
        lin_vel = torch.norm(self._flatten_vec(self.robot_manager.get_linear_velocity())[:, :2], dim=-1)
        ang_vel = torch.norm(self._flatten_vec(self.robot_manager.get_angular_velocity()), dim=-1)
        dof_vel = torch.norm(self.action_manager.get_dofs_velocity(), dim=-1)
        still = (lin_vel < lin_thresh) & (ang_vel < ang_thresh) & (dof_vel < dof_thresh)
        return still.float()

    def _build_reward_cfg(self) -> dict:
        if self.skill == "jump":
            return {
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
                "forward_vel": {
                    "weight": self._skill_cfg["forward_vel_weight"],
                    "fn": self._forward_velocity_reward,
                },
                "distance_progress": {
                    "weight": self._skill_cfg["distance_progress_weight"],
                    "fn": self._jump_distance_progress,
                },
                "lat_vel": {
                    "weight": self._skill_cfg["lat_vel_weight"],
                    "fn": self._lateral_velocity_penalty,
                },
                "ang_vel": {
                    "weight": self._skill_cfg["ang_vel_weight"],
                    "fn": self._angular_velocity_penalty,
                },
                "flight_bonus": {
                    "weight": self._skill_cfg["flight_bonus_weight"],
                    "fn": self._flight_bonus,
                },
                "foot_height_bonus": {
                    "weight": self._skill_cfg["foot_height_bonus_weight"],
                    "fn": self._avg_foot_height_bonus,
                },
                "action_rate": {
                    "weight": self._skill_cfg["action_rate_weight"],
                    "fn": rewards.action_rate_l2,
                },
                "dof_velocity": {
                    "weight": self._skill_cfg["dof_vel_weight"],
                    "fn": self._dof_velocity_penalty,
                },
                "bad_contact": {
                    "weight": -1.0,
                    "fn": rewards.contact_force,
                    "params": {
                        "contact_manager": self.bad_contact_manager,
                    },
                },
            }

        if self.skill == "rear_stand":
            return {
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
                "front_air_bonus": {
                    "weight": self._skill_cfg["front_air_bonus_weight"],
                    "fn": self._front_air_bonus,
                },
                "rear_contact_bonus": {
                    "weight": self._skill_cfg["rear_contact_bonus_weight"],
                    "fn": self._rear_contact_bonus,
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
            }

        return {
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
        }

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
        if skill == "rear_stand":
            return {
                "default_pos": REAR_STAND_DEFAULT_POS,
                "target_height": 0.4,
                "base_height_weight": -25.0,
                "pose_weight": -2.5,
                "lin_vel_weight": -2.0,
                "ang_vel_weight": -0.8,
                "action_rate_weight": -0.04,
                "dof_vel_weight": -0.08,
                "stillness_bonus_weight": 2.0,
                "front_air_bonus_weight": 2.0,
                "rear_contact_bonus_weight": 1.0,
            }
        if skill == "jump":
            return {
                "default_pos": JUMP_CROUCH_DEFAULT_POS,
                "target_height": 0.9144,
                "target_distance": 0.9144,
                "base_height_weight": -8.0,
                "pose_weight": -0.8,
                "ang_vel_weight": -0.5,
                "action_rate_weight": -0.02,
                "dof_vel_weight": -0.05,
                "forward_vel_weight": 1.5,
                "distance_progress_weight": 2.0,
                "lat_vel_weight": -1.0,
                "flight_bonus_weight": 2.0,
                "foot_height_bonus_weight": 0.5,
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


class Go2RearStandEnv(Go2SkillEnv):
    def __init__(self, *args, **kwargs):
        super().__init__("rear_stand", *args, **kwargs)


class Go2JumpEnv(Go2SkillEnv):
    def __init__(self, *args, **kwargs):
        super().__init__("jump", *args, **kwargs)
