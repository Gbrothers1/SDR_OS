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

from rl.envs.gait_command_manager import GaitCommandManager, GAIT_PRESETS


HEIGHT_OFFSET = 0.4
INITIAL_BODY_POSITION = [0.0, 0.0, HEIGHT_OFFSET]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]


class Go2GamepadLocomotionEnv(ManagedEnvironment):
    """
    Go2 locomotion environment with gamepad-style command sampling.

    Commands are sampled to mimic gamepad usage:
    - Speed scales (precision/default/full)
    - Gait presets (walk/trot/run)
    """

    def __init__(
        self,
        num_envs: int = 1,
        dt: float = 1 / 50,
        max_episode_length_s: int | None = 20,
        headless: bool = True,
        max_lin_vel: float = 3.5,
        max_lat_vel: float = 0.5,
        max_ang_vel: float = 5.0,
        command_resample_time: float = 1.5,
        speed_scale_probs: tuple[float, float, float] = (0.3, 0.4, 0.3),
        gait_preset_probs: tuple[float, float, float] = (0.34, 0.33, 0.33),
        standing_probability: float = 0.05,
    ):
        super().__init__(
            num_envs=num_envs,
            dt=dt,
            max_episode_length_sec=max_episode_length_s,
            max_episode_random_scaling=0.4,
        )

        self._max_lin_vel = max_lin_vel
        self._max_lat_vel = max_lat_vel
        self._max_ang_vel = max_ang_vel
        self._command_resample_steps = max(1, int(command_resample_time / dt))
        self._standing_probability = standing_probability

        self._speed_scales = torch.tensor([0.3, 0.6, 1.0], device=gs.device)
        self._speed_scale_probs = torch.tensor(speed_scale_probs, device=gs.device)
        self._speed_scale_probs = self._speed_scale_probs / self._speed_scale_probs.sum()
        self._preset_names = ["walk", "trot", "run"]
        self._preset_probs = torch.tensor(gait_preset_probs, device=gs.device)
        self._preset_probs = self._preset_probs / self._preset_probs.sum()

        self._cmd_buf = torch.zeros((num_envs, 3), device=gs.device)
        self._command_step = 0
        self._next_command_step = 0

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

        # Velocity command manager for locomotion
        self.velocity_command = VelocityCommandManager(
            self,
            range={
                "lin_vel_x": (-self._max_lin_vel, self._max_lin_vel),
                "lin_vel_y": (-self._max_lat_vel, self._max_lat_vel),
                "ang_vel_z": (-self._max_ang_vel, self._max_ang_vel),
            },
            standing_probability=0.0,
            resample_time_sec=1.0,
        )
        self.velocity_command.enabled = True
        self.velocity_command.use_external_controller(lambda step: self._cmd_buf)

        # Gait command manager
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
                "gait_phase_reward": {
                    "weight": 1.5,
                    "fn": self.gait_command_manager.gait_phase_reward,
                    "params": {
                        "contact_manager": self.foot_contact_manager,
                    },
                },
                "foot_height_reward": {
                    "weight": 0.9,
                    "fn": self.gait_command_manager.foot_height_reward,
                },
                "base_height_target": {
                    "weight": -25.0,
                    "fn": rewards.base_height,
                    "params": {
                        "target_height": 0.35,
                        "entity_attr": "robot",
                    },
                },
                "tracking_lin_vel": {
                    "weight": 1.0,
                    "fn": rewards.command_tracking_lin_vel,
                    "params": {
                        "vel_cmd_manager": self.velocity_command,
                        "entity_manager": self.robot_manager,
                    },
                },
                "tracking_ang_vel": {
                    "weight": 0.5,
                    "fn": rewards.command_tracking_ang_vel,
                    "params": {
                        "vel_cmd_manager": self.velocity_command,
                        "entity_manager": self.robot_manager,
                    },
                },
                "body_acceleration": {
                    "weight": -0.1,
                    "fn": rewards.body_acceleration_exp,
                    "params": {
                        "entity_manager": self.robot_manager,
                    },
                },
                "lin_vel_z": {
                    "weight": -0.1,
                    "fn": rewards.lin_vel_z_l2,
                    "params": {
                        "entity_manager": self.robot_manager,
                    },
                },
                "action_rate": {
                    "weight": -0.01,
                    "fn": rewards.action_rate_l2,
                },
                "bad_contact": {
                    "weight": -1.0,
                    "fn": rewards.contact_force,
                    "params": {
                        "contact_manager": self.bad_contact_manager,
                    },
                },
                "foot_slip": {
                    "weight": -0.2,
                    "fn": self.foot_slip_penalty,
                    "params": {
                        "contact_manager": self.foot_contact_manager,
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
        self._resample_gamepad_commands()

    def reset(self, envs_idx: list[int] | None = None):
        reset = super().reset(envs_idx)
        if envs_idx is None:
            self._command_step = 0
            self._next_command_step = 0
        self._resample_gamepad_commands(envs_idx)
        return reset

    def step(self, actions: torch.Tensor):
        if self._command_step >= self._next_command_step:
            self._resample_gamepad_commands()
            self._next_command_step = self._command_step + self._command_resample_steps
        self._command_step += 1
        return super().step(actions)

    def _resample_gamepad_commands(self, envs_idx: list[int] | torch.Tensor | None = None):
        if envs_idx is None:
            env_ids = torch.arange(self.num_envs, device=gs.device)
        elif isinstance(envs_idx, torch.Tensor):
            env_ids = envs_idx.to(device=gs.device, dtype=torch.long)
        else:
            env_ids = torch.tensor(envs_idx, device=gs.device, dtype=torch.long)

        num = len(env_ids)
        if num == 0:
            return

        speed_scale = self._sample_speed_scales(num)
        stick = torch.rand((num, 3), device=gs.device) * 2.0 - 1.0

        # Deadzone
        stick = torch.where(torch.abs(stick) < 0.1, torch.zeros_like(stick), stick)

        lin_x = stick[:, 0] * self._max_lin_vel * speed_scale
        lin_y = stick[:, 1] * self._max_lat_vel * speed_scale
        ang_z = stick[:, 2] * self._max_ang_vel * speed_scale

        # Standing probability
        if self._standing_probability > 0.0:
            stand_mask = torch.rand(num, device=gs.device) < self._standing_probability
            lin_x[stand_mask] = 0.0
            lin_y[stand_mask] = 0.0
            ang_z[stand_mask] = 0.0

        self._cmd_buf[env_ids, 0] = lin_x
        self._cmd_buf[env_ids, 1] = lin_y
        self._cmd_buf[env_ids, 2] = ang_z

        self._sample_gait_presets(env_ids)

    def _sample_speed_scales(self, num: int) -> torch.Tensor:
        idx = torch.multinomial(self._speed_scale_probs, num, replacement=True)
        return self._speed_scales[idx]

    def _sample_gait_presets(self, env_ids: torch.Tensor):
        if not GAIT_PRESETS:
            return
        num = len(env_ids)
        idx = torch.multinomial(self._preset_probs, num, replacement=True)
        for i, env_id in enumerate(env_ids):
            preset = self._preset_names[int(idx[i].item())]
            self.gait_command_manager.apply_preset(preset, env_ids=env_id.view(1))

    def foot_slip_penalty(
        self, env: ManagedEnvironment, contact_manager: ContactManager, threshold: float = 1.0
    ) -> torch.Tensor:
        if not self.gait_command_manager.foot_links:
            return torch.zeros(self.num_envs, device=gs.device)
        link_indices = [f.idx_local for f in self.gait_command_manager.foot_links]
        foot_vel = env.robot.get_links_vel(links_idx_local=link_indices)
        foot_vel_xy = torch.norm(foot_vel[:, :, :2], dim=-1)
        contact_forces = []
        for link in self.gait_command_manager.foot_links:
            forces = torch.norm(contact_manager.get_contact_forces(link.idx), dim=-1)
            contact_forces.append(forces)
        contact = torch.stack(contact_forces, dim=1)
        in_contact = contact > threshold
        slip = foot_vel_xy * in_contact
        return slip.sum(dim=1)
