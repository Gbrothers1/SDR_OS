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
from genesis_forge.managers.command.command_manager import CommandManager
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

# Jump phase constants
JUMP_PHASE_GROUND = 0
JUMP_PHASE_CROUCH = 1
JUMP_PHASE_FLIGHT = 2
JUMP_PHASE_LANDED = 3
JUMP_PHASE_RECOVERY = 4


class Go2SkillEnv(ManagedEnvironment):
    """
    Base environment for Go2 skill policies (stand/sit/freeze).
    Uses a fixed gait command and zero velocity commands.
    """

    def __init__(
        self,
        skill: str,
        profile: str | None = None,
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
        self.profile = profile or skill
        self._skill_cfg = self._get_skill_cfg(self.profile)
        self._jump_progress = torch.zeros(self.num_envs, device=gs.device)
        self._step_id = 0
        self._jump_phase_step_id = -1
        if self.skill == "jump":
            # Phase state machine
            self._jump_phase = torch.zeros(self.num_envs, dtype=torch.long, device=gs.device)
            self._phase_steps = torch.zeros(self.num_envs, dtype=torch.long, device=gs.device)
            self._ground_contact_steps = torch.zeros(self.num_envs, dtype=torch.long, device=gs.device)
            # Jump tracking
            self._jump_cmd_override_count = torch.tensor(0.0, device=gs.device)
            self._jump_cmd_buf = torch.zeros((self.num_envs, 3), device=gs.device)
            self._jump_cmd_hold_counter = 0
            self._takeoff_pos = torch.zeros((self.num_envs, 3), device=gs.device)
            self._takeoff_vel = torch.zeros((self.num_envs, 3), device=gs.device)
            self._peak_height_tracker = torch.zeros(self.num_envs, device=gs.device)
            self._front_takeoff_force = torch.zeros(self.num_envs, device=gs.device)
            self._rear_takeoff_force = torch.zeros(self.num_envs, device=gs.device)
            # One-time reward caches (set on phase transitions, consumed by reward fns)
            self._jump_distance_cache = torch.zeros(self.num_envs, device=gs.device)
            self._peak_height_cache = torch.zeros(self.num_envs, device=gs.device)
            self._landing_quality_cache = torch.zeros(self.num_envs, device=gs.device)
            self._four_foot_landing_cache = torch.zeros(self.num_envs, device=gs.device)
            self._launch_quality_cache = torch.zeros(self.num_envs, device=gs.device)
            self._crouch_timeout_cache = torch.zeros(self.num_envs, device=gs.device)
            self._landing_events_total = torch.tensor(0.0, device=gs.device)

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
                        "position": self._skill_cfg.get("reset_body_position", INITIAL_BODY_POSITION),
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
            kp=float(self._skill_cfg.get("actuator_kp", 20.0)),
            kv=float(self._skill_cfg.get("actuator_kv", 0.5)),
        )
        self.action_manager = PositionActionManager(
            self,
            scale=float(self._skill_cfg.get("action_scale", 0.25)),
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
        vel_range = self._skill_cfg.get(
            "velocity_range",
            {
                "lin_vel_x": (0.0, 0.0),
                "lin_vel_y": (0.0, 0.0),
                "ang_vel_z": (0.0, 0.0),
            },
        )
        vel_resample = self._skill_cfg.get("velocity_resample_time", 5.0)
        vel_stand_prob = self._skill_cfg.get("standing_probability", 1.0)
        self.velocity_command = VelocityCommandManager(
            self,
            range=vel_range,
            standing_probability=vel_stand_prob,
            resample_time_sec=vel_resample,
        )
        if self.skill == "jump":
            # Force explicit command sampling so warm-start curriculum actually controls exploration.
            self.velocity_command.use_external_controller(lambda step: self._jump_cmd_buf)
            self._sample_jump_velocity_commands(force=True)

        # Jump power command (for jump skill)
        self.jump_power_command = None
        if self.skill == "jump":
            jump_power_range = self._skill_cfg.get("jump_power_range", (0.0, 1.0))
            jump_power_resample = self._skill_cfg.get("jump_power_resample_time", 0.75)
            self.jump_power_command = CommandManager(
                self,
                range={"jump_power": jump_power_range},
                resample_time_sec=jump_power_resample,
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
                        "limit_angle": float(self._skill_cfg.get("fall_over_limit_angle", 20.0)),
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
        policy_obs_cfg = {
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
        }
        if self.skill == "jump" and self.jump_power_command is not None:
            policy_obs_cfg["jump_power"] = {
                "fn": self.jump_power_command.observation,
            }
            policy_obs_cfg["jump_phase"] = {
                "fn": lambda env: (self._jump_phase.float() / 4.0).unsqueeze(-1),
            }
        ObservationManager(
            self,
            name="policy",
            history_len=5,
            cfg=policy_obs_cfg,
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
        gait_name = self._skill_cfg.get("gait_name", "walk")
        gait_period = float(self._skill_cfg.get("gait_period", 0.8))
        gait_clearance = float(self._skill_cfg.get("gait_clearance", 0.05))
        self.gait_command_manager.set_fixed_gait(
            gait_name,
            period=gait_period,
            clearance=gait_clearance,
        )

    def reset(self, envs_idx: list[int] | torch.Tensor | None = None):
        reset = super().reset(envs_idx)
        if self.skill == "jump":
            self._ground_contact_steps.zero_()
            self._jump_cmd_override_count.zero_()
            self._jump_cmd_hold_counter = 0
            self._front_takeoff_force.zero_()
            self._rear_takeoff_force.zero_()
            self._jump_distance_cache.zero_()
            self._peak_height_cache.zero_()
            self._landing_quality_cache.zero_()
            self._four_foot_landing_cache.zero_()
            self._launch_quality_cache.zero_()
            self._crouch_timeout_cache.zero_()
            if envs_idx is None:
                self._jump_phase.zero_()
                self._phase_steps.zero_()
                self._jump_progress.zero_()
                self._takeoff_pos.copy_(self.robot.get_pos())
                self._takeoff_vel.zero_()
                self._peak_height_tracker.zero_()
                self._jump_cmd_buf.zero_()
                self._landing_events_total.zero_()
            else:
                if isinstance(envs_idx, torch.Tensor):
                    env_ids = envs_idx.to(device=gs.device, dtype=torch.long)
                else:
                    env_ids = torch.tensor(envs_idx, device=gs.device, dtype=torch.long)
                self._jump_phase[env_ids] = JUMP_PHASE_GROUND
                self._phase_steps[env_ids] = 0
                self._jump_progress[env_ids] = 0.0
                self._takeoff_pos[env_ids] = self.robot.get_pos()[env_ids]
                self._takeoff_vel[env_ids] = 0.0
                self._peak_height_tracker[env_ids] = 0.0
                self._ground_contact_steps[env_ids] = 0
                self._jump_cmd_override_count.zero_()
                self._jump_cmd_hold_counter = 0
                self._jump_cmd_buf[env_ids, :] = 0.0
                self._front_takeoff_force[env_ids] = 0.0
                self._rear_takeoff_force[env_ids] = 0.0
            self._sample_jump_velocity_commands(force=True)
            self._jump_phase_step_id = -1
        return reset

    def step(self, actions: torch.Tensor):
        self._step_id += 1
        self._jump_phase_step_id = -1
        if self.skill == "jump":
            self._sample_jump_velocity_commands()
        obs, rewards, terminated, truncated, extras = super().step(actions)
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
        cmd_dir = self._jump_direction()
        proj = torch.sum(vel[:, :2] * cmd_dir, dim=-1, keepdim=True)
        lat = vel[:, :2] - proj * cmd_dir
        return torch.norm(lat, dim=-1)

    def _forward_velocity_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
        cmd_dir = self._jump_direction()
        proj = torch.sum(vel[:, :2] * cmd_dir, dim=-1)
        return torch.clamp(proj, min=0.0)

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

    def _update_jump_phase(self):
        """Phase-gated jump state machine. Called once per step (lazy)."""
        if self._jump_phase_step_id == self._step_id:
            return
        contact = self._get_foot_contact_forces()
        base_vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
        base_pos = self.robot.get_pos()
        base_vz = base_vel[:, 2]
        progress = self._jump_curriculum_progress()

        contact_threshold = self._interp_cfg(
            "warmup_contact_threshold", "contact_threshold", 1.0, progress,
        )
        air_feet = (contact < contact_threshold).sum(dim=1)
        contact_feet = (contact >= contact_threshold).sum(dim=1)

        self._phase_steps += 1

        # Clear one-time reward caches
        self._jump_distance_cache.zero_()
        self._peak_height_cache.zero_()
        self._landing_quality_cache.zero_()
        self._four_foot_landing_cache.zero_()
        self._launch_quality_cache.zero_()
        self._crouch_timeout_cache.zero_()

        standing_h = float(self._skill_cfg.get(
            "reset_body_position", INITIAL_BODY_POSITION,
        )[2])
        cmd_dir = self._jump_direction()

        # -- GROUND -> CROUCH --
        crouch_threshold = self._interp_cfg(
            "warmup_crouch_threshold", "crouch_threshold", 0.30, progress,
        )
        ground_to_crouch = (
            (self._jump_phase == JUMP_PHASE_GROUND)
            & (base_pos[:, 2] < crouch_threshold)
        )
        if ground_to_crouch.any():
            self._jump_phase[ground_to_crouch] = JUMP_PHASE_CROUCH
            self._phase_steps[ground_to_crouch] = 0

        # -- CROUCH -> FLIGHT (launch) --
        forward_vel_min = self._interp_cfg(
            "warmup_forward_vel_gate", "forward_vel_gate", 0.5, progress,
        )
        vz_min = float(self._skill_cfg.get("launch_vz_min", 0.3))
        forward_vel = torch.sum(base_vel[:, :2] * cmd_dir, dim=-1)
        launch_feet_min = int(round(self._interp_cfg(
            "warmup_launch_feet_min", "launch_feet_min", 3.0, progress,
        )))
        crouch_min_steps = int(self._skill_cfg.get("crouch_min_steps", 3))

        crouch_to_flight = (
            (self._jump_phase == JUMP_PHASE_CROUCH)
            & (self._phase_steps >= crouch_min_steps)
            & (base_vz > vz_min)
            & (forward_vel > forward_vel_min)
            & (air_feet >= launch_feet_min)
        )

        # Cache launch forces
        front_force = contact[:, :2].mean(dim=1)
        rear_force = contact[:, 2:].mean(dim=1)

        if crouch_to_flight.any():
            self._jump_phase[crouch_to_flight] = JUMP_PHASE_FLIGHT
            self._phase_steps[crouch_to_flight] = 0
            self._takeoff_pos[crouch_to_flight] = base_pos[crouch_to_flight]
            self._takeoff_vel[crouch_to_flight] = base_vel[crouch_to_flight]
            self._peak_height_tracker[crouch_to_flight] = base_pos[crouch_to_flight, 2]
            self._front_takeoff_force[crouch_to_flight] = front_force[crouch_to_flight]
            self._rear_takeoff_force[crouch_to_flight] = rear_force[crouch_to_flight]

        # -- CROUCH timeout --
        crouch_max_steps = int(round(self._interp_cfg(
            "warmup_crouch_max_steps", "crouch_max_steps", 25.0, progress,
        )))
        crouch_timeout = (
            (self._jump_phase == JUMP_PHASE_CROUCH)
            & (self._phase_steps > crouch_max_steps)
        )
        if crouch_timeout.any():
            self._jump_phase[crouch_timeout] = JUMP_PHASE_GROUND
            self._phase_steps[crouch_timeout] = 0
            self._crouch_timeout_cache[crouch_timeout] = 1.0

        # -- Track peak height during FLIGHT --
        in_flight = self._jump_phase == JUMP_PHASE_FLIGHT
        if in_flight.any():
            self._peak_height_tracker = torch.where(
                in_flight,
                torch.maximum(self._peak_height_tracker, base_pos[:, 2]),
                self._peak_height_tracker,
            )

        # -- FLIGHT invalidation --
        flight_timeout = int(round(self._interp_cfg(
            "warmup_flight_timeout", "flight_timeout", 20.0, progress,
        )))
        flight_base_min = float(self._skill_cfg.get(
            "flight_base_height_min", standing_h + 0.07,
        ))
        flight_invalid = in_flight & (
            (self._phase_steps > flight_timeout)
            | (
                (contact_feet > 0)
                & (base_pos[:, 2] < flight_base_min)
                & (self._phase_steps > 1)
            )
        )
        if flight_invalid.any():
            self._jump_phase[flight_invalid] = JUMP_PHASE_GROUND
            self._phase_steps[flight_invalid] = 0

        # -- FLIGHT -> LANDED --
        landing_feet_min = int(round(self._interp_cfg(
            "warmup_landing_feet_min", "landing_feet_min", 3.0, progress,
        )))
        min_flight_steps = int(round(self._interp_cfg(
            "warmup_min_flight_steps", "min_flight_steps", 3.0, progress,
        )))
        in_flight = self._jump_phase == JUMP_PHASE_FLIGHT
        self._ground_contact_steps = torch.where(
            contact_feet >= landing_feet_min,
            self._ground_contact_steps + 1,
            torch.zeros_like(self._ground_contact_steps),
        )
        landing_confirm = int(self._skill_cfg.get("landing_confirm_steps", 2))
        landed = (
            in_flight
            & (self._phase_steps >= min_flight_steps)
            & (self._ground_contact_steps >= landing_confirm)
            & (base_vz < 0)
        )

        if landed.any():
            # Score: jump distance
            delta = base_pos[landed, :2] - self._takeoff_pos[landed, :2]
            cmd_dir_l = cmd_dir[landed]
            distance = torch.sum(delta * cmd_dir_l, dim=-1).clamp(min=0.0)
            target_dist = self._jump_target_distance()[landed]
            dist_score = torch.clamp(distance / target_dist, max=1.0)
            min_dist = float(self._skill_cfg.get("min_scoring_distance", 0.1))
            dist_score = torch.where(
                distance < min_dist, torch.zeros_like(dist_score), dist_score,
            )
            self._jump_distance_cache[landed] = dist_score

            # Score: peak height
            target_height = self._jump_target_height()[landed]
            peak_delta = (self._peak_height_tracker[landed] - standing_h).clamp(min=0.0)
            target_h_delta = (target_height - standing_h).clamp(min=0.01)
            self._peak_height_cache[landed] = torch.clamp(
                peak_delta / target_h_delta, max=1.0,
            )

            # Score: landing quality (orientation + angular velocity)
            proj_grav = self._flatten_vec(self.robot_manager.get_projected_gravity())
            tilt_quality = proj_grav[landed, 2].abs()
            ang_vel = torch.norm(
                self._flatten_vec(self.robot_manager.get_angular_velocity())[landed],
                dim=-1,
            )
            ang_vel_quality = torch.clamp(1.0 - ang_vel / 3.0, min=0.0, max=1.0)
            self._landing_quality_cache[landed] = tilt_quality * ang_vel_quality

            # Score: four-foot landing bonus
            self._four_foot_landing_cache[landed] = (contact_feet[landed] >= 4).float()

            # Score: launch quality (rear-dominant push)
            rear_dom = torch.clamp(
                self._rear_takeoff_force[landed] - self._front_takeoff_force[landed],
                min=0.0,
            )
            denom = float(self._skill_cfg.get("rear_launch_force_denom", 80.0))
            self._launch_quality_cache[landed] = torch.clamp(rear_dom / denom, max=1.0)

            # Transition to LANDED
            self._jump_phase[landed] = JUMP_PHASE_LANDED
            self._phase_steps[landed] = 0
            self._landing_events_total += landed.float().sum()

        # -- LANDED -> RECOVERY (immediate) --
        just_landed = self._jump_phase == JUMP_PHASE_LANDED
        if just_landed.any():
            self._jump_phase[just_landed] = JUMP_PHASE_RECOVERY
            self._phase_steps[just_landed] = 0

        # -- RECOVERY -> GROUND --
        recovery_min_steps = int(round(self._interp_cfg(
            "warmup_recovery_min_steps", "recovery_min_steps", 10.0, progress,
        )))
        in_recovery = self._jump_phase == JUMP_PHASE_RECOVERY
        recovery_done = (
            in_recovery
            & (self._phase_steps >= recovery_min_steps)
            & (contact_feet >= 3)
            & (base_pos[:, 2] > standing_h - 0.05)
            & (base_pos[:, 2] < standing_h + 0.05)
        )
        if recovery_done.any():
            self._jump_phase[recovery_done] = JUMP_PHASE_GROUND
            self._phase_steps[recovery_done] = 0

        # Metrics
        self.extras[self.extras_logging_key]["Metrics / jump_phase_ground"] = (
            self._jump_phase == JUMP_PHASE_GROUND
        ).float().sum()
        self.extras[self.extras_logging_key]["Metrics / jump_phase_crouch"] = (
            self._jump_phase == JUMP_PHASE_CROUCH
        ).float().sum()
        self.extras[self.extras_logging_key]["Metrics / jump_phase_flight"] = (
            self._jump_phase == JUMP_PHASE_FLIGHT
        ).float().sum()
        self.extras[self.extras_logging_key]["Metrics / jump_phase_recovery"] = (
            self._jump_phase == JUMP_PHASE_RECOVERY
        ).float().sum()
        self.extras[self.extras_logging_key]["Metrics / jump_landings"] = (
            landed.float().sum()
        )
        self.extras[self.extras_logging_key]["Metrics / jump_crouch_timeouts"] = (
            crouch_timeout.float().sum()
        )
        self.extras[self.extras_logging_key]["Metrics / jump_flight_invalidations"] = (
            flight_invalid.float().sum()
        )
        self.extras[self.extras_logging_key]["Metrics / jump_air_feet_mean"] = (
            air_feet.float().mean()
        )
        self.extras[self.extras_logging_key]["Metrics / jump_contact_feet_mean"] = (
            contact_feet.float().mean()
        )
        self.extras[self.extras_logging_key]["Metrics / jump_curriculum_progress"] = (
            torch.tensor(progress, device=gs.device)
        )
        self.extras[self.extras_logging_key]["Metrics / jump_cmd_speed_mean"] = (
            torch.norm(self._jump_cmd_buf[:, :2], dim=-1).mean()
        )
        self.extras[self.extras_logging_key]["Metrics / jump_landing_events_total"] = (
            self._landing_events_total
        )
        self._jump_phase_step_id = self._step_id

    def _jump_target_height(self) -> torch.Tensor:
        min_h = self._skill_cfg.get("min_jump_height", 0.2)
        max_h = self._skill_cfg.get("max_jump_height", self._skill_cfg.get("target_height", 0.4))
        power = self._jump_power()
        return min_h + (max_h - min_h) * power

    def _jump_target_distance(self) -> torch.Tensor:
        min_d = self._skill_cfg.get("min_jump_distance", 0.2)
        max_d = self._skill_cfg.get("max_jump_distance", self._skill_cfg.get("target_distance", 0.4))
        power = self._jump_power()
        return min_d + (max_d - min_d) * power

    def _jump_power(self) -> torch.Tensor:
        if self.jump_power_command is None:
            return torch.ones(self.num_envs, device=gs.device)
        power = self.jump_power_command.command[:, 0]
        progress = self._jump_curriculum_progress()
        warmup_min = float(self._skill_cfg.get("warmup_min_jump_power", 0.0))
        final_min = float(self._skill_cfg.get("final_min_jump_power", self._skill_cfg.get("jump_power_range", (0.0, 1.0))[0]))
        dynamic_min = warmup_min + (final_min - warmup_min) * progress
        return torch.clamp(power, min=dynamic_min, max=1.0)

    def _jump_direction(self) -> torch.Tensor:
        cmd = self.velocity_command.command[:, :2]
        norm = torch.norm(cmd, dim=-1, keepdim=True)
        default = torch.zeros_like(cmd)
        default[:, 0] = 1.0
        return torch.where(norm > 1e-3, cmd / norm, default)

    def _jump_curriculum_progress(self) -> float:
        warmup_steps = int(self._skill_cfg.get("warmup_steps", 0))
        if warmup_steps <= 0:
            return 1.0
        return min(1.0, float(self._step_id) / float(warmup_steps))

    def _interp_cfg(self, start_key: str, end_key: str, default: float, progress: float) -> float:
        start = float(self._skill_cfg.get(start_key, default))
        end = float(self._skill_cfg.get(end_key, start))
        return start + (end - start) * progress

    def _curriculum_gate(self, start_key: str, end_key: str, default_start: float, default_end: float) -> float:
        progress = self._jump_curriculum_progress()
        start = float(self._skill_cfg.get(start_key, default_start))
        end = float(self._skill_cfg.get(end_key, default_end))
        if end <= start:
            return 1.0 if progress >= end else 0.0
        return max(0.0, min(1.0, (progress - start) / (end - start)))

    def _sample_jump_velocity_commands(self, force: bool = False):
        progress = self._jump_curriculum_progress()
        hold_steps = int(round(self._interp_cfg("warmup_cmd_hold_steps", "final_cmd_hold_steps", 10.0, progress)))
        hold_steps = max(1, hold_steps)
        if not force and self._jump_cmd_hold_counter > 0:
            self._jump_cmd_hold_counter -= 1
            return

        speed_min = self._interp_cfg("warmup_cmd_min_speed", "final_cmd_min_speed", 0.5, progress)
        speed_max = self._interp_cfg("warmup_cmd_max_speed", "final_cmd_max_speed", 2.0, progress)
        ang_max = self._interp_cfg("warmup_cmd_ang_max", "final_cmd_ang_max", 0.5, progress)
        lat_scale = self._interp_cfg("warmup_cmd_lat_scale", "final_cmd_lat_scale", 0.5, progress)
        stand_prob = self._interp_cfg("warmup_standing_probability", "final_standing_probability", 0.0, progress)

        dirs = torch.randn((self.num_envs, 2), device=gs.device)
        dirs = dirs / torch.norm(dirs, dim=-1, keepdim=True).clamp_min(1e-6)
        speeds = torch.empty(self.num_envs, device=gs.device).uniform_(speed_min, speed_max)
        stand_mask = torch.rand(self.num_envs, device=gs.device) < stand_prob
        speeds = torch.where(stand_mask, torch.zeros_like(speeds), speeds)
        lin_x = dirs[:, 0] * speeds
        lin_y = dirs[:, 1] * speeds * lat_scale

        x_min, x_max = self.velocity_command.range["lin_vel_x"]
        y_min, y_max = self.velocity_command.range["lin_vel_y"]
        z_min, z_max = self.velocity_command.range["ang_vel_z"]
        self._jump_cmd_buf[:, 0] = lin_x.clamp(min=x_min, max=x_max)
        self._jump_cmd_buf[:, 1] = lin_y.clamp(min=y_min, max=y_max)
        self._jump_cmd_buf[:, 2] = torch.empty(self.num_envs, device=gs.device).uniform_(-ang_max, ang_max).clamp(min=z_min, max=z_max)
        self._jump_cmd_buf[stand_mask, 2] = 0.0

        moving = torch.norm(self._jump_cmd_buf[:, :2], dim=-1) > 1e-3
        self._jump_cmd_override_count.fill_(float(moving.sum().item()))
        self._jump_cmd_hold_counter = hold_steps

    # -- Phase-gated jump reward functions --

    def _crouch_depth_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """Per-step reward during CROUCH: reward lowering center of mass."""
        self._update_jump_phase()
        standing_h = float(self._skill_cfg.get(
            "reset_body_position", INITIAL_BODY_POSITION,
        )[2])
        base_z = self.robot.get_pos()[:, 2]
        depth = torch.clamp((standing_h - base_z) / 0.08, min=0.0, max=1.0)
        in_crouch = (self._jump_phase == JUMP_PHASE_CROUCH).float()
        return depth * in_crouch

    def _crouch_symmetry_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """Per-step reward during CROUCH: front and rear legs compress equally."""
        self._update_jump_phase()
        dof_pos = self.action_manager.get_dofs_position()
        # Thigh joints: FL=1, FR=4, RL=7, RR=10 (0-indexed in 12-DOF Go2)
        front_thigh = (dof_pos[:, 1] + dof_pos[:, 4]) / 2.0
        rear_thigh = (dof_pos[:, 7] + dof_pos[:, 10]) / 2.0
        symmetry = torch.clamp(1.0 - (front_thigh - rear_thigh).abs() / 0.5, min=0.0)
        in_crouch = (self._jump_phase == JUMP_PHASE_CROUCH).float()
        return symmetry * in_crouch

    def _crouch_timeout_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """One-time penalty when CROUCH times out."""
        self._update_jump_phase()
        return self._crouch_timeout_cache

    def _jump_distance_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """One-time reward on LANDED: forward distance covered."""
        self._update_jump_phase()
        return self._jump_distance_cache

    def _peak_height_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """One-time reward on LANDED: peak height during flight."""
        self._update_jump_phase()
        return self._peak_height_cache

    def _landing_quality_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """One-time reward on LANDED: orientation + low angular velocity."""
        self._update_jump_phase()
        return self._landing_quality_cache

    def _four_foot_landing_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """One-time bonus on LANDED: all 4 feet touch down."""
        self._update_jump_phase()
        return self._four_foot_landing_cache

    def _launch_quality_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """One-time reward on LANDED: rear-dominant push at takeoff."""
        self._update_jump_phase()
        return self._launch_quality_cache

    def _recovery_stability_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """Per-step reward during RECOVERY: stabilize after landing."""
        self._update_jump_phase()
        in_recovery = (self._jump_phase == JUMP_PHASE_RECOVERY).float()
        return self._stillness_bonus(env) * in_recovery

    def _phase_pose_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        """Per-step pose penalty during GROUND and RECOVERY phases."""
        self._update_jump_phase()
        penalty = rewards.dof_similar_to_default(env, action_manager=self.action_manager)
        active = (
            (self._jump_phase == JUMP_PHASE_GROUND)
            | (self._jump_phase == JUMP_PHASE_RECOVERY)
        ).float()
        return penalty * active

    def _phase_stillness_reward(self, env: ManagedEnvironment) -> torch.Tensor:
        """Per-step stillness reward during GROUND phase."""
        self._update_jump_phase()
        in_ground = (self._jump_phase == JUMP_PHASE_GROUND).float()
        return self._stillness_bonus(env) * in_ground

    def _energy_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        """Per-step energy penalty: sum(|torque * dof_vel|)."""
        dof_vel = self.action_manager.get_dofs_velocity()
        dof_force = self.action_manager.get_dofs_force()
        return torch.sum(torch.abs(dof_force * dof_vel), dim=-1)

    def _orientation_penalty(self, env: ManagedEnvironment) -> torch.Tensor:
        """Per-step penalty for excessive body tilt."""
        proj_grav = self._flatten_vec(self.robot_manager.get_projected_gravity())
        # When level, |gz| ~ 1.0. Penalize deviation.
        tilt = 1.0 - proj_grav[:, 2].abs()
        return tilt

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
                # Per-step: GROUND phase
                "pose_target": {
                    "weight": self._skill_cfg["pose_weight"],
                    "fn": self._phase_pose_penalty,
                },
                "stillness": {
                    "weight": self._skill_cfg.get("stillness_weight", 0.3),
                    "fn": self._phase_stillness_reward,
                },
                # Per-step: CROUCH phase
                "crouch_depth": {
                    "weight": self._skill_cfg["crouch_depth_weight"],
                    "fn": self._crouch_depth_reward,
                },
                "crouch_symmetry": {
                    "weight": self._skill_cfg["crouch_symmetry_weight"],
                    "fn": self._crouch_symmetry_reward,
                },
                "crouch_timeout": {
                    "weight": self._skill_cfg["crouch_timeout_weight"],
                    "fn": self._crouch_timeout_reward,
                },
                # One-time: LANDED rewards
                "jump_distance": {
                    "weight": self._skill_cfg["jump_distance_weight"],
                    "fn": self._jump_distance_reward,
                },
                "peak_height": {
                    "weight": self._skill_cfg["peak_height_weight"],
                    "fn": self._peak_height_reward,
                },
                "landing_quality": {
                    "weight": self._skill_cfg["landing_quality_weight"],
                    "fn": self._landing_quality_reward,
                },
                "four_foot_landing": {
                    "weight": self._skill_cfg["four_foot_landing_weight"],
                    "fn": self._four_foot_landing_reward,
                },
                "launch_quality": {
                    "weight": self._skill_cfg["launch_quality_weight"],
                    "fn": self._launch_quality_reward,
                },
                # Per-step: RECOVERY phase
                "recovery_stability": {
                    "weight": self._skill_cfg["recovery_stability_weight"],
                    "fn": self._recovery_stability_reward,
                },
                # Per-step: always-on penalties
                "body_contact": {
                    "weight": self._skill_cfg["body_contact_weight"],
                    "fn": rewards.contact_force,
                    "params": {
                        "contact_manager": self.body_contact_manager,
                    },
                },
                "bad_orientation": {
                    "weight": self._skill_cfg["bad_orientation_weight"],
                    "fn": self._orientation_penalty,
                },
                "action_rate": {
                    "weight": self._skill_cfg["action_rate_weight"],
                    "fn": rewards.action_rate_l2,
                },
                "energy": {
                    "weight": self._skill_cfg["energy_weight"],
                    "fn": self._energy_penalty,
                },
                "bad_contact": {
                    "weight": self._skill_cfg.get("bad_contact_weight", -2.0),
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
                # Start from a stable standing reset.
                "default_pos": STAND_DEFAULT_POS,
                "reset_body_position": [0.0, 0.0, 0.35],
                # Wide action range for crouch-to-extend motion.
                "action_scale": 1.5,
                # High motor authority for ballistic motion.
                "actuator_kp": 100.0,
                "actuator_kv": 2.5,
                "min_jump_height": 0.25,
                "max_jump_height": 0.9144,
                "min_jump_distance": 0.3,
                "max_jump_distance": 0.9144,
                "gait_name": "pronk",
                "gait_period": 0.5,
                "gait_clearance": 0.10,
                "fall_over_limit_angle": 25.0,
                # Phase-gated reward weights
                "pose_weight": -0.5,
                "stillness_weight": 0.3,
                "crouch_depth_weight": 2.0,
                "crouch_symmetry_weight": 1.0,
                "crouch_timeout_weight": -1.0,
                "jump_distance_weight": 50.0,
                "peak_height_weight": 15.0,
                "landing_quality_weight": 20.0,
                "four_foot_landing_weight": 10.0,
                "launch_quality_weight": 8.0,
                "recovery_stability_weight": 1.0,
                "body_contact_weight": -12.0,
                "bad_orientation_weight": -3.0,
                "action_rate_weight": -0.01,
                "energy_weight": -0.002,
                "bad_contact_weight": -2.0,
                # Phase transition parameters
                "launch_vz_min": 0.3,
                "crouch_min_steps": 3,
                "flight_base_height_min": 0.42,
                "min_scoring_distance": 0.1,
                "rear_launch_force_denom": 80.0,
                "landing_confirm_steps": 2,
                # Velocity command config
                "velocity_range": {
                    "lin_vel_x": (-4.0, 4.0),
                    "lin_vel_y": (-1.25, 1.25),
                    "ang_vel_z": (-1.25, 1.25),
                },
                "standing_probability": 0.35,
                "velocity_resample_time": 0.7,
                # Curriculum (step-based warmup over 6000 steps)
                "warmup_steps": 6000,
                "warmup_cmd_hold_steps": 8,
                "final_cmd_hold_steps": 20,
                "warmup_cmd_min_speed": 1.2,
                "warmup_cmd_max_speed": 3.2,
                "final_cmd_min_speed": 0.2,
                "final_cmd_max_speed": 2.0,
                "warmup_cmd_ang_max": 0.25,
                "final_cmd_ang_max": 0.75,
                "warmup_cmd_lat_scale": 0.25,
                "final_cmd_lat_scale": 0.5,
                "warmup_standing_probability": 0.05,
                "final_standing_probability": 0.35,
                "warmup_min_jump_power": 0.2,
                "final_min_jump_power": 0.5,
                # Phase transition curriculum
                "warmup_contact_threshold": 0.12,
                "contact_threshold": 0.25,
                "warmup_crouch_threshold": 0.34,
                "crouch_threshold": 0.30,
                "warmup_forward_vel_gate": 0.1,
                "forward_vel_gate": 0.5,
                "warmup_launch_feet_min": 2,
                "launch_feet_min": 3,
                "warmup_landing_feet_min": 2,
                "landing_feet_min": 3,
                "warmup_min_flight_steps": 2,
                "min_flight_steps": 3,
                "warmup_flight_timeout": 30,
                "flight_timeout": 20,
                "warmup_crouch_max_steps": 35,
                "crouch_max_steps": 25,
                "warmup_recovery_min_steps": 5,
                "recovery_min_steps": 10,
                "jump_power_range": (0.0, 1.0),
                "jump_power_resample_time": 0.5,
            }
        if skill == "adaptive_low_crawl":
            return {
                "default_pos": JUMP_CROUCH_DEFAULT_POS,
                "min_jump_height": 0.25,
                "max_jump_height": 0.9144,
                "min_jump_distance": 0.3,
                "max_jump_distance": 0.9144,
                "gait_name": "walk",
                "gait_period": 0.8,
                "gait_clearance": 0.05,
                "base_height_weight": -2.0,
                "pose_weight": -0.8,
                "ang_vel_weight": -0.6,
                "action_rate_weight": -0.02,
                "dof_vel_weight": -0.05,
                "forward_vel_weight": 4.0,
                "distance_progress_weight": 10.0,
                "lat_vel_weight": -2.5,
                "airborne_forward_bonus_weight": 1.5,
                "foot_height_bonus_weight": 0.0,
                "landing_bonus_weight": 6.0,
                "landing_distance_weight": 10.0,
                "track_lin_vel_weight": 2.0,
                "track_ang_vel_weight": 0.2,
                "distance_progress_on_landing": False,
                "velocity_range": {
                    "lin_vel_x": (-2.0, 2.0),
                    "lin_vel_y": (-1.5, 1.5),
                    "ang_vel_z": (-1.0, 1.0),
                },
                "standing_probability": 0.1,
                "velocity_resample_time": 1.0,
                "contact_threshold": 1.0,
                "airborne_feet_min": 3,
                "landing_feet_min": 1,
                "min_airborne_steps": 0,
                "landing_cooldown_steps": 0,
                "airborne_forward_target": 1.0,
                "jump_power_range": (0.0, 1.0),
                "jump_power_resample_time": 0.75,
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


class Go2AdaptiveLowCrawlEnv(Go2SkillEnv):
    def __init__(self, *args, **kwargs):
        super().__init__("jump", profile="adaptive_low_crawl", *args, **kwargs)
