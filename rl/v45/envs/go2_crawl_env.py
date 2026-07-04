"""v45.0 — Go2 LL-Crawl primitive: height-commanded locomotion (flat ground, NO barrier).

The crawl mirror of the v44 "no bar" decision: the robot is commanded a body height h*
and forward speed vx* (CrawlCommandManager) and rewarded for genuine low locomotion via
a MULTIPLICATIVE height x velocity tracking term (rl/v45/envs/crawl_rewards.py).
Commanded low = crawl; the obstacle is HL's job (Phase F). Constraints are enforced by
CaT terminations (virtual ceiling, belly-drag — arXiv:2403.18765), not penalties.
Spec: artifacts/knowledge_base/experiments/v45.0-ll-crawl-spec.md.

Architecture cloned from the validated rl/v44/envs/go2_launch_env.py.
"""

import os
import torch
import genesis as gs

from genesis_forge import ManagedEnvironment
from genesis_forge.managers import (
    RewardManager, TerminationManager, EntityManager, ObservationManager,
    ActuatorManager, PositionActionManager, ContactManager,
)
from genesis_forge.mdp import reset, rewards, terminations, observations

from rl.v45.envs.crawl_command_manager import CrawlCommandManager, CRAWL_STAGES
from rl.v45.envs import crawl_rewards as R

HEIGHT_OFFSET = 0.4
INITIAL_BODY_POSITION = [0.0, 0.0, HEIGHT_OFFSET]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]


class Go2CrawlEnv(ManagedEnvironment):
    def __init__(self, num_envs: int = 1, dt: float = 1 / 50,
                 max_episode_length_s: float = 8.0, headless: bool = True):
        super().__init__(num_envs=num_envs, dt=dt,
                         max_episode_length_sec=max_episode_length_s,
                         max_episode_random_scaling=0.2)
        self._curriculum_stage = 1
        self.scene = gs.Scene(
            show_viewer=not headless,
            sim_options=gs.options.SimOptions(dt=self.dt, substeps=2),
            viewer_options=gs.options.ViewerOptions(
                max_FPS=int(0.5 / self.dt), camera_pos=(2.0, 0.0, 2.5),
                camera_lookat=(0.0, 0.0, 0.5), camera_fov=40),
            vis_options=gs.options.VisOptions(rendered_envs_idx=list(range(1))),
            rigid_options=gs.options.RigidOptions(
                dt=self.dt, constraint_solver=gs.constraint_solver.Newton,
                enable_collision=True, enable_joint_limit=True, max_collision_pairs=60),
        )
        self.terrain = self.scene.add_entity(gs.morphs.Plane())
        self.robot = self.scene.add_entity(gs.morphs.URDF(
            file="urdf/go2/urdf/go2.urdf", pos=INITIAL_BODY_POSITION, quat=INITIAL_QUAT,
            links_to_keep=["FL_foot", "FR_foot", "RL_foot", "RR_foot"]))
        # Side camera is the informative view for crawl height.
        self.camera = self.scene.add_camera(pos=(2.5, 1.5, 0.8), lookat=(1.0, 0.0, 0.25),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self.camera_side = self.scene.add_camera(pos=(1.0, -2.5, 0.4), lookat=(1.0, 0.0, 0.22),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        # Sidecar plumbing (set by trainer if a VideoWrapper is attached).
        self._video_wrapper_ref = None
        self._video_meta_buffer = []
        self._video_meta_frame_idx = 0
        self._video_meta_was_recording = False
        self._video_episode_id = 0
        self._step_id = 0

    def config(self):
        self.robot_manager = EntityManager(self, entity_attr="robot", on_reset={
            "position": {"fn": reset.position, "params": {
                "position": INITIAL_BODY_POSITION, "quat": INITIAL_QUAT,
                "zero_velocity": True}}})
        self.actuator_manager = ActuatorManager(self,
            joint_names=["FL_.*_joint", "FR_.*_joint", "RL_.*_joint", "RR_.*_joint"],
            default_pos={".*_hip_joint": 0.0, "FL_thigh_joint": 0.8, "FR_thigh_joint": 0.8,
                         "RL_thigh_joint": 1.0, "RR_thigh_joint": 1.0, ".*_calf_joint": -1.5},
            kp=20, kv=0.5)
        self.action_manager = PositionActionManager(self, scale=0.25,
            use_default_offset=True, actuator_manager=self.actuator_manager)
        self.foot_contact_manager = ContactManager(self, link_names=[".*_foot"],
            air_time_contact_threshold=1.0)
        self.body_contact_manager = ContactManager(self, link_names=["base"],
            air_time_contact_threshold=1.0)
        self.crawl_command = CrawlCommandManager(self, device=gs.device)
        self.crawl_command.allocate()  # before ObservationManager.build()

        # CaT grace counters (spec §2: structural constraints, not penalties).
        n, dev = self.num_envs, gs.device
        self._ceiling_run = torch.zeros(n, dtype=torch.long, device=dev)
        self._belly_run = torch.zeros(n, dtype=torch.long, device=dev)
        # v45.0 spawn-transient fix: the robot spawns at z=0.40 and settles THROUGH the
        # virtual ceiling band (e.g. 0.32 for h*=0.26) — far longer than the 5-step grace,
        # which would mass-terminate every episode at t=0. The ceiling CaT therefore ARMS
        # only once the env first complies (z <= h*+margin); after arming, popping back
        # above the ceiling terminates as designed. Standing forever (never arming) is not
        # an attractor: the multiplicative track term pays up to 3.0/step on a smooth
        # gradient down into the band vs alive's 0.1/step.
        self._ceiling_armed = torch.zeros(n, dtype=torch.bool, device=dev)

        # Reward budget (spec §3): perfect crawl ~ (3.0*0.8 + 0.1)*400 steps; standing earns
        # ~4% of crawling because the multiplicative track term zeroes on unmet velocity.
        # SIGN CONVENTION (gate BLOCK-2 resolution): `energy` and `posture` fns return
        # SIGNED (<=0) values with their coefficients baked in (energy: -2e-5*sum|tau*qd|),
        # so their env weights are +1.0. `action_rate` follows the genesis_forge stock
        # convention (unsigned fn, negative weight). Do NOT change a fn to unsigned without
        # flipping its weight here — tests/v45 asserts the fn signs.
        self.reward_manager = RewardManager(self, logging_enabled=True, cfg={
            "track":       {"weight": 3.0,   "fn": self._r_track},
            "alive":       {"weight": 0.1,   "fn": rewards.is_alive},
            "energy":      {"weight": 1.0,   "fn": self._r_energy},     # signed fn, coef inside
            "action_rate": {"weight": -0.01, "fn": rewards.action_rate_l2},
            "posture":     {"weight": 1.0,   "fn": self._r_posture},    # signed fn
        })
        self.termination_manager = TerminationManager(self, logging_enabled=True, term_cfg={
            "timeout": {"fn": terminations.timeout, "time_out": True},
            "fall_over": {"fn": terminations.bad_orientation,
                          "params": {"limit_angle": 50.0, "entity_manager": self.robot_manager}},
            # CaT (arXiv:2403.18765): constraint violations terminate (with a grace window)
            # instead of accumulating farmable penalties.
            "ceiling": {"fn": self._t_ceiling},
            "belly_drag": {"fn": self._t_belly_drag},
        })
        ObservationManager(self, name="policy", history_len=5, cfg={
            "command": {"fn": self.crawl_command.observation},                          # 2
            "angle_velocity": {"fn": lambda env: self.robot_manager.get_angular_velocity()},     # 3
            "projected_gravity": {"fn": lambda env: self.robot_manager.get_projected_gravity()}, # 3
            "dof_position": {"fn": lambda env: self.action_manager.get_dofs_position()},   # 12
            "dof_velocity": {"fn": lambda env: self.action_manager.get_dofs_velocity(), "scale": 0.05}, # 12
            "actions": {"fn": lambda env: self.action_manager.get_actions()},              # 12
            "base_height": {"fn": self._obs_base_height},                                  # 1
        })
        ObservationManager(self, name="critic", history_len=5, cfg={
            "foot_contact_force": {"fn": observations.contact_force,
                                   "params": {"contact_manager": self.foot_contact_manager}},
            "base_lin_vel": {"fn": lambda env: self.robot_manager.get_linear_velocity()},
            "dof_force": {"fn": observations.entity_dofs_force,
                          "params": {"action_manager": self.action_manager}, "scale": 0.1},
        })

    # ── curriculum ────────────────────────────────────────────────────────────
    def set_curriculum_stage(self, stage: int):
        self._curriculum_stage = stage
        if hasattr(self, "crawl_command") and self.crawl_command.command is not None:
            self.crawl_command.set_curriculum_stage(stage)

    def get_stage_gate_metric(self, name: str) -> float:
        return self.reward_manager.last_episode_mean_reward(name)

    # ── build / reset / step ─────────────────────────────────────────────────
    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)
        self.set_curriculum_stage(self._curriculum_stage)
        print(f"[v45.0] LL-Crawl stage={self._curriculum_stage} "
              f"bands={CRAWL_STAGES[self._curriculum_stage]}", flush=True)

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        if envs_idx is None:
            envs_idx = torch.arange(self.num_envs, device=gs.device)
        self.crawl_command.reset(envs_idx)
        self._ceiling_run[envs_idx] = 0
        self._belly_run[envs_idx] = 0
        self._ceiling_armed[envs_idx] = False
        return result

    def step(self, actions):
        self._update_cat_counters()
        obs, rew, terminated, truncated, infos = super().step(actions)
        dones = terminated | truncated
        self._step_id += 1
        if dones is not None and dones.numel() > 0 and dones[0].item():
            self._video_episode_id += 1
        self._buffer_video_metadata()
        return obs, rew, terminated, truncated, infos

    # ── helpers ───────────────────────────────────────────────────────────────
    def _z(self):
        return self.robot.get_pos()[:, 2]

    def _vx(self):
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        return vel[:, 0]

    def _base_contact_force(self):
        if self.body_contact_manager.contacts is None:
            return torch.zeros(self.num_envs, device=gs.device)
        f = torch.norm(self.body_contact_manager.contacts, dim=-1)  # (n, n_links)
        return f.view(self.num_envs, -1).max(dim=-1).values

    def _update_cat_counters(self):
        """Maintain CaT grace counters from the previous physics step's state."""
        z = self._z()
        h_star = self.crawl_command.command[:, 0]
        # Spawn-transient fix: arm the ceiling only after first compliance (see config()).
        self._ceiling_armed |= z <= (h_star + R._CEILING_MARGIN)
        viol_c = R.ceiling_violation(z, h_star) & self._ceiling_armed
        self._ceiling_run = torch.where(
            viol_c, self._ceiling_run + 1, torch.zeros_like(self._ceiling_run))
        viol_b = R.belly_drag(self._base_contact_force(), self._vx())
        self._belly_run = torch.where(
            viol_b, self._belly_run + 1, torch.zeros_like(self._belly_run))

    # ── reward fns ────────────────────────────────────────────────────────────
    def _r_track(self, env):
        return R.track(self._z(), self.crawl_command.command[:, 0],
                       self._vx(), self.crawl_command.command[:, 1])

    def _r_energy(self, env):
        torques = self.action_manager.get_dofs_force()
        dof_vel = self.action_manager.get_dofs_velocity()
        return R.energy(torques, dof_vel)

    def _r_posture(self, env):
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        return R.posture(gz)

    # ── CaT termination fns ───────────────────────────────────────────────────
    def _t_ceiling(self, env):
        return self._ceiling_run >= R._CEILING_GRACE_STEPS

    def _t_belly_drag(self, env):
        return self._belly_run >= R._BELLY_GRACE_STEPS

    def _obs_base_height(self, env):
        return self.robot.get_pos()[:, 2:3]

    # ── video metadata sidecar (crawl phases) ─────────────────────────────────
    def _buffer_video_metadata(self):
        vw = self._video_wrapper_ref
        if vw is None:
            return
        is_recording = vw._is_recording and vw._recording_type == "active"
        if is_recording and not self._video_meta_was_recording:
            self._video_meta_buffer = []
            self._video_meta_frame_idx = 0
        if is_recording:
            steps_since_start = self._step_id - vw._recording_start_step
            if steps_since_start >= 0 and steps_since_start % vw._steps_per_frame == 0:
                pos = self.robot.get_pos()
                z = float(pos[0, 2].item())
                h_star = float(self.crawl_command.command[0, 0].item())
                vel = self.robot_manager.get_linear_velocity()
                if vel.ndim > 2:
                    vel = vel.view(vel.shape[0], -1)
                gz = float(self.robot_manager.get_projected_gravity()[0, 2].item())
                phase = "crawl" if z < 0.28 else "stand"
                events = []
                if self._belly_run[0].item() > 0:
                    events.append("belly_contact")
                if self._ceiling_run[0].item() > 0:
                    events.append("ceiling_violation")
                self._video_meta_buffer.append({
                    "idx": self._video_meta_frame_idx,
                    "episode": self._video_episode_id,
                    "phase": phase,
                    "com_z": round(z, 4),
                    "com_x": round(float(pos[0, 0].item()), 4),
                    "h_star": round(h_star, 3),
                    "vel_x": round(float(vel[0, 0].item()), 4),
                    "vel_z": round(float(vel[0, 2].item()), 4),
                    "gz": round(gz, 4),
                    "events": events,
                })
                self._video_meta_frame_idx += 1
        if self._video_meta_was_recording and not is_recording:
            self._write_video_metadata_sidecar()
        self._video_meta_was_recording = is_recording

    def _write_video_metadata_sidecar(self):
        import json as _json
        vw = self._video_wrapper_ref
        if not self._video_meta_buffer or vw is None:
            self._video_meta_buffer = []
            self._video_meta_frame_idx = 0
            return
        meta = {
            "step": vw._recording_start_step,
            "curriculum_stage": int(self._curriculum_stage),
            "total_frames": len(self._video_meta_buffer),
            "num_episodes": len(set(f.get("episode", 0) for f in self._video_meta_buffer)),
            "camera_mode": "arc",
            "fps": vw._actual_fps,
            "frames": self._video_meta_buffer,
        }
        meta_path = os.path.join(vw._out_dir, f"{vw._recording_start_step}.meta.json")
        try:
            with open(meta_path, "w") as f:
                _json.dump(meta, f)
        except OSError:
            pass
        self._video_meta_buffer = []
        self._video_meta_frame_idx = 0
