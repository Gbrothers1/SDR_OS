"""v44.1 — Go2 directed-launch environment: jump ON COMMAND, then recover.

Faithful clone of the validated rl/v44/envs/go2_launch_env.py with the v44.1
design deltas (artifacts/knowledge_base/experiments/v44.1-directed-launch-design.md):

1. 3-dim command (h*, vx*, trigger): trigger is 0 for a random 0.5-2.5 s STAND
   phase, then latches 1. Per-frame obs 48 -> 49, history 5 -> 245 dims.
2. Phase structure: the ENTIRE jump reward machinery is trigger-gated (a
   pre-trigger jump pays nothing and forfeits stand pay); the apex fires only
   during the sanctioned arc (post-trigger liftoff -> first touchdown,
   _flight_closed latch — both lessons from v46.1); stand_track (dense,
   multiplicative) pays in the STAND phase and again post-arc (RECOVER), which
   trains the post-landing recovery the v44.0.10 model lacks.
3. Trained FROM SCRATCH (Ethan: warm start does not work — the prior is the
   in-place-jump attractor).

Episode 6 s. Flat ground, no bar (this is the commandable PRIMITIVE; obstacle
composition happens at the HL/scripted layer).
"""

import os
import torch
import genesis as gs

from genesis_forge import ManagedEnvironment
from genesis_forge.managers import (
    RewardManager, TerminationManager, EntityManager, ObservationManager,
    ActuatorManager, PositionActionManager, VelocityCommandManager, ContactManager,
)
from genesis_forge.mdp import reset, rewards, terminations, observations

from rl.v44.envs.directed_launch_command_manager import DirectedLaunchCommandManager
from rl.v44.envs import directed_launch_rewards as R

HEIGHT_OFFSET = 0.4
INITIAL_BODY_POSITION = [0.0, 0.0, HEIGHT_OFFSET]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]

# 1-D apex curriculum, v44 bands (the gate-rate fix lives in the trainer).
APEX_STAGES = {
    1: {"h_low": 0.38, "h_high": 0.42, "vx_high": 0.5},
    2: {"h_low": 0.40, "h_high": 0.52, "vx_high": 1.0},
    3: {"h_low": 0.40, "h_high": 0.57, "vx_high": 1.5},
}

_FOOT_NAMES = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
_GROUND_CONTACT_FORCE = 5.0
_LANDING_MIN_FEET = 3
_LANDING_SETTLE_WINDOW = 5


class Go2DirectedLaunchEnv(ManagedEnvironment):
    def __init__(self, num_envs: int = 1, dt: float = 1 / 50,
                 max_episode_length_s: float = 6.0, headless: bool = True):
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
        self.camera = self.scene.add_camera(pos=(2.5, 1.5, 1.0), lookat=(1.0, 0.0, 0.4),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self.camera_side = self.scene.add_camera(pos=(1.0, -2.5, 0.5), lookat=(1.0, 0.0, 0.3),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self.camera_top = self.scene.add_camera(pos=(1.0, 0.0, 3.0), lookat=(1.0, 0.0, 0.0),
            res=(480, 360), fov=60, env_idx=0, debug=True, GUI=False)
        self.camera_front = self.scene.add_camera(pos=(3.5, 0.0, 0.6), lookat=(0.0, 0.0, 0.3),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self._video_wrapper_ref = None
        self._video_meta_buffer = []
        self._video_meta_frame_idx = 0
        self._video_meta_was_recording = False
        self._video_episode_id = 0
        self._step_id = 0
        self._foot_links = []

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
        self.velocity_command = VelocityCommandManager(self, range={
            "lin_vel_x": [0.0, 1.5], "lin_vel_y": [0.0, 0.0], "ang_vel_z": [0.0, 0.0]},
            standing_probability=0.0, resample_time_sec=4.0)
        self.launch_command = DirectedLaunchCommandManager(self, device=gs.device)
        self.launch_command.allocate()  # before ObservationManager.build()

        self._alloc_launch_state()

        # v44.0.10 weights + stand_track (design budget: standing through the
        # trigger earns 0 for the window; jumping earns ~44 + recovered stand pay).
        self.reward_manager = RewardManager(self, logging_enabled=True, cfg={
            "liftoff":           {"weight": 3.0,   "fn": self._r_liftoff},
            "crouch_load":       {"weight": 1.5,   "fn": self._r_crouch_load},
            "liftoff_progress":  {"weight": 10.0,  "fn": self._r_liftoff_progress},
            "apex_match":        {"weight": 10.0,  "fn": self._r_apex_match},
            "ascent_shaping":    {"weight": 2.5,   "fn": self._r_ascent_shaping},
            "forward_vel_track": {"weight": 1.5,   "fn": self._r_forward_vel_track},
            "posture_flight":    {"weight": 2.0,   "fn": self._r_posture_flight},
            "landing":           {"weight": 8.0,   "fn": self._r_landing},
            "grounded_posture":  {"weight": 2.0,   "fn": self._r_grounded_posture},
            "action_rate":       {"weight": -0.01, "fn": rewards.action_rate_l2},
            "alive":             {"weight": 0.1,   "fn": rewards.is_alive},
            # v44.1: dense stand reward for STAND + RECOVER phases.
            "stand_track":       {"weight": 1.0,   "fn": self._r_stand_track},
        })
        self.termination_manager = TerminationManager(self, logging_enabled=True, term_cfg={
            "timeout": {"fn": terminations.timeout, "time_out": True},
            "fall_over": {"fn": terminations.bad_orientation,
                          "params": {"limit_angle": 50.0, "entity_manager": self.robot_manager}},
        })
        ObservationManager(self, name="policy", history_len=5, cfg={
            "command": {"fn": self.launch_command.observation},                       # 3 (v44.1)
            "velocity_cmd": {"fn": self.velocity_command.observation},                # 3
            "angle_velocity": {"fn": lambda env: self.robot_manager.get_angular_velocity()},   # 3
            "projected_gravity": {"fn": lambda env: self.robot_manager.get_projected_gravity()}, # 3
            "dof_position": {"fn": lambda env: self.action_manager.get_dofs_position()},  # 12
            "dof_velocity": {"fn": lambda env: self.action_manager.get_dofs_velocity(), "scale": 0.05}, # 12
            "actions": {"fn": lambda env: self.action_manager.get_actions()},            # 12
            "base_height": {"fn": self._obs_base_height},                              # 1
        })
        ObservationManager(self, name="critic", history_len=5, cfg={
            "foot_contact_force": {"fn": observations.contact_force,
                                   "params": {"contact_manager": self.foot_contact_manager}},
            "dof_force": {"fn": observations.entity_dofs_force,
                          "params": {"action_manager": self.action_manager}, "scale": 0.1},
            "base_lin_vel": {"fn": lambda env: self.robot_manager.get_linear_velocity()},
            "base_height_peak": {"fn": self._obs_base_height_peak},
        })

    def _alloc_launch_state(self):
        n, dev = self.num_envs, gs.device
        self._all_feet_off = torch.zeros(n, dtype=torch.bool, device=dev)
        self._airborne = torch.zeros(n, dtype=torch.bool, device=dev)
        self._peak_z = torch.zeros(n, device=dev)
        self._z_apex_at_fire = torch.zeros(n, device=dev)
        self._apex_fired = torch.zeros(n, dtype=torch.bool, device=dev)
        self._apex_credited = torch.zeros(n, dtype=torch.bool, device=dev)
        self._liftoff_seen = torch.zeros(n, dtype=torch.bool, device=dev)
        self._liftoff_credited = torch.zeros(n, dtype=torch.bool, device=dev)
        self._landing_credited = torch.zeros(n, dtype=torch.bool, device=dev)
        self._just_landed = torch.zeros(n, dtype=torch.bool, device=dev)
        self._prev_airborne = torch.zeros(n, dtype=torch.bool, device=dev)
        self._prev_vz = torch.zeros(n, device=dev)
        self._airborne_settle_timer = torch.zeros(n, dtype=torch.long, device=dev)
        self._running_peak_z = torch.full((n,), R._LIFTOFF_FLOOR_Z, device=dev)
        self._peak_baseline_set = torch.zeros(n, dtype=torch.bool, device=dev)
        self._grounded_run = torch.zeros(n, dtype=torch.long, device=dev)
        self._liftoff_progress_buf = torch.zeros(n, device=dev)
        self._crouch_credited = torch.zeros(n, dtype=torch.bool, device=dev)
        self._stood_since_baseline = torch.zeros(n, dtype=torch.bool, device=dev)
        # ── v44.1 phase state ──
        self._trigger_liftoff_seen = torch.zeros(n, dtype=torch.bool, device=dev)
        self._flight_closed = torch.zeros(n, dtype=torch.bool, device=dev)

    # ── curriculum ────────────────────────────────────────────────────────────
    def set_curriculum_stage(self, stage: int):
        self._curriculum_stage = stage
        if hasattr(self, "launch_command") and self.launch_command.command is not None:
            s = APEX_STAGES[stage]
            self.launch_command.set_curriculum_range(s["h_low"], s["h_high"], s["vx_high"])

    def get_stage_gate_metric(self, name: str) -> float:
        return self.reward_manager.last_episode_mean_reward(name)

    # ── build / reset / step ─────────────────────────────────────────────────
    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)
        for name in _FOOT_NAMES:
            self._foot_links.append(self.robot.get_link(name))
        self.set_curriculum_stage(self._curriculum_stage)
        print(f"[v44.1] directed launch stage={self._curriculum_stage}", flush=True)

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        if envs_idx is None:
            envs_idx = torch.arange(self.num_envs, device=gs.device)
        self.launch_command.reset(envs_idx)
        for t in (self._all_feet_off, self._airborne, self._apex_fired,
                  self._apex_credited, self._liftoff_seen, self._liftoff_credited,
                  self._landing_credited, self._just_landed, self._prev_airborne,
                  self._trigger_liftoff_seen, self._flight_closed):
            t[envs_idx] = False
        for t in (self._peak_z, self._z_apex_at_fire, self._prev_vz):
            t[envs_idx] = 0.0
        self._airborne_settle_timer[envs_idx] = 0
        self._running_peak_z[envs_idx] = R._LIFTOFF_FLOOR_Z
        self._peak_baseline_set[envs_idx] = False
        self._grounded_run[envs_idx] = 0
        self._liftoff_progress_buf[envs_idx] = 0.0
        self._crouch_credited[envs_idx] = False
        self._stood_since_baseline[envs_idx] = False
        return result

    def step(self, actions):
        # DirectedLaunchCommandManager is a plain object (not a genesis_forge
        # registered manager), so its per-step trigger latch must be called
        # explicitly — without this the trigger NEVER fires and the policy
        # learns to stand forever (caught in the first v44.1 smoke: stand_track
        # at 0.99 of max, zero jumps).
        self.launch_command.step()
        self._update_launch_state_pre()
        obs, rew, terminated, truncated, infos = super().step(actions)
        dones = terminated | truncated
        self._step_id += 1
        if dones is not None and dones.numel() > 0 and dones[0].item():
            self._video_episode_id += 1
        self._buffer_video_metadata()
        return obs, rew, terminated, truncated, infos

    @property
    def _triggered(self):
        return self.launch_command.triggered

    def _update_launch_state_pre(self):
        pos = self.robot.get_pos()
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        z = pos[:, 2]
        vz = vel[:, 2]
        _AIRBORNE_MIN_Z = 0.28
        foot_down = self._feet_in_contact()
        all_off = ~foot_down.any(dim=-1) & (z > _AIRBORNE_MIN_Z)
        self._all_feet_off = all_off
        self._airborne = all_off
        self._airborne_settle_timer = torch.where(
            self._airborne,
            torch.full_like(self._airborne_settle_timer, _LANDING_SETTLE_WINDOW),
            torch.clamp(self._airborne_settle_timer - 1, min=0),
        )
        liftoff_edge = all_off & (vz > 0.0)
        self._liftoff_seen |= liftoff_edge
        # v44.1: only a POST-TRIGGER liftoff opens the sanctioned arc.
        self._trigger_liftoff_seen |= liftoff_edge & self._triggered
        self._peak_z = torch.maximum(self._peak_z, torch.where(all_off, z, self._peak_z))
        grounded = foot_down.any(dim=-1)
        self._grounded_run = torch.where(
            grounded, self._grounded_run + 1, torch.zeros_like(self._grounded_run))
        settle = grounded & (~self._peak_baseline_set) & (
            (z <= R._LIFTOFF_FLOOR_Z) | (self._grounded_run >= R._PROGRESS_BASELINE_GROUNDED_STEPS))
        self._peak_baseline_set = self._peak_baseline_set | settle
        self._liftoff_progress_buf, self._running_peak_z = R.liftoff_progress(
            z, self._running_peak_z, self._peak_baseline_set)
        # v44.1: height records pay only post-trigger (pre-trigger hops earn 0).
        self._liftoff_progress_buf = self._liftoff_progress_buf * self._triggered.to(
            self._liftoff_progress_buf.dtype)
        self._stood_since_baseline |= self._peak_baseline_set & grounded & (z >= 0.30)
        # apex one-shot: only during the sanctioned arc (v46.1 stale-latch lesson).
        crossed_apex = (self._airborne & (self._prev_vz > 0.0) & (vz <= 0.0)
                        & (~self._apex_fired)
                        & self._trigger_liftoff_seen & (~self._flight_closed))
        self._z_apex_at_fire = torch.where(crossed_apex, z, self._z_apex_at_fire)
        self._apex_fired |= crossed_apex
        recently_airborne = self._airborne_settle_timer > 0
        feet_down_count = foot_down.sum(dim=-1)
        self._just_landed = recently_airborne & (feet_down_count >= _LANDING_MIN_FEET)
        # Close the arc at the first touchdown after the commanded liftoff.
        self._flight_closed |= self._trigger_liftoff_seen & self._just_landed & (~self._airborne)
        self._prev_airborne = self._airborne.clone()
        self._prev_vz = vz.clone()

    def _feet_in_contact(self) -> torch.Tensor:
        if self.foot_contact_manager.contacts is None:
            return torch.ones(self.num_envs, 4, dtype=torch.bool, device=gs.device)
        force_norm = torch.norm(self.foot_contact_manager.contacts, dim=-1)
        return force_norm > _GROUND_CONTACT_FORCE

    # ── reward methods ─────────────────────────────────────────────────────────
    def _vz(self):
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        return vel[:, 2]

    def _speed_sq(self):
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        return (vel ** 2).sum(dim=-1)

    def _r_liftoff(self, env):
        # v44.1: the rewarded liftoff is the COMMANDED one.
        newly = (self._all_feet_off & (self._vz() > 0.0) & self._triggered
                 & (~self._liftoff_credited))
        self._liftoff_credited |= newly
        return newly.to(torch.float)

    def _r_apex_match(self, env):
        newly = self._apex_fired & (~self._apex_credited)
        self._apex_credited |= newly
        return R.apex_match(self._z_apex_at_fire, self.launch_command.command[:, 0],
                            k=R.K_APEX, fired=newly)

    def _flight_active(self):
        return self._trigger_liftoff_seen & (~self._flight_closed)

    def _r_ascent_shaping(self, env):
        pos = self.robot.get_pos()
        below = pos[:, 2] < self.launch_command.command[:, 0]
        return R.ascent_shaping(self._vz(), self._airborne, below,
                                flight_active=self._flight_active())

    def _r_forward_vel_track(self, env):
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        vx = vel[:, 0]
        return R.forward_vel_track(vx, self.launch_command.command[:, 1],
                                   self._airborne, flight_active=self._flight_active())

    def _r_posture_flight(self, env):
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        asym = self._foot_lateral_asymmetry()
        return R.posture_flight(gz, asym, self._airborne)

    def _foot_lateral_asymmetry(self):
        foot_down = self._feet_in_contact().to(torch.float)
        left = foot_down[:, 0] + foot_down[:, 2]
        right = foot_down[:, 1] + foot_down[:, 3]
        return torch.abs(left - right)

    def _r_landing(self, env):
        foot_down = self._feet_in_contact()
        feet_down_count = foot_down.sum(dim=-1)
        landed = (feet_down_count >= _LANDING_MIN_FEET) & (self._airborne_settle_timer > 0)
        pitch = torch.abs(self.robot_manager.get_projected_gravity()[:, 0])
        z = self.robot.get_pos()[:, 2]
        # Stick the COMMANDED jump (arc landing), once.
        earned_landing = landed & self._trigger_liftoff_seen & (~self._landing_credited)
        self._landing_credited |= earned_landing
        return R.landing(landed, pitch, z, earned_landing)

    def _r_grounded_posture(self, env):
        z = self.robot.get_pos()[:, 2]
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        return R.grounded_posture(z, gz, self._airborne)

    def _r_crouch_load(self, env):
        z = self.robot.get_pos()[:, 2]
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        grounded = self._feet_in_contact().any(dim=-1)
        in_band = (z >= R._CROUCH_FLOOR_Z) & (z <= R._CROUCH_TRIGGER_Z)
        # v44.1: the rewarded load crouch is the one AFTER the trigger.
        fired = (grounded & self._stood_since_baseline & (gz <= -0.9) & in_band
                 & self._triggered
                 & (self._vz() > R._CROUCH_SPRING_VZ) & (~self._trigger_liftoff_seen)
                 & (~self._crouch_credited))
        self._crouch_credited |= fired
        return R.crouch_load(z, fired)

    def _r_liftoff_progress(self, env):
        return self._liftoff_progress_buf

    def _r_stand_track(self, env):
        """Dense stand reward: STAND phase (pre-trigger) and RECOVER (post-arc).
        Gated OFF during the launch window so standing through the trigger
        forfeits everything until the jump happens."""
        z = self.robot.get_pos()[:, 2]
        active = (~self._triggered) | self._flight_closed
        return R.stand_track(z, self._speed_sq(), active)

    # ── observations ──────────────────────────────────────────────────────────
    def _obs_base_height(self, env):
        return self.robot.get_pos()[:, 2:3]

    def _obs_base_height_peak(self, env):
        return self._peak_z.unsqueeze(-1)

    # ── video metadata sidecar ─────────────────────────────────────────────────
    def _flatten_vec(self, vec: torch.Tensor) -> torch.Tensor:
        if vec.ndim > 2:
            return vec.view(vec.shape[0], -1)
        return vec

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
                phase = "airborne" if self._airborne[0].item() else (
                    "launch_window" if (self._triggered[0].item()
                                        and not self._flight_closed[0].item())
                    else "stand")
                events = []
                if self._all_feet_off[0].item() and not self._prev_airborne[0].item():
                    events.append("launch")
                if self._just_landed[0].item():
                    events.append("landing")
                if self._triggered[0].item() and not self._flight_closed[0].item():
                    events.append("trigger_active")
                vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
                proj_grav = self._flatten_vec(self.robot_manager.get_projected_gravity())
                self._video_meta_buffer.append({
                    "idx": self._video_meta_frame_idx,
                    "episode": self._video_episode_id,
                    "phase": phase,
                    "com_z": round(float(pos[0, 2].item()), 4),
                    "com_x": round(float(pos[0, 0].item()), 4),
                    "triggered": bool(self._triggered[0].item()),
                    "vel_x": round(float(vel[0, 0].item()), 4),
                    "vel_z": round(float(vel[0, 2].item()), 4),
                    "gz": round(float(proj_grav[0, 2].item()), 4),
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
        step = vw._recording_start_step
        episode_ids = sorted(set(f.get("episode", 0) for f in self._video_meta_buffer))
        meta = {
            "step": step,
            "curriculum_stage": int(self._curriculum_stage),
            "total_frames": len(self._video_meta_buffer),
            "num_episodes": len(episode_ids),
            "episode_ids": episode_ids,
            "camera_mode": "arc",
            "fps": vw._actual_fps,
            "frames": self._video_meta_buffer,
        }
        meta_path = os.path.join(vw._out_dir, f"{step}.meta.json")
        try:
            with open(meta_path, "w") as f:
                _json.dump(meta, f)
        except OSError:
            pass
        self._video_meta_buffer = []
        self._video_meta_frame_idx = 0
