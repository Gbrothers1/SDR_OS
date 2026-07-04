"""v46.0 — Go2 LL-Hurdle environment: jump over a PHYSICAL bar.

Faithful clone of the validated rl/v44/envs/go2_launch_env.py (doctrine rule 4)
with the surgical additions from the v46.0 design doc:

1. A physical bar (collision=True cylinder) across the lane at BAR_X. v43's bar
   was visualization-only and its analytical "clearance detection" was gamed for
   23 sub-versions; with physics on, a below-top crossing is impossible and
   perching on a 3 cm cylinder is useless.
2. A 2-dim bar observation (bar_dx, bar_top) appended to the policy obs
   (48 -> 50 dims/frame, 250 total) so the policy can time the jump.
3. bar_clearance one-shot (hard-gated airborne crossing) + approach_progress
   telescoping forward-record reward.
4. HurdleCommandManager: h* = bar_top + margin (feasibility-clamped), so the
   commanded apex is always consistent with the obstacle.

Spec: artifacts/knowledge_base/experiments/v46.0-ll-hurdle-design.md.
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

from rl.v46.envs.hurdle_command_manager import HurdleCommandManager, BAR_STAGES
from rl.v46.envs import hurdle_rewards as R

HEIGHT_OFFSET = 0.4
# Run-up spawn: behind the bar with randomized distance (the bar obs makes the
# varying distance learnable; fixed distance would invite open-loop timing).
SPAWN_X_LOW, SPAWN_X_HIGH = -1.4, -1.0
INITIAL_BODY_POSITION = [-1.2, 0.0, HEIGHT_OFFSET]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]

BAR_X = 0.0           # bar plane (world x)
BAR_RADIUS = 0.03     # m — physical cylinder
BAR_LANE_HALF = 1.0   # m — lane half-width
_BAR_BUILD_TOP = BAR_STAGES[1]["bar_top"]  # built at stage-1 height; staged via set_pos

# v46.1 — TAKEOFF-ZONE GATING (Ethan's smoke visual: warm-started policy
# twirl-jumped in clockwise circles at spawn forever; the v44 jump one-shots
# pay ~40/episode for jumping ANYWHERE vs ~3 for approaching, so in-place
# jumping was the optimum). Structural fix per doctrine (gates, not penalties):
# the entire jump reward machinery (liftoff, crouch, telescoping progress,
# apex, flight shaping) pays ONLY when it happens within TAKEOFF_ZONE of the
# bar, and the landing one-shot requires the bar to have been CROSSED. Far
# from the bar, approach_progress is the only gradient.
TAKEOFF_ZONE = 0.6    # m before the bar where the jump machinery is armed
# Flight yaw-spin penalty: a spinning jump drifts off-heading and cannot cross
# reliably. Tolerate gentle rotation; penalize genuine spins. (The twirl
# aesthetic lives on in the preserved v44.0.10 model — hurdling needs heading.)
_YAW_SPIN_TOLERANCE = 1.5   # rad/s

# Foot order: FL=0, FR=1, RL=2, RR=3 (matches v44 convention)
_FOOT_NAMES = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
_GROUND_CONTACT_FORCE = 5.0   # N (v44.0.2 calibration)

_LANDING_MIN_FEET = 3        # v44.0.5 relaxed landing detection
_LANDING_SETTLE_WINDOW = 5   # steps


class Go2HurdleEnv(ManagedEnvironment):
    def __init__(self, num_envs: int = 1, dt: float = 1 / 50,
                 max_episode_length_s: float = 5.0, headless: bool = True):
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
        # PHYSICAL bar (v43 lesson: collision must be real, not analytical).
        self.hurdle_bar = self.scene.add_entity(gs.morphs.Cylinder(
            radius=BAR_RADIUS, height=BAR_LANE_HALF * 2,
            pos=(BAR_X, 0.0, _BAR_BUILD_TOP - BAR_RADIUS),
            euler=(90.0, 0.0, 0.0), collision=True, fixed=True))
        # Posts: visualization only (jump corridor cue on video).
        self.hurdle_post_l = self.scene.add_entity(gs.morphs.Box(
            size=(0.04, 0.04, 0.5), pos=(BAR_X, -BAR_LANE_HALF, 0.25),
            visualization=True, collision=False, fixed=True))
        self.hurdle_post_r = self.scene.add_entity(gs.morphs.Box(
            size=(0.04, 0.04, 0.5), pos=(BAR_X, BAR_LANE_HALF, 0.25),
            visualization=True, collision=False, fixed=True))
        # 4-cam recording (v44 layout, retargeted at the bar).
        self.camera = self.scene.add_camera(pos=(1.5, 1.5, 1.0), lookat=(0.0, 0.0, 0.4),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self.camera_side = self.scene.add_camera(pos=(0.0, -2.5, 0.5), lookat=(0.0, 0.0, 0.3),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self.camera_top = self.scene.add_camera(pos=(0.0, 0.0, 3.0), lookat=(0.0, 0.0, 0.0),
            res=(480, 360), fov=60, env_idx=0, debug=True, GUI=False)
        self.camera_front = self.scene.add_camera(pos=(2.5, 0.0, 0.6), lookat=(-0.8, 0.0, 0.3),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        # Sidecar plumbing (set by trainer if a VideoWrapper is attached).
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
        self.hurdle_command = HurdleCommandManager(self, device=gs.device)
        self.hurdle_command.allocate()  # must be before ObservationManager.build()

        self._alloc_launch_state()

        # Reward budget (design doc): v44.0.10 set unchanged + two additions.
        # bar_clearance (15.0, one-shot) structurally dominates every dense term;
        # approach_progress telescopes to ~run-up-length x 2 per episode.
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
            # v46.0 additions:
            "bar_clearance":     {"weight": 15.0,  "fn": self._r_bar_clearance},
            # v46.1: 2.0 -> 3.0 — with the jump machinery zone-gated, this is
            # the ONLY gradient far from the bar; it must out-pay idling.
            "approach_progress": {"weight": 3.0,   "fn": self._r_approach_progress},
            # v46.1: keep flight directed (negative fn, see _r_flight_yaw).
            "flight_yaw":        {"weight": 0.2,   "fn": self._r_flight_yaw},
            # v46.2: dense run-up driver. The v46.1 smoke showed telescoping
            # approach alone doesn't teach walking from scratch (no per-step
            # gradient once paused; robot stands at spawn). This is the
            # v45-crawl-validated locomotion pattern: exp velocity tracking,
            # but gated GROUNDED & PRE-BAR only and weighted so the jump bundle
            # still dominates at the bar (doctrine rule 3 budget: waiting at
            # the bar ~0.16/step ≈ 24 over a remaining episode vs jump+cross
            # ~59 one-time; running around the bar end forfeits clearance+
            # landing and pays nothing past the bar plane).
            "ground_vel_track":  {"weight": 0.5,   "fn": self._r_ground_vel_track},
        })
        self.termination_manager = TerminationManager(self, logging_enabled=True, term_cfg={
            "timeout": {"fn": terminations.timeout, "time_out": True},
            "fall_over": {"fn": terminations.bad_orientation,
                          "params": {"limit_angle": 50.0, "entity_manager": self.robot_manager}},
        })
        ObservationManager(self, name="policy", history_len=5, cfg={
            "command": {"fn": self.hurdle_command.observation},                       # 2
            "velocity_cmd": {"fn": self.velocity_command.observation},                # 3
            "angle_velocity": {"fn": lambda env: self.robot_manager.get_angular_velocity()},   # 3
            "projected_gravity": {"fn": lambda env: self.robot_manager.get_projected_gravity()}, # 3
            "dof_position": {"fn": lambda env: self.action_manager.get_dofs_position()},  # 12
            "dof_velocity": {"fn": lambda env: self.action_manager.get_dofs_velocity(), "scale": 0.05}, # 12
            "actions": {"fn": lambda env: self.action_manager.get_actions()},            # 12
            "base_height": {"fn": self._obs_base_height},                              # 1
            # v46.0: appended LAST so warm-start surgery maps the first 48 cols 1:1.
            # Scaled so bar_dx (up to ~1.4 m at spawn) lands in ~[0, 1] like the
            # other inputs (red-team minor #5: unscaled would need ~14x first-layer
            # weights on the bar columns, slowing their gradient learning).
            "bar_obs": {"fn": self._obs_bar, "scale": 1.0 / 1.5},                     # 2
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
        # ── v44 launch-state machinery (verbatim semantics) ──
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
        # ── v46.0 bar/approach state ──
        self._prev_x = torch.full((n,), INITIAL_BODY_POSITION[0], device=dev)
        self._x_peak = torch.full((n,), INITIAL_BODY_POSITION[0], device=dev)
        self._approach_progress_buf = torch.zeros(n, device=dev)
        self._bar_cross_fired = torch.zeros(n, dtype=torch.bool, device=dev)   # edge this step
        self._bar_credited = torch.zeros(n, dtype=torch.bool, device=dev)      # one-shot latch
        self._z_at_crossing = torch.zeros(n, device=dev)                        # telemetry
        # ── v46.1 takeoff-zone state ──
        self._in_zone = torch.zeros(n, dtype=torch.bool, device=dev)            # x within zone now
        self._zone_liftoff_seen = torch.zeros(n, dtype=torch.bool, device=dev)  # liftoff FROM the zone
        # Closes the flight window at the first touchdown after a zone liftoff
        # regardless of crossing success — with the landing one-shot now gated
        # on _bar_credited, the v44 closure (landing credit) no longer fires on
        # failed crossings, and without this latch post-landing bounces would
        # farm ascent/forward_vel_track forever.
        self._flight_closed = torch.zeros(n, dtype=torch.bool, device=dev)

    # ── curriculum ────────────────────────────────────────────────────────────
    def set_curriculum_stage(self, stage: int):
        self._curriculum_stage = stage
        if hasattr(self, "hurdle_command") and self.hurdle_command.command is not None:
            self.hurdle_command.set_curriculum_stage(stage)
        # Reposition the physical bar so exactly `bar_top` is the top of the
        # cylinder (smoke gate G-BAR verifies set_pos works on a fixed collider).
        if hasattr(self, "hurdle_bar") and self.scene.is_built:
            bar_top = BAR_STAGES[stage]["bar_top"]
            self.hurdle_bar.set_pos(torch.tensor(
                [[BAR_X, 0.0, bar_top - BAR_RADIUS]], device=gs.device))

    def get_stage_gate_metric(self, name: str) -> float:
        return self.reward_manager.last_episode_mean_reward(name)

    @property
    def bar_top(self) -> float:
        return BAR_STAGES[self._curriculum_stage]["bar_top"]

    # ── build / reset / step ─────────────────────────────────────────────────
    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)
        for name in _FOOT_NAMES:
            self._foot_links.append(self.robot.get_link(name))
        self.set_curriculum_stage(self._curriculum_stage)
        print(f"[v46.0] LL-Hurdle stage={self._curriculum_stage} "
              f"bar_top={self.bar_top}", flush=True)

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        # ManagedEnvironment.reset() returns (obs, extras) — unpack defensively
        # (v44 parity; RslRlWrapper expects the tuple passed through).
        if envs_idx is None:
            envs_idx = torch.arange(self.num_envs, device=gs.device)
        self.hurdle_command.reset(envs_idx)
        for t in (self._all_feet_off, self._airborne, self._apex_fired,
                  self._apex_credited, self._liftoff_seen, self._liftoff_credited,
                  self._landing_credited, self._just_landed, self._prev_airborne,
                  self._bar_cross_fired, self._bar_credited,
                  self._in_zone, self._zone_liftoff_seen, self._flight_closed):
            t[envs_idx] = False
        for t in (self._peak_z, self._z_apex_at_fire, self._prev_vz,
                  self._approach_progress_buf, self._z_at_crossing):
            t[envs_idx] = 0.0
        self._airborne_settle_timer[envs_idx] = 0
        self._running_peak_z[envs_idx] = R._LIFTOFF_FLOOR_Z
        self._peak_baseline_set[envs_idx] = False
        self._grounded_run[envs_idx] = 0
        self._liftoff_progress_buf[envs_idx] = 0.0
        self._crouch_credited[envs_idx] = False
        self._stood_since_baseline[envs_idx] = False

        # v46.0: randomized run-up spawn x (bar obs makes the varying distance
        # learnable; a fixed distance would invite open-loop jump timing).
        n_reset = len(envs_idx)
        if n_reset > 0:
            pos = self.robot.get_pos()[envs_idx].clone()
            spawn_x = torch.empty(n_reset, device=gs.device).uniform_(SPAWN_X_LOW, SPAWN_X_HIGH)
            pos[:, 0] = spawn_x
            pos[:, 1] = 0.0
            self.robot.set_pos(pos, envs_idx=envs_idx)
            self._prev_x[envs_idx] = spawn_x
            self._x_peak[envs_idx] = spawn_x
        return result

    def step(self, actions):
        self._update_launch_state_pre()
        obs, rew, terminated, truncated, infos = super().step(actions)
        dones = terminated | truncated
        self._step_id += 1
        if dones is not None and dones.numel() > 0 and dones[0].item():
            self._video_episode_id += 1
        self._buffer_video_metadata()
        return obs, rew, terminated, truncated, infos

    def _update_launch_state_pre(self):
        pos = self.robot.get_pos()
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        x = pos[:, 0]
        z = pos[:, 2]
        vz = vel[:, 2]
        # v44 airborne detection with the belly-splay height guard.
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
        # v46.1: zone membership BEFORE the liftoff edge so a liftoff initiated
        # inside the zone is attributed to it (x at the edge step is still pre-bar).
        self._in_zone = (x >= BAR_X - TAKEOFF_ZONE) & (x <= BAR_X)
        liftoff_edge = all_off & (vz > 0.0)
        self._liftoff_seen |= liftoff_edge
        self._zone_liftoff_seen |= liftoff_edge & self._in_zone
        self._peak_z = torch.maximum(self._peak_z, torch.where(all_off, z, self._peak_z))
        # v44.0.7 telescoping liftoff-progress with robust baseline.
        grounded = foot_down.any(dim=-1)
        self._grounded_run = torch.where(
            grounded, self._grounded_run + 1, torch.zeros_like(self._grounded_run))
        settle = grounded & (~self._peak_baseline_set) & (
            (z <= R._LIFTOFF_FLOOR_Z) | (self._grounded_run >= R._PROGRESS_BASELINE_GROUNDED_STEPS))
        self._peak_baseline_set = self._peak_baseline_set | settle
        self._liftoff_progress_buf, self._running_peak_z = R.liftoff_progress(
            z, self._running_peak_z, self._peak_baseline_set)
        # v46.1: height records only pay when set from/over the takeoff zone —
        # in-place jumping at spawn earns nothing from the telescoping term.
        self._liftoff_progress_buf = self._liftoff_progress_buf * (
            self._in_zone | self._zone_liftoff_seen).to(self._liftoff_progress_buf.dtype)
        self._stood_since_baseline |= self._peak_baseline_set & grounded & (z >= 0.30)
        # apex one-shot edge — v46.1: only during the sanctioned arc (a zone
        # liftoff that has not yet touched down). Without this, an approach
        # jump's apex would latch _apex_fired/_z_apex_at_fire pre-zone and get
        # stale-credited once the zone liftoff happens.
        crossed_apex = (self._airborne & (self._prev_vz > 0.0) & (vz <= 0.0)
                        & (~self._apex_fired)
                        & self._zone_liftoff_seen & (~self._flight_closed))
        self._z_apex_at_fire = torch.where(crossed_apex, z, self._z_apex_at_fire)
        self._apex_fired |= crossed_apex
        # v44.0.5 landing edge.
        recently_airborne = self._airborne_settle_timer > 0
        feet_down_count = foot_down.sum(dim=-1)
        self._just_landed = recently_airborne & (feet_down_count >= _LANDING_MIN_FEET)
        # v46.1: close the sanctioned flight window at the first touchdown after
        # a zone liftoff (crossing or not) — kills the failed-crossing bounce farm.
        self._flight_closed |= self._zone_liftoff_seen & self._just_landed & (~self._airborne)
        self._prev_airborne = self._airborne.clone()
        self._prev_vz = vz.clone()

        # ── v46.0: bar crossing one-shot + telescoping approach record ──
        bar_top = self.bar_top
        crossing_edge = (self._prev_x < BAR_X) & (x >= BAR_X)
        self._bar_cross_fired = (
            crossing_edge & self._airborne
            & (z > bar_top + R.BAR_CLEARANCE_MARGIN)
            & (~self._bar_credited)
        )
        self._z_at_crossing = torch.where(crossing_edge, z, self._z_at_crossing)
        self._approach_progress_buf, self._x_peak = R.approach_progress(x, self._x_peak)
        self._prev_x = x.clone()

    def _feet_in_contact(self) -> torch.Tensor:
        """(num_envs, 4) bool — per-foot contact via force magnitude (v44 verbatim)."""
        if self.foot_contact_manager.contacts is None:
            return torch.ones(self.num_envs, 4, dtype=torch.bool, device=gs.device)
        force_norm = torch.norm(self.foot_contact_manager.contacts, dim=-1)
        return force_norm > _GROUND_CONTACT_FORCE

    # ── reward methods (thin wrappers into hurdle_rewards) ────────────────────
    def _vz(self):
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        return vel[:, 2]

    def _r_liftoff(self, env):
        # v46.1: only a liftoff FROM the takeoff zone is the rewarded behavior.
        newly = (self._all_feet_off & (self._vz() > 0.0) & self._in_zone
                 & (~self._liftoff_credited))
        self._liftoff_credited |= newly
        return newly.to(torch.float)

    def _r_apex_match(self, env):
        # v46.1: zone gating lives in the apex FIRING condition (pre-physics);
        # _apex_fired can only latch during the sanctioned arc.
        newly = self._apex_fired & (~self._apex_credited)
        self._apex_credited |= newly
        return R.apex_match(self._z_apex_at_fire, self.hurdle_command.command[:, 0],
                            k=R.K_APEX, fired=newly)

    def _flight_active(self):
        # v46.1: the sanctioned launch arc runs from the ZONE liftoff to the
        # first touchdown after it (crossing or not).
        return self._zone_liftoff_seen & (~self._flight_closed)

    def _r_ascent_shaping(self, env):
        pos = self.robot.get_pos()
        below = pos[:, 2] < self.hurdle_command.command[:, 0]
        return R.ascent_shaping(self._vz(), self._airborne, below,
                                flight_active=self._flight_active())

    def _r_forward_vel_track(self, env):
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        vx = vel[:, 0]
        return R.forward_vel_track(vx, self.hurdle_command.command[:, 1],
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
        # v46.1: the landing one-shot requires the bar to have been CROSSED —
        # landing is "stick it on the far side," not "touch down anywhere."
        earned_landing = landed & self._bar_credited & (~self._landing_credited)
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
        # v46.1: the rewarded load crouch is the one AT the bar (in the zone).
        fired = (grounded & self._stood_since_baseline & (gz <= -0.9) & in_band
                 & self._in_zone
                 & (self._vz() > R._CROUCH_SPRING_VZ) & (~self._zone_liftoff_seen)
                 & (~self._crouch_credited))
        self._crouch_credited |= fired
        return R.crouch_load(z, fired)

    def _r_liftoff_progress(self, env):
        return self._liftoff_progress_buf

    def _r_ground_vel_track(self, env):
        """v46.2: dense forward-velocity tracking during the grounded run-up.

        exp(-K_VX (vx - vx*)^2), only while grounded AND before the bar plane.
        Stops paying past the bar (no run-past or run-around farming) and in
        the air (flight pay belongs to the arc terms)."""
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        vx = vel[:, 0]
        x = self.robot.get_pos()[:, 0]
        r = torch.exp(-R.K_VX * (vx - self.hurdle_command.command[:, 1]) ** 2)
        gate = (~self._airborne) & (x < BAR_X)
        return r * gate.to(r.dtype)

    def _r_flight_yaw(self, env):
        """v46.1: negative shaping for yaw spin during flight (signed fn, +weight).

        A spinning jump (the warm-start twirl) drifts off-heading and cannot
        cross the bar reliably. Gentle rotation under _YAW_SPIN_TOLERANCE is
        free; only genuine spins pay a penalty, and only while airborne."""
        wz = self.robot_manager.get_angular_velocity()[:, 2]
        spin = torch.clamp(torch.abs(wz) - _YAW_SPIN_TOLERANCE, min=0.0)
        return -spin * self._airborne.to(spin.dtype)

    # ── v46.0 reward methods ──────────────────────────────────────────────────
    def _r_bar_clearance(self, env):
        # Edge computed pre-physics in _update_launch_state_pre; latch here so
        # the one-shot credits exactly once per episode.
        fired = self._bar_cross_fired
        self._bar_credited |= fired
        return R.bar_clearance(fired)

    def _r_approach_progress(self, env):
        return self._approach_progress_buf

    # ── observations ──────────────────────────────────────────────────────────
    def _obs_base_height(self, env):
        return self.robot.get_pos()[:, 2:3]

    def _obs_base_height_peak(self, env):
        return self._peak_z.unsqueeze(-1)

    def _obs_bar(self, env):
        """(num_envs, 2): [bar_x - base_x, bar_top]. Analytic, noise-free (sim)."""
        x = self.robot.get_pos()[:, 0:1]
        dx = BAR_X - x
        bar_top = torch.full_like(dx, self.bar_top)
        return torch.cat([dx, bar_top], dim=-1)

    # ── video metadata sidecar (v44 pattern + bar events) ─────────────────────
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
                com_x = float(pos[0, 0].item())
                com_z = float(pos[0, 2].item())
                phase = "airborne" if self._airborne[0].item() else "ground"
                events = []
                if self._all_feet_off[0].item() and not self._prev_airborne[0].item():
                    events.append("launch")
                if self._just_landed[0].item():
                    events.append("landing")
                if self._bar_cross_fired[0].item():
                    events.append("bar_clearance")
                vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
                proj_grav = self._flatten_vec(self.robot_manager.get_projected_gravity())
                self._video_meta_buffer.append({
                    "idx": self._video_meta_frame_idx,
                    "episode": self._video_episode_id,
                    "phase": phase,
                    "com_z": round(com_z, 4),
                    "com_x": round(com_x, 4),
                    "bar_top": round(self.bar_top, 3),
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
            "bar_top": self.bar_top,
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
