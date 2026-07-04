"""v44.0 — Go2 ballistic-launch primitive environment (flat ground, NO bar).

Decoupled from the bar/hurdle task entirely. The robot is commanded a target
apex height h* and forward speed vx* (LaunchCommandManager) and rewarded for a
genuine ballistic launch to that apex with doglike posture and a stable landing
(rl/v44/envs/launch_rewards.py). A single 1-D apex curriculum (set_curriculum_
stage) widens h* over time. See docs/superpowers/specs/2026-06-08-v44-ll-launch-
primitive-design.md.
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

from rl.v44.envs.launch_command_manager import LaunchCommandManager
from rl.v44.envs import launch_rewards as R
from rl.v44.envs.launch_feasibility import STANDING_COM_HEIGHT

HEIGHT_OFFSET = 0.4
INITIAL_BODY_POSITION = [0.0, 0.0, HEIGHT_OFFSET]
INITIAL_QUAT = [1.0, 0.0, 0.0, 0.0]

# 1-D apex curriculum (spec §6). Low end fixed (cumulative skills); high end rises.
APEX_STAGES = {
    # v44.0.8: stage-1 low end lowered 0.40->0.38 (h*-0.05 gate => 0.33, just above the new 0.315
    # liftoff_progress floor) so the reachable on-ramp connects to the airborne apex reward. apex_match
    # stays gated on `airborne`, so a standing robot earns 0 regardless of the lower h*.
    1: {"h_low": 0.38, "h_high": 0.42, "vx_high": 0.5},   # A: clear hop (reachable first target)
    2: {"h_low": 0.40, "h_high": 0.52, "vx_high": 1.0},   # B
    3: {"h_low": 0.40, "h_high": 0.57, "vx_high": 1.5},   # C: full range
}

# Foot order: FL=0, FR=1, RL=2, RR=3 (matches GaitCommandManager convention)
_FOOT_NAMES = ("FL_foot", "FR_foot", "RL_foot", "RR_foot")
_GROUND_CONTACT_FORCE = 5.0   # N, above this a foot is "in contact"
# v44.0.2: raised from 1.0 N. Near-liftoff residual contact (whisker touch) must
# not block all_off detection. At 5.0 N, a foot generating < 5 N is considered
# airborne; standing GRF per foot is ~37 N so this threshold is still safe for
# normal contact detection. Fixes the root cause identified by @sdr-code-review.

# v44.0.5: relaxed landing detection (landing path ONLY — no other reward touched).
# The v44.0.2–0.4 landing one-shot required all four feet >5 N on the single step
# where _prev_airborne was still True. But _airborne (and therefore _prev_airborne)
# collapses to False the instant the FIRST foot touches (all_off needs zero feet
# down), so an asynchronous touchdown (feet settling 1-3 steps apart) never presents
# the all-4 edge — landing=0 in the v44.0.4 smoke despite visual-confirmed 4-foot
# landings. Fix: credit when a MAJORITY of feet are down within a short settle window
# after the last airborne step. Both relaxations are needed (the window keeps the
# edge open past first-foot collapse; the majority lets the trailing foot settle
# late). See knowledge_base/experiments/v44.0.5-landing-relaxation-spec.md.
_LANDING_MIN_FEET = 3        # credit a doglike touchdown on >=3/4 feet in contact
_LANDING_SETTLE_WINDOW = 5   # timer steps. The edge is open on the airborne step + the
                             # next 4 grounded steps (~0.08 s at dt=0.02) — ample for the
                             # 2-3 step asynchronous touchdown observed in the v44.0.4 smoke.


class Go2LaunchEnv(ManagedEnvironment):
    def __init__(self, num_envs: int = 1, dt: float = 1 / 50,
                 max_episode_length_s: float = 4.0, headless: bool = True):
        super().__init__(num_envs=num_envs, dt=dt,
                         max_episode_length_sec=max_episode_length_s,
                         max_episode_random_scaling=0.2)
        self._curriculum_stage = 1
        # v44.0.4: RSI mid-flight seeding fraction (vertical-drop seed) to bootstrap
        # the first liftoff/landing. Default 0.0 => behavior identical to ac4b48faf.
        # Per-env independent Bernoulli draw in reset(). See knowledge_base/
        # experiments/v44.0.4-rsi-adaptation-spec.md.
        self._rsi_fraction = float(os.environ.get("RSI_FRACTION", "0.0"))
        self._rsi_fraction = max(0.0, min(1.0, self._rsi_fraction))
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
        # 4-cam recording (same layout as v43; side cam = best arc view).
        self.camera = self.scene.add_camera(pos=(2.5, 1.5, 1.0), lookat=(1.0, 0.0, 0.4),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self.camera_side = self.scene.add_camera(pos=(1.0, -2.5, 0.5), lookat=(1.0, 0.0, 0.3),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        self.camera_top = self.scene.add_camera(pos=(1.0, 0.0, 3.0), lookat=(1.0, 0.0, 0.0),
            res=(480, 360), fov=60, env_idx=0, debug=True, GUI=False)
        self.camera_front = self.scene.add_camera(pos=(3.5, 0.0, 0.6), lookat=(0.0, 0.0, 0.3),
            res=(480, 360), fov=50, env_idx=0, debug=True, GUI=False)
        # Sidecar plumbing (set by trainer if a VideoWrapper is attached).
        self._video_wrapper_ref = None
        self._video_meta_buffer = []
        self._video_meta_frame_idx = 0
        self._video_meta_was_recording = False
        self._video_episode_id = 0
        self._step_id = 0
        # Foot link handles (populated in build()).
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
        self.launch_command = LaunchCommandManager(self, device=gs.device)
        self.launch_command.allocate()  # must be before ObservationManager.build()

        # Per-episode launch state tensors.
        self._alloc_launch_state()

        # Reward budget (per-episode maxima, clean stage-1 jump):
        #   liftoff (one-shot):        3.0 * 1.0  =  3.0   [hard gate + K_APEX=30]
        #   liftoff_progress (dense): 10.0 * (z_peak-0.315)  v44.0.8 telescoping, max ~2.25 (Go2 apex ~0.54)
        #   apex_match (one-shot):    10.0 * 1.0  = 10.0
        #   ascent_shaping (dense):    2.5 * ~5   = 12.5   [gated on flight window]
        #   forward_vel_track (dense): 1.5 * ~12  = 18.0
        #   landing (one-shot):        8.0 * 2.0  = 16.0
        #   grounded_posture (dense negative, weight 2.0): penalizes belly-splay/tilt.
        #   alive: v44.0.7 reverted to FULL (rewards.is_alive, 0.10/step ungated). v44.0.6
        #     gated it on _liftoff_seen and that removed the stability scaffold — the robot
        #     fell over (ep_len 18, fall_over 94%) and never produced the stand-then-jump
        #     trajectory liftoff_progress needs. Keep stability; break the standing attractor
        #     via the (now reliably firing) liftoff_progress gradient instead. See
        #     knowledge_base/experiments/v44.0.7-stable-jump-gradient-spec.md.
        self.reward_manager = RewardManager(self, logging_enabled=True, cfg={
            "liftoff":           {"weight": 3.0,   "fn": self._r_liftoff},
            "crouch_load":       {"weight": 1.5,   "fn": self._r_crouch_load},        # v44.0.9 (2.5->1.5)
            "liftoff_progress":  {"weight": 10.0,  "fn": self._r_liftoff_progress},  # v44.0.7: 6->10
            "apex_match":        {"weight": 10.0,  "fn": self._r_apex_match},
            "ascent_shaping":    {"weight": 2.5,   "fn": self._r_ascent_shaping},
            "forward_vel_track": {"weight": 1.5,   "fn": self._r_forward_vel_track},
            "posture_flight":    {"weight": 2.0,   "fn": self._r_posture_flight},
            "landing":           {"weight": 8.0,   "fn": self._r_landing},
            "grounded_posture":  {"weight": 2.0,   "fn": self._r_grounded_posture},
            "action_rate":       {"weight": -0.01, "fn": rewards.action_rate_l2},
            # v44.0.7: alive reverted to FULL (stability scaffold). The standing attractor
            # is broken by the liftoff_progress gradient, not by weakening alive.
            "alive":             {"weight": 0.1,   "fn": rewards.is_alive},
        })
        self.termination_manager = TerminationManager(self, logging_enabled=True, term_cfg={
            "timeout": {"fn": terminations.timeout, "time_out": True},
            "fall_over": {"fn": terminations.bad_orientation,
                          "params": {"limit_angle": 50.0, "entity_manager": self.robot_manager}},
        })
        ObservationManager(self, name="policy", history_len=5, cfg={
            "command": {"fn": self.launch_command.observation},                       # 2
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
        # v44.0.1: one-shot liftoff latch — reward fires at most once per episode.
        self._liftoff_credited = torch.zeros(n, dtype=torch.bool, device=dev)
        # v44.0.2: one-shot landing latch — mirrors _liftoff_credited.
        # Without this, every micro-touchdown after liftoff_seen is set earns a
        # landing reward (identified by @sdr-rl-red-team as the primary exploit).
        self._landing_credited = torch.zeros(n, dtype=torch.bool, device=dev)
        self._just_landed = torch.zeros(n, dtype=torch.bool, device=dev)
        self._prev_airborne = torch.zeros(n, dtype=torch.bool, device=dev)
        self._prev_vz = torch.zeros(n, device=dev)
        # v44.0.5: landing-settle countdown. Re-armed to _LANDING_SETTLE_WINDOW while
        # airborne, decremented once grounded; >0 means "was airborne within the last
        # window steps" — survives the first-foot-contact collapse of _prev_airborne.
        self._airborne_settle_timer = torch.zeros(n, dtype=torch.long, device=dev)
        # v44.0.6/0.7: telescoping liftoff-progress state. _running_peak_z holds the episode
        # height record; it is INITIALIZED TO THE FLOOR (v44.0.7) and only telescopes upward
        # after baseline, so spawn(0.4)/RSI(0.65)/transient contact heights never raise the
        # bar — the bar is exactly the floor at baseline regardless of the baseline-step z.
        # _peak_baseline_set gates progress; _grounded_run (v44.0.7) counts consecutive
        # grounded steps for the robust baseline fallback; _liftoff_progress_buf carries the
        # per-step telescoping reward to _r_liftoff_progress.
        self._running_peak_z = torch.full((n,), R._LIFTOFF_FLOOR_Z, device=dev)
        self._peak_baseline_set = torch.zeros(n, dtype=torch.bool, device=dev)
        self._grounded_run = torch.zeros(n, dtype=torch.long, device=dev)
        self._liftoff_progress_buf = torch.zeros(n, device=dev)
        # v44.0.9: crouch-load one-shot. _crouch_credited latches once a genuine load crouch is
        # credited; _stood_since_baseline gates it on the robot first reaching a stable stand
        # (z>=0.30 post-baseline) so the spawn-settle dip-through never credits.
        self._crouch_credited = torch.zeros(n, dtype=torch.bool, device=dev)
        self._stood_since_baseline = torch.zeros(n, dtype=torch.bool, device=dev)

    # ── curriculum ────────────────────────────────────────────────────────────
    def set_curriculum_stage(self, stage: int):
        self._curriculum_stage = stage
        # launch_command is created in config(); guard pre-build calls.
        if hasattr(self, "launch_command") and self.launch_command.command is not None:
            s = APEX_STAGES[stage]
            self.launch_command.set_curriculum_range(s["h_low"], s["h_high"], s["vx_high"])

    def get_stage_gate_metric(self, name: str) -> float:
        return self.reward_manager.last_episode_mean_reward(name)

    # ── build / reset / step ─────────────────────────────────────────────────
    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)
        # Populate foot link handles used by _feet_in_contact().
        for name in _FOOT_NAMES:
            self._foot_links.append(self.robot.get_link(name))
        # Apply curriculum range now that foot links are available.
        self.set_curriculum_stage(self._curriculum_stage)
        # v44.0.4: record RSI fraction in the train log header (smoke gate G8).
        print(f"[v44.0.4] RSI_FRACTION={self._rsi_fraction}", flush=True)

    def reset(self, envs_idx=None):
        result = super().reset(envs_idx)
        # ManagedEnvironment.reset() returns (obs, extras) — unpack safely.
        if isinstance(result, tuple):
            obs = result[0]
        else:
            obs = result
        if envs_idx is None:
            envs_idx = torch.arange(self.num_envs, device=gs.device)
        self.launch_command.reset(envs_idx)
        for t in (self._all_feet_off, self._airborne, self._apex_fired,
                  self._apex_credited, self._liftoff_seen, self._liftoff_credited,
                  self._landing_credited,
                  self._just_landed, self._prev_airborne):
            t[envs_idx] = False
        for t in (self._peak_z, self._z_apex_at_fire, self._prev_vz):
            t[envs_idx] = 0.0
        # v44.0.5: clear the landing-settle countdown on reset (cross-episode hygiene).
        # RSI-seeded envs are airborne at spawn, so the first _update_launch_state_pre
        # re-arms this to _LANDING_SETTLE_WINDOW; no explicit RSI-block seed needed.
        self._airborne_settle_timer[envs_idx] = 0
        # v44.0.6/0.7: reset telescoping liftoff-progress state. Running peak resets to the
        # FLOOR (so it telescopes up only after baseline; spawn/RSI never raise the bar);
        # baseline + grounded-run counter cleared for the new episode.
        self._running_peak_z[envs_idx] = R._LIFTOFF_FLOOR_Z
        self._peak_baseline_set[envs_idx] = False
        self._grounded_run[envs_idx] = 0
        self._liftoff_progress_buf[envs_idx] = 0.0
        # v44.0.9: clear crouch-load one-shot + prior-stand gate.
        self._crouch_credited[envs_idx] = False
        self._stood_since_baseline[envs_idx] = False

        # ── v44.0.4: RSI mid-flight (vertical-drop) seeding ───────────────────
        # Per-env independent Bernoulli over the envs being reset. Seeds a high
        # airborne pose (vertical drop from rest at z=0.65 — base velocity is NOT
        # injectable via the Genesis API, per the proven v39.9 RSI). The seeded
        # env earns the landing one-shot (gated on _liftoff_seen) and learns the
        # airborne->land value backward. Fix B's single-flight window closes at the
        # seeded landing, so no bounce-farm. Spec: knowledge_base/experiments/
        # v44.0.4-rsi-adaptation-spec.md (Hermes-approved 2026-06-09).
        if self._rsi_fraction > 0.0 and len(envs_idx) > 0:
            dev = gs.device
            draw = torch.rand(len(envs_idx), device=dev) < self._rsi_fraction
            rsi_ids = envs_idx[draw]
            n_rsi = len(rsi_ids)
            if n_rsi > 0:
                # Physical pose: high, level, tucked (proven v39.9 pose).
                pos = torch.zeros(n_rsi, 3, device=dev); pos[:, 2] = 0.65
                self.robot.set_pos(pos, envs_idx=rsi_ids)
                quat = torch.zeros(n_rsi, 4, device=dev); quat[:, 0] = 1.0
                self.robot.set_quat(quat, envs_idx=rsi_ids)
                act_dofs = self.actuator_manager.dofs_idx
                tuck = torch.zeros(n_rsi, 12, device=dev)
                tuck[:, [1, 4, 7, 10]] = 1.5    # thigh
                tuck[:, [2, 5, 8, 11]] = -2.4   # calf (hip stays 0)
                self.robot.set_dofs_position(tuck, act_dofs, envs_idx=rsi_ids)
                self.robot.set_dofs_velocity(
                    torch.zeros(n_rsi, 12, device=dev), act_dofs, envs_idx=rsi_ids)
                # v44 launch-state latches (the adaptation).
                self._liftoff_seen[rsi_ids] = True      # enables landing one-shot + flight window
                self._liftoff_credited[rsi_ids] = True  # seed = completed liftoff; one-shot won't mis-fire
                self._landing_credited[rsi_ids] = False  # seeded descent->land earns landing once
                self._apex_fired[rsi_ids] = False        # no vz>0->vz<=0 crossing on a pure descent
                self._apex_credited[rsi_ids] = False
                self._airborne[rsi_ids] = True
                self._all_feet_off[rsi_ids] = True
                self._prev_airborne[rsi_ids] = True      # landing edge needs "was airborne"
                # v44.0.5: arm the landing-settle window for the seed explicitly. The
                # seed IS airborne (z=0.65), but _update_launch_state_pre re-derives the
                # timer from physics contacts; arming it here removes any dependency on
                # Genesis reporting zero foot contact on the first post-set_pos() step,
                # so the seeded landing (the value RSI exists to bootstrap) always
                # credits. (@sdr-code-review v44.0.5, non-blocking hardening.)
                self._airborne_settle_timer[rsi_ids] = _LANDING_SETTLE_WINDOW
                self._peak_z[rsi_ids] = 0.65
                self._prev_vz[rsi_ids] = 0.0
                # Command alignment: seed has no forward velocity (vertical drop).
                self.launch_command.command[rsi_ids, 1] = 0.0

        return result  # pass the full return through (RslRlWrapper expects tuple)

    def step(self, actions):
        self._update_launch_state_pre()       # uses state from previous physics step
        # ManagedEnvironment.step() returns 5-tuple: (obs, rewards, terminated, truncated, extras)
        obs, rew, terminated, truncated, infos = super().step(actions)
        dones = terminated | truncated
        self._step_id += 1
        # Increment episode counter for env 0 on done.
        if dones is not None and dones.numel() > 0 and dones[0].item():
            self._video_episode_id += 1
        self._buffer_video_metadata()
        # Return 5-tuple so RslRlWrapper.step() can split terminated/truncated correctly.
        return obs, rew, terminated, truncated, infos

    def _update_launch_state_pre(self):
        pos = self.robot.get_pos()
        vel = self.robot_manager.get_linear_velocity()
        # Flatten in case shape is (n, 1, 3).
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        z = pos[:, 2]
        vz = vel[:, 2]
        # foot contact -> airborne when all four feet are off AND CoM is above
        # the height-guard threshold.
        # v44.0.2 ROOT FIX: belly-splay at z=0.15–0.19 m has feet unloaded but is
        # NOT genuinely airborne. Without the height guard, grounded_posture (gated
        # on ~airborne) never fires on the splay. Guard set to 0.28 m > splay range
        # (0.19 m) but < normal standing height (~0.35 m) so genuine early-liftoff
        # (robot rising from crouch) is correctly classified as airborne.
        _AIRBORNE_MIN_Z = 0.28  # m; body below this cannot be genuinely airborne
        foot_down = self._feet_in_contact()
        all_off = ~foot_down.any(dim=-1) & (z > _AIRBORNE_MIN_Z)
        self._all_feet_off = all_off
        self._airborne = all_off
        # v44.0.5: maintain the landing-settle countdown. Re-arm to the full window
        # while airborne; decay (clamped >=0) once grounded. "recently_airborne" then
        # persists for _LANDING_SETTLE_WINDOW steps past the first-foot-contact step
        # that collapses _airborne/_prev_airborne to False — this is what lets an
        # asynchronous touchdown be observed by the landing edge.
        self._airborne_settle_timer = torch.where(
            self._airborne,
            torch.full_like(self._airborne_settle_timer, _LANDING_SETTLE_WINDOW),
            torch.clamp(self._airborne_settle_timer - 1, min=0),
        )
        self._liftoff_seen |= all_off & (vz > 0.0)
        self._peak_z = torch.maximum(self._peak_z, torch.where(all_off, z, self._peak_z))
        # v44.0.7: telescoping liftoff-progress with a robust baseline. _running_peak_z is
        # initialized to the floor and frozen until baseline, so the bar is exactly the floor
        # regardless of the baseline-step z (zero spawn/RSI/transient contamination). Baseline
        # fires once a grounded robot settles below the floor OR has been grounded for
        # _PROGRESS_BASELINE_GROUNDED_STEPS (the fallback that the v44.0.6 settle-only trigger
        # lacked — it starved when the robot fell before settling). Then reward only NEW height
        # records above the floor (telescoping ⇒ bobbing/crouch-pump/bounce earn 0).
        grounded = foot_down.any(dim=-1)
        self._grounded_run = torch.where(
            grounded, self._grounded_run + 1, torch.zeros_like(self._grounded_run))
        settle = grounded & (~self._peak_baseline_set) & (
            (z <= R._LIFTOFF_FLOOR_Z) | (self._grounded_run >= R._PROGRESS_BASELINE_GROUNDED_STEPS))
        self._peak_baseline_set = self._peak_baseline_set | settle
        self._liftoff_progress_buf, self._running_peak_z = R.liftoff_progress(
            z, self._running_peak_z, self._peak_baseline_set)
        # v44.0.9: latch that the robot reached a stable stand (z>=0.30) post-baseline. A crouch
        # only counts as an intentional load AFTER a stand, so the spawn-settle dip never credits.
        self._stood_since_baseline |= self._peak_baseline_set & grounded & (z >= 0.30)
        # apex one-shot: airborne and vertical velocity crosses + -> -.
        crossed = self._airborne & (self._prev_vz > 0.0) & (vz <= 0.0) & (~self._apex_fired)
        self._z_apex_at_fire = torch.where(crossed, z, self._z_apex_at_fire)
        self._apex_fired |= crossed
        # v44.0.5 landing edge (telemetry mirror of the reward gate in _r_landing):
        # recently airborne AND a majority of feet (>=_LANDING_MIN_FEET of 4) down.
        # Drives only the video "landing" event + eval land-rate; the reward path
        # recomputes the same condition on POST-physics contacts. _just_landed here uses
        # the PRE-physics contacts (same fetch as _airborne), so it can lead/lag the
        # reward credit by one step and is NOT authoritative — the G1 smoke gate reads
        # the RewardManager 'landing' metric, not the video event. _prev_airborne is
        # preserved as-is for the "launch" event annotation below.
        recently_airborne = self._airborne_settle_timer > 0
        feet_down_count = foot_down.sum(dim=-1)
        self._just_landed = recently_airborne & (feet_down_count >= _LANDING_MIN_FEET)
        self._prev_airborne = self._airborne.clone()
        self._prev_vz = vz.clone()

    def _feet_in_contact(self) -> torch.Tensor:
        """(num_envs, 4) bool — per-foot ground contact via contact force magnitude.

        Uses foot_contact_manager.contacts directly (shape: n_envs, 4, 3) which
        is populated by ContactManager.step() each physics step.  This is the same
        data source as get_contact_forces() but avoids the per-link index lookup
        that returns shape (n_envs, 1, 3), requiring a squeeze to avoid broadcast
        errors.  contacts[:, :4, :] covers FL/FR/RL/RR in the order the regex
        '.*_foot' resolves (alphabetical: FL, FR, RL, RR — matches FOOT_NAMES).
        """
        if self.foot_contact_manager.contacts is None:
            # ContactManager not yet built; return all-grounded as safe default.
            return torch.ones(self.num_envs, 4, dtype=torch.bool, device=gs.device)
        # contacts shape: (n_envs, 4, 3) — one row per matched link
        force_norm = torch.norm(self.foot_contact_manager.contacts, dim=-1)  # (n_envs, 4)
        return force_norm > _GROUND_CONTACT_FORCE

    # ── reward methods (thin wrappers into launch_rewards) ────────────────────
    def _vz(self):
        vel = self.robot_manager.get_linear_velocity()
        if vel.ndim > 2:
            vel = vel.view(vel.shape[0], -1)
        return vel[:, 2]

    def _r_liftoff(self, env):
        # v44.0.1 fix: TRUE one-shot latch.
        # _liftoff_credited is reset to False in reset() and latched True here on
        # the first step where all four feet leave the ground while ascending.
        # After that step the reward is 0 for the rest of the episode, preventing
        # the belly-bob exploit where liftoff fired every ascending-airborne step.
        newly = self._all_feet_off & (self._vz() > 0.0) & (~self._liftoff_credited)
        self._liftoff_credited |= newly
        return newly.to(torch.float)

    def _r_apex_match(self, env):
        # _apex_credited is reset in reset(); newly = apex fired but not yet credited.
        newly = self._apex_fired & (~self._apex_credited)
        self._apex_credited |= newly
        return R.apex_match(self._z_apex_at_fire, self.launch_command.command[:, 0],
                            k=R.K_APEX, fired=newly)

    def _flight_active(self):
        # v44.0.3 single-flight window: True only between the episode's genuine
        # liftoff and its genuine landing. _liftoff_seen latches at the first
        # all-four-feet liftoff; _landing_credited latches at the first earned
        # landing. Their AND (~credited) marks the one sanctioned launch arc, so
        # repeated post-landing bounces earn no flight reward (kills the red-team
        # 4.55x forward_vel/ascent bounce-farm, duration-independent).
        return self._liftoff_seen & (~self._landing_credited)

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
        foot_down = self._feet_in_contact().to(torch.float)  # (n,4) FL,FR,RL,RR
        left = foot_down[:, 0] + foot_down[:, 2]
        right = foot_down[:, 1] + foot_down[:, 3]
        return torch.abs(left - right)

    def _r_landing(self, env):
        # v44.0.5: relaxed landing detection. The v44.0.2–0.4 condition
        # (_prev_airborne & all-4-feet-down on a single step) missed asynchronous
        # touchdowns — _prev_airborne collapses to False the instant the first foot
        # contacts, so the all-4 edge was unobservable (landing=0 in the v44.0.4
        # smoke despite visual 4-foot landings). Credit when a MAJORITY of feet
        # (>=_LANDING_MIN_FEET of 4) are in contact within the settle window after the
        # last airborne step (_airborne_settle_timer maintained in
        # _update_launch_state_pre using last-step state, then read here against
        # post-physics contacts — same pre/post timing the v44.0.2 fix relied on).
        # Both guards below are UNCHANGED, so the v44.0.3 single-flight window and
        # bounce-farm closure stay intact. See v44.0.5 spec.
        foot_down = self._feet_in_contact()
        feet_down_count = foot_down.sum(dim=-1)
        landed = (feet_down_count >= _LANDING_MIN_FEET) & (self._airborne_settle_timer > 0)
        pitch = torch.abs(self.robot_manager.get_projected_gravity()[:, 0])  # gx ~ pitch proxy
        z = self.robot.get_pos()[:, 2]
        # Guard 1: only credit if a genuine liftoff occurred this episode.
        # Guard 2: one-shot latch — landing reward fires at most once per episode,
        #          mirroring _liftoff_credited. Prevents micro-touch farming.
        earned_landing = landed & self._liftoff_seen & (~self._landing_credited)
        self._landing_credited |= earned_landing
        # `landed` is the flat/all-feet-down mask: earned_landing ⊆ landed, so flat==1
        # on every credited step — reward at credit = pitch_ok + height_ok (max 2.0),
        # the same budget as v44.0.4. Only the firing condition is relaxed.
        return R.landing(landed, pitch, z, earned_landing)

    def _r_grounded_posture(self, env):
        # v44.0.1 anti-collapse penalty while grounded; v44.0.3: threshold 0.22 m.
        # Penalizes belly-splay (z < _GROUNDED_MIN_Z=0.22 m) and tilt while NOT
        # airborne. fn returns <=0, env weight=2.0; accumulates over grounded steps
        # so belly-splay stays strongly negative vs the alive bonus, while legit
        # crouch-load (CoM floor ~0.244 m, above 0.22) is NOT penalized.
        z = self.robot.get_pos()[:, 2]
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        return R.grounded_posture(z, gz, self._airborne)

    def _r_crouch_load(self, env):
        # v44.0.9 (+0.9b coupling): one-shot reward for SPRINGING UP out of a genuine pre-jump LOAD
        # crouch (teaches load-and-spring -> controlled take-off; the v44.0.8 longer run reverted to
        # standing because uncontrolled leaps fell). The reviewers' convergent concern was that
        # rewarding the *dip* lets the policy farm stand->dip->re-stand (the attractor in disguise);
        # the fix couples the credit to the upward spring. Seven gates make it farm-resistant (spec §4):
        #   grounded               — a crouch, not a flight dip;
        #   _stood_since_baseline  — followed a real stand (not the spawn-settle dip-through);
        #   upright (gz<=-0.9)     — not a tilted collapse;
        #   z in [floor, trigger]  — a real load, 2 cm ABOVE the belly-splay floor;
        #   vz > _CROUCH_SPRING_VZ — RISING out of the load (the spring), NOT a dip-and-stay;
        #   ~_liftoff_seen         — the load that precedes the FIRST liftoff (kills post-landing bounce);
        #   ~_crouch_credited      — one-shot per episode.
        z = self.robot.get_pos()[:, 2]
        gz = self.robot_manager.get_projected_gravity()[:, 2]
        grounded = self._feet_in_contact().any(dim=-1)
        in_band = (z >= R._CROUCH_FLOOR_Z) & (z <= R._CROUCH_TRIGGER_Z)
        fired = (grounded & self._stood_since_baseline & (gz <= -0.9) & in_band
                 & (self._vz() > R._CROUCH_SPRING_VZ) & (~self._liftoff_seen)
                 & (~self._crouch_credited))
        self._crouch_credited |= fired
        return R.crouch_load(z, fired)

    def _r_liftoff_progress(self, env):
        # v44.0.6 Lever 2: telescoping height-progress reward. The per-step value is
        # computed in _update_launch_state_pre (baseline + telescoping vs the episode
        # peak) and buffered here. Farm-resistant: only new records above the floor pay.
        return self._liftoff_progress_buf

    def _obs_base_height(self, env):
        return self.robot.get_pos()[:, 2:3]

    def _obs_base_height_peak(self, env):
        return self._peak_z.unsqueeze(-1)

    # ── sidecar writer (adapted from go2_skill_env.py:2548-2660, no bar) ──────
    def _flatten_vec(self, vec: torch.Tensor) -> torch.Tensor:
        if vec.ndim > 2:
            return vec.view(vec.shape[0], -1)
        return vec

    def _buffer_video_metadata(self):
        """Buffer env_idx=0 physics state for video metadata sidecar."""
        vw = self._video_wrapper_ref
        if vw is None:
            return

        is_recording = vw._is_recording and vw._recording_type == "active"

        # Reset buffer on recording start transition.
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

                # Events for this frame (v44: launch/landing/peak only; no bar events).
                events = []
                # Liftoff detection: first frame where robot becomes airborne.
                if self._all_feet_off[0].item() and not self._prev_airborne[0].item():
                    events.append("launch")
                if self._just_landed[0].item():
                    events.append("landing")
                # "peak" is annotated post-hoc in _write_video_metadata_sidecar.

                vel = self._flatten_vec(self.robot_manager.get_linear_velocity())
                proj_grav = self._flatten_vec(self.robot_manager.get_projected_gravity())
                ang = self._flatten_vec(self.robot_manager.get_angular_velocity())

                self._video_meta_buffer.append({
                    "idx": self._video_meta_frame_idx,
                    "episode": self._video_episode_id,
                    "phase": phase,
                    "com_z": round(com_z, 4),
                    "com_x": round(com_x, 4),
                    "vel_x": round(float(vel[0, 0].item()), 4),
                    "vel_z": round(float(vel[0, 2].item()), 4),
                    "gz": round(float(proj_grav[0, 2].item()), 4),
                    "pitch_rate": round(float(ang[0, 1].item()), 3),
                    "events": events,
                })
                self._video_meta_frame_idx += 1

        # Detect recording end -> write sidecar.
        if self._video_meta_was_recording and not is_recording:
            self._write_video_metadata_sidecar()

        self._video_meta_was_recording = is_recording

    def _write_video_metadata_sidecar(self):
        """Write buffered metadata as JSON sidecar next to the MP4."""
        import json as _json
        vw = self._video_wrapper_ref
        if not self._video_meta_buffer or vw is None:
            self._video_meta_buffer = []
            self._video_meta_frame_idx = 0
            return

        # Annotate peak height frame (highest airborne frame).
        max_z = -1.0
        peak_idx = -1
        for f in self._video_meta_buffer:
            if f["phase"] == "airborne" and f["com_z"] > max_z:
                max_z = f["com_z"]
                peak_idx = f["idx"]
        if peak_idx >= 0:
            for f in self._video_meta_buffer:
                if f["idx"] == peak_idx and "peak" not in f["events"]:
                    f["events"].append("peak")

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
            pass  # Non-fatal

        self._video_meta_buffer = []
        self._video_meta_frame_idx = 0
