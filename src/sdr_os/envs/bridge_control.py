"""
Shared bridge-control behavior for live (gamepad-driven) Go2 environments.

Extracted from Go2BridgeEnv so that every bridge env — walk (go2_env.py),
crawl (go2_crawl_bridge_env.py), and future skill bridges — shares one
implementation of:

- stand/walk PD gain switching (stand mode also widens the action scale)
- IK-style stand pose control from joystick axes
- obs/reward breakdown telemetry for the web UI

Host classes must provide: robot, actuator_manager, action_manager,
obs_groups, managers (genesis_forge ManagedEnvironment), num_envs,
get_observations().
"""

import torch
import genesis as gs

# PD gains — walk must match policy training; stand is higher for passive stability
WALK_KP = 20.0
WALK_KV = 0.5
STAND_KP = 50.0
STAND_KV = 2.0


class BridgeControlMixin:
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

    def get_obs_breakdown(self) -> dict:
        """
        Decompose the flat observation tensor into named groups.
        Returns dict of {group_name: list_of_floats} for env 0.
        Uses only the most recent frame of the history-stacked obs.
        Used by bridge_server to emit structured obs to the UI.
        """
        # Read the per-step obs cache directly — calling get_observations()
        # outside the step loop would push a frame into the obs-manager history
        # and corrupt the policy's 5-frame stack.
        obs = self.extras.get("observations", {}).get("policy")
        if obs is None:
            return {}

        # With history_len=5, obs is 5 * frame_size. Take last frame.
        frame_size = sum(size for _, size in self.obs_groups)
        latest_frame = obs[0, -frame_size:]

        breakdown = {}
        offset = 0
        for name, size in self.obs_groups:
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

    @staticmethod
    def _map_stick(value: float, range_min: float, range_max: float) -> float:
        """Map stick value (-1..1) to a velocity range."""
        return (value + 1.0) * (range_max - range_min) / 2.0 + range_min
