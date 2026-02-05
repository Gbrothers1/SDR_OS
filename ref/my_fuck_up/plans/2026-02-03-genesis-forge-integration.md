# Genesis-Forge Integration Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace SDR_OS's manual Genesis scene management with genesis-forge's `ManagedEnvironment`, wire a Go2 quadruped into the web UI via the existing bridge server, add mode controls to TrustStrip, and build a preset-based Telemetry Console for the right panel.

**Architecture:** A new `Go2BridgeEnv` (subclass of genesis-forge `ManagedEnvironment`) lives in `genesis_bridge/envs/`. The existing `bridge_server.py` instantiates it instead of manually creating Genesis scenes. Gamepad input from Socket.io feeds a `VelocityCommandManager`. The bridge emits structured observation/reward/command breakdowns alongside existing events. On the frontend, TrustStrip gains mode buttons (TELEOP/POLICY/BLEND) + alpha slider. The right EdgePanel becomes a `TelemetryConsole` with preset-based layouts (Operate/Sim/Split) that auto-switch based on connection state.

**Tech Stack:** Python 3, genesis-forge (ManagedEnvironment, ObservationManager, VelocityCommandManager, etc.), PyTorch, Socket.io (python-socketio), React 18, CSS custom properties (existing design system)

**Design doc:** `docs/plans/2026-02-03-genesis-forge-integration-design.md`

---

## Task 1: Create Go2BridgeEnv

The genesis-forge environment class that wraps the Go2 quadruped for use inside bridge_server.py.

**Files:**
- Create: `genesis_bridge/envs/__init__.py`
- Create: `genesis_bridge/envs/go2_env.py`
- Reference: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/examples/simple/environment.py`
- Reference: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/genesis_forge/managed_env.py`

**Step 1: Create the package init**

Create `genesis_bridge/envs/__init__.py`:

```python
from .go2_env import Go2BridgeEnv

__all__ = ["Go2BridgeEnv"]
```

**Step 2: Create Go2BridgeEnv**

Create `genesis_bridge/envs/go2_env.py`. This is adapted from genesis-forge's `examples/simple/environment.py` with additions for bridge integration (camera, velocity command manager):

```python
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
OBS_GROUPS = [
    ("angular_velocity", 3),
    ("linear_velocity", 3),
    ("projected_gravity", 3),
    ("velocity_command", 3),
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

        # Observations (48 dim total)
        ObservationManager(
            self,
            cfg={
                "angular_velocity": {
                    "fn": lambda env: self.robot_manager.get_angular_velocity(),
                    "scale": 0.25,
                },
                "linear_velocity": {
                    "fn": lambda env: self.robot_manager.get_linear_velocity(),
                    "scale": 2.0,
                },
                "projected_gravity": {
                    "fn": lambda env: self.robot_manager.get_projected_gravity(),
                },
                "velocity_command": {
                    "fn": self.velocity_command.observation,
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

    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)

    def get_obs_breakdown(self) -> dict:
        """
        Decompose the flat observation tensor into named groups.
        Returns dict of {group_name: list_of_floats} for env 0.
        Used by bridge_server to emit structured obs to the UI.
        """
        obs = self.get_observations()
        if obs is None:
            return {}

        breakdown = {}
        offset = 0
        for name, size in OBS_GROUPS:
            values = obs[0, offset:offset + size]
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
        - Left stick Y (inverted) → lin_vel_x (forward/back)
        - Left stick X → lin_vel_y (strafe)
        - Right stick X → ang_vel_z (turn)
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
```

**Step 3: Verify the file imports correctly**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -c "from genesis_bridge.envs import Go2BridgeEnv; print('OK')"`

Expected: `OK` (or ImportError for genesis/genesis_forge if not in the current venv — that's fine, the import path itself must not error on syntax)

**Step 4: Commit**

```bash
git add genesis_bridge/envs/__init__.py genesis_bridge/envs/go2_env.py
git commit -m "feat: add Go2BridgeEnv genesis-forge environment for bridge server"
```

---

## Task 2: Adapt bridge_server.py to use Go2BridgeEnv

Replace manual Genesis scene management with the genesis-forge environment. The bridge's main loop calls `env.step()` instead of `scene.step()`, and uses `env.camera.render()` for frames.

**Files:**
- Modify: `genesis_bridge/bridge_server.py`

**Step 1: Add Go2BridgeEnv import and new init path**

At the top of `bridge_server.py`, after the existing genesis import block (around line 47), add:

```python
# genesis-forge environment (optional — graceful fallback if not available)
_genesis_forge_available = False
try:
    from genesis_bridge.envs import Go2BridgeEnv
    _genesis_forge_available = True
except ImportError:
    pass
```

**Step 2: Add env attribute to GenesisBridgeServer.__init__**

In `GenesisBridgeServer.__init__` (around line 376), after `self.env = None`, add:

```python
self.forge_env = None  # genesis-forge ManagedEnvironment
```

**Step 3: Replace initialize_genesis with forge env creation**

Replace the body of `initialize_genesis()` (keeping the method signature). The new version:
1. If genesis-forge is available, creates a `Go2BridgeEnv` and calls `build()`
2. Sets `self.camera` and `self.scene` from the env for backward compat
3. Falls back to existing mock mode if genesis-forge is not available

Add a new method `_initialize_forge_env` after `initialize_genesis`:

```python
async def _initialize_forge_env(self):
    """Initialize a genesis-forge ManagedEnvironment."""
    if not _genesis_forge_available or gs is None:
        return False

    try:
        logger.info("Creating Go2BridgeEnv (genesis-forge)...")
        self.forge_env = Go2BridgeEnv(
            num_envs=self.config.num_envs,
            dt=self.config.dt,
            headless=True,
            camera_res=self.config.camera_res,
        )
        self.forge_env.build()

        # Backward compat: expose scene/camera/robot
        self.scene = self.forge_env.scene
        self.camera = self.forge_env.camera
        self.robot = self.forge_env.robot
        self.scene_built = True

        # Initial reset
        obs, _ = self.forge_env.reset()
        self.current_obs = obs

        logger.info("Go2BridgeEnv initialized successfully")
        return True
    except Exception as e:
        logger.error(f"Failed to initialize Go2BridgeEnv: {e}")
        self.forge_env = None
        return False
```

In `initialize_genesis()`, at the start of the genesis-available branch, add:

```python
# Try genesis-forge first
if _genesis_forge_available:
    success = await self._initialize_forge_env()
    if success:
        return True
    logger.warning("genesis-forge init failed, falling back to manual scene setup")
```

**Step 4: Replace step_simulation for forge env**

In `step_simulation()` (line 935), add a forge_env path at the top:

```python
def step_simulation(self):
    """Execute one simulation step."""
    if self.auto_stopped:
        return

    # genesis-forge path
    if self.forge_env is not None:
        self._step_forge_env()
        return

    # ... existing manual path below ...
```

Add the new method:

```python
def _step_forge_env(self):
    """Step the genesis-forge environment."""
    try:
        # Create zero actions (policy will override later)
        num_actions = self.forge_env.action_space.shape[0]
        actions = torch.zeros(
            (self.forge_env.num_envs, num_actions),
            device=gs.device,
        )

        obs, reward, terminated, truncated, extras = self.forge_env.step(actions)

        self.current_obs = obs
        self.step_count += 1

        # Track rewards
        if reward is not None:
            self.total_reward = float(reward[0])

        # Track episode resets
        if terminated is not None and truncated is not None:
            done = (terminated | truncated).any().item()
            if done:
                self.episode_count += 1
                self.episode_rewards.append(self.total_reward)
                self.total_reward = 0.0

    except Exception as e:
        logger.error(f"Forge env step error: {e}")
```

**Step 5: Update render_frame for forge env**

In `render_frame()` (line 709), add a forge_env path before the existing camera render:

```python
def render_frame(self) -> Optional[bytes]:
    # Forge env path
    if self.forge_env is not None and self.forge_env.camera is not None:
        try:
            render_result = self.forge_env.camera.render(rgb=True)
            if isinstance(render_result, tuple):
                rgb_array = render_result[0]
            else:
                rgb_array = gs.utils.misc.tensor_to_array(render_result)

            bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            _, jpeg_buf = cv2.imencode(
                '.jpg', bgr_array,
                [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality]
            )
            return jpeg_buf.tobytes()
        except Exception as e:
            if not hasattr(self, '_last_render_error_time') or time.time() - self._last_render_error_time > 5.0:
                logger.error(f"Forge render error: {e}")
                self._last_render_error_time = time.time()
            return None

    # ... existing render_frame code below (mock mode + manual camera) ...
```

**Step 6: Update get_training_metrics for forge env**

In `get_training_metrics()` (line 885), add forge env data:

```python
def get_training_metrics(self) -> Dict[str, Any]:
    metrics = {
        "type": "training_metrics",
        "step_count": self.step_count,
        "num_envs": self.config.num_envs,
        "total_reward": self.total_reward,
        "fps": self.fps,
        "mode": self.mode,
        "episode_count": self.episode_count,
    }

    # Forge env structured data
    if self.forge_env is not None:
        metrics["obs_breakdown"] = self.forge_env.get_obs_breakdown()
        metrics["reward_breakdown"] = self.forge_env.get_reward_breakdown()
        metrics["velocity_command"] = self.forge_env.get_velocity_command()
        metrics["num_envs"] = self.forge_env.num_envs

    # ... keep existing router_state, memory_stats, etc ...
```

**Step 7: Wire gamepad joystick to velocity command**

In the Socket.io `controller_joystick_state` handler (line 1266), add forge env path:

```python
@self.sio.on('controller_joystick_state')
async def on_joystick(data):
    if self.forge_env is not None:
        self.forge_env.set_velocity_from_gamepad(data)
    else:
        self.teleop_bridge.get_action(joystick_state=data)
```

**Step 8: Update genesis_reset handler for forge env**

In the `genesis_reset` handler (line 1280):

```python
@self.sio.on('genesis_reset')
async def on_reset(data):
    logger.info("Resetting environment")
    if self.forge_env is not None:
        obs, _ = self.forge_env.reset()
        self.current_obs = obs
        self.step_count = 0
        self.total_reward = 0.0
    else:
        self.router.reset()
        self.action_spec.reset()
    # ... keep rest of handler ...
```

**Step 9: Verify the bridge starts**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python genesis_bridge/bridge_server.py --help`

Expected: Should show help/start without crashing. If Genesis is not installed, it should fall back to mock mode gracefully.

**Step 10: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat: wire Go2BridgeEnv into bridge_server with forge env step/render path"
```

---

## Task 3: Add new Socket.io events to server.js relay

The Node.js server needs to forward the new structured telemetry events between bridge and browser clients.

**Files:**
- Modify: `server.js:26-48`

**Step 1: Add new events to GENESIS_EVENTS array**

In `server.js`, add these to the `GENESIS_EVENTS` array (line 26):

```javascript
'genesis_obs_breakdown',
'genesis_reward_breakdown',
'genesis_velocity_command',
'genesis_estop',
```

**Step 2: Verify the server starts**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && node -e "require('./server.js')" &` then kill it.

Or simpler: `node -c server.js` (syntax check only).

**Step 3: Commit**

```bash
git add server.js
git commit -m "feat: add genesis-forge telemetry events to Socket.io relay"
```

---

## Task 4: Add new events to GenesisContext

The React context needs to receive and expose the new structured telemetry data.

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx`

**Step 1: Add state for new telemetry data**

After the existing state declarations (around line 42), add:

```javascript
// Structured telemetry from genesis-forge
const [obsBreakdown, setObsBreakdown] = useState(null);
const [rewardBreakdown, setRewardBreakdown] = useState(null);
const [velocityCommand, setVelocityCommand] = useState(null);
```

**Step 2: Add Socket.io listeners**

In the `useEffect` that sets up socket listeners (around line 280-303), add handlers:

```javascript
const handleObsBreakdown = (data) => setObsBreakdown(data);
const handleRewardBreakdown = (data) => setRewardBreakdown(data);
const handleVelocityCommand = (data) => setVelocityCommand(data);

socket.on('genesis_obs_breakdown', handleObsBreakdown);
socket.on('genesis_reward_breakdown', handleRewardBreakdown);
socket.on('genesis_velocity_command', handleVelocityCommand);
```

And in the cleanup:

```javascript
socket.off('genesis_obs_breakdown', handleObsBreakdown);
socket.off('genesis_reward_breakdown', handleRewardBreakdown);
socket.off('genesis_velocity_command', handleVelocityCommand);
```

**Step 3: Also extract from training metrics (fallback)**

The bridge may bundle obs/reward/velocity data inside `genesis_training_metrics` instead of separate events. Update the `handleTrainingMetrics` handler:

```javascript
const handleTrainingMetrics = (data) => {
  setTrainingMetrics(data);
  // Extract structured telemetry if bundled in metrics
  if (data.obs_breakdown) setObsBreakdown(data.obs_breakdown);
  if (data.reward_breakdown) setRewardBreakdown(data.reward_breakdown);
  if (data.velocity_command) setVelocityCommand(data.velocity_command);
};
```

**Step 4: Add estop action**

After the existing `setAlpha` action:

```javascript
const estop = useCallback(() => {
  if (socket) {
    socket.emit('genesis_estop', {});
    // Also pause locally
    pauseSim(true);
  }
}, [socket, pauseSim]);
```

**Step 5: Add to provider value**

In the `value` object (around line 376), add:

```javascript
// Structured telemetry
obsBreakdown,
rewardBreakdown,
velocityCommand,

// Actions
estop,
```

**Step 6: Build and verify**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`

Expected: Build succeeds with no errors.

**Step 7: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx
git commit -m "feat: add structured telemetry state and estop action to GenesisContext"
```

---

## Task 5: Add mode controls to TrustStrip

Add TELEOP/POLICY/BLEND mode buttons, alpha slider, and E-Stop to the TrustStrip bar.

**Files:**
- Modify: `src/client/components/TrustStrip.jsx`
- Modify: `src/client/styles/TrustStrip.css`

**Step 1: Update TrustStrip.jsx with mode controls**

Replace the contents of `TrustStrip.jsx`:

```jsx
import React, { useCallback } from 'react';
import { usePhase } from '../contexts/PhaseContext';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/TrustStrip.css';

const MODES = [
  { key: 'teleop', label: 'TELEOP' },
  { key: 'policy', label: 'POLICY' },
  { key: 'blend', label: 'BLEND' },
];

const TREND_ARROWS = {
  rising: '\u2197',
  falling: '\u2198',
  flat: '\u2192',
};

const StageCell = ({ stage, data, collapsed, onClick }) => {
  const cellClass = [
    'stage-cell',
    collapsed && 'stage-cell--collapsed',
  ].filter(Boolean).join(' ');

  const dotClass = `stage-cell__dot stage-cell__dot--${data.health}`;

  return (
    <div className={cellClass} onClick={onClick}>
      <span className={dotClass} />
      {!collapsed && (
        <>
          <span className="stage-cell__label">{stage}</span>
          {stage === 'teleop' && (
            <span className="stage-cell__signal">
              {data.recording
                ? `${data.recordingStep ?? '?'}/${data.recordingTotal ?? '?'}`
                : data.actor}
            </span>
          )}
          {stage === 'train' && data.health !== 'dead' && (
            <>
              <span className={`stage-cell__trend stage-cell__trend--${data.trend}${data.unstable ? ' stage-cell__trend--unstable' : ''}`}>
                {TREND_ARROWS[data.trend] || '\u2192'}
              </span>
              <span className="stage-cell__signal">
                {data.reward != null ? data.reward.toFixed(1) + 'r' : ''}
              </span>
            </>
          )}
          {stage === 'eval' && data.health !== 'dead' && (
            <span className={`stage-cell__risk stage-cell__risk--${data.riskLevel}`}>
              {data.riskLevel === 'nominal' ? 'nominal' :
                `${data.riskLevel}${data.clampCount > 0 ? ` \u00d7${data.clampCount}` : ''}`}
            </span>
          )}
          {data.error && (
            <span className="stage-cell__error" title={data.error}>
              {data.error.length > 30 ? data.error.slice(0, 30) + '...' : data.error}
            </span>
          )}
        </>
      )}
    </div>
  );
};

const TrustStrip = ({ onStageClick }) => {
  const { activePhase, authority, stages } = usePhase();
  const {
    genesisConnected,
    genesisMode,
    setMode,
    setAlpha,
    blendAlpha,
    estop,
    currentRobot,
  } = useGenesis();

  const stripClass = `trust-strip trust-strip--${authority}`;

  const isTeleopCollapsed = stages.teleop.health === 'dead' && activePhase !== 'teleop';
  const isTrainCollapsed = stages.train.health === 'dead' && activePhase !== 'train';
  const isEvalCollapsed = stages.eval.health === 'dead' && activePhase !== 'eval';

  const handleModeClick = useCallback((mode) => {
    setMode(mode);
  }, [setMode]);

  const handleAlphaChange = useCallback((e) => {
    setAlpha(parseFloat(e.target.value));
  }, [setAlpha]);

  const handleEstop = useCallback(() => {
    if (estop) estop();
  }, [estop]);

  // Map genesis mode names to our mode keys
  const activeModeKey = (() => {
    if (genesisMode === 'teleop_record' || genesisMode === 'teleop') return 'teleop';
    if (genesisMode === 'eval' || genesisMode === 'policy') return 'policy';
    if (genesisMode === 'hil_blend' || genesisMode === 'blend' || genesisMode === 'online_finetune') return 'blend';
    return 'teleop';
  })();

  return (
    <div className={stripClass}>
      {/* Connection + Robot */}
      <div className="trust-strip__connection">
        <span className={`trust-strip__conn-dot trust-strip__conn-dot--${genesisConnected ? 'on' : 'off'}`} />
        <span className="trust-strip__conn-label">
          {genesisConnected ? 'SIM' : 'OFFLINE'}
        </span>
        {currentRobot && (
          <span className="trust-strip__robot-name">{currentRobot.label || currentRobot.name}</span>
        )}
      </div>

      <div className="trust-strip__divider" />

      {/* Stage cells */}
      <StageCell stage="teleop" data={stages.teleop} collapsed={isTeleopCollapsed} onClick={() => onStageClick?.('teleop')} />
      <div className="trust-strip__divider" />
      <StageCell stage="train" data={stages.train} collapsed={isTrainCollapsed} onClick={() => onStageClick?.('train')} />
      <div className="trust-strip__divider" />
      <StageCell stage="eval" data={stages.eval} collapsed={isEvalCollapsed} onClick={() => onStageClick?.('eval')} />

      <div className="trust-strip__spacer" />

      {/* Mode buttons */}
      {genesisConnected && (
        <div className="trust-strip__mode-group">
          {MODES.map(({ key, label }) => (
            <button
              key={key}
              className={`trust-strip__mode-btn ${activeModeKey === key ? 'trust-strip__mode-btn--active' : ''}`}
              onClick={() => handleModeClick(key)}
            >
              {label}
            </button>
          ))}

          {/* Alpha slider — only in blend mode */}
          {activeModeKey === 'blend' && (
            <div className="trust-strip__alpha">
              <span className="trust-strip__alpha-label">α</span>
              <input
                type="range"
                min="0"
                max="1"
                step="0.05"
                value={blendAlpha}
                onChange={handleAlphaChange}
                className="trust-strip__alpha-slider"
              />
              <span className="trust-strip__alpha-value">{blendAlpha.toFixed(2)}</span>
            </div>
          )}
        </div>
      )}

      {/* E-STOP */}
      <button className="trust-strip__estop" onClick={handleEstop}>
        E-STOP
      </button>
    </div>
  );
};

export default TrustStrip;
```

**Step 2: Add mode control CSS to TrustStrip.css**

Append to `src/client/styles/TrustStrip.css`:

```css
/* ===== Connection indicator ===== */
.trust-strip__connection {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  padding: var(--space-1) var(--space-2);
  white-space: nowrap;
}

.trust-strip__conn-dot {
  width: 6px;
  height: 6px;
  border-radius: 50%;
  flex-shrink: 0;
}

.trust-strip__conn-dot--on {
  background: var(--color-accent-green);
  box-shadow: 0 0 6px rgba(62, 207, 142, 0.5);
}

.trust-strip__conn-dot--off {
  background: var(--color-health-dead);
  opacity: 0.5;
}

.trust-strip__conn-label {
  font-size: var(--font-size-xs);
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--color-text-secondary);
}

.trust-strip__robot-name {
  font-size: var(--font-size-xs);
  font-family: var(--font-mono);
  color: var(--color-text-primary);
}

/* ===== Mode buttons ===== */
.trust-strip__mode-group {
  display: flex;
  align-items: center;
  gap: var(--space-1);
}

.trust-strip__mode-btn {
  height: 28px;
  padding: 0 var(--space-3);
  background: rgba(0, 0, 0, 0.2);
  border: var(--border-width) solid var(--color-glass-border);
  border-radius: var(--radius-sm);
  color: var(--color-text-muted);
  font-size: var(--font-size-xs);
  font-weight: 600;
  font-family: var(--font-primary);
  text-transform: uppercase;
  letter-spacing: 0.05em;
  cursor: pointer;
  transition: all var(--transition-fast);
  white-space: nowrap;
}

.trust-strip__mode-btn:hover {
  background: rgba(91, 141, 239, 0.1);
  border-color: var(--color-glass-border-hover);
  color: var(--color-text-primary);
}

.trust-strip__mode-btn--active {
  background: rgba(91, 141, 239, 0.2);
  border-color: var(--color-accent-blue);
  color: var(--color-accent-blue);
}

/* ===== Alpha slider ===== */
.trust-strip__alpha {
  display: flex;
  align-items: center;
  gap: var(--space-1);
  margin-left: var(--space-2);
}

.trust-strip__alpha-label {
  font-size: var(--font-size-xs);
  font-family: var(--font-mono);
  color: var(--color-text-muted);
}

.trust-strip__alpha-slider {
  width: 80px;
  height: 4px;
  -webkit-appearance: none;
  appearance: none;
  background: var(--color-glass-border);
  border-radius: 2px;
  outline: none;
}

.trust-strip__alpha-slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  width: 14px;
  height: 14px;
  border-radius: 50%;
  background: var(--color-accent-blue);
  cursor: pointer;
  border: 2px solid var(--color-bg-primary);
}

.trust-strip__alpha-value {
  font-size: var(--font-size-xs);
  font-family: var(--font-mono);
  color: var(--color-accent-blue);
  min-width: 32px;
  text-align: right;
}
```

**Step 3: Build and verify**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`

Expected: Build succeeds.

**Step 4: Commit**

```bash
git add src/client/components/TrustStrip.jsx src/client/styles/TrustStrip.css
git commit -m "feat: add mode control buttons, alpha slider, and connection indicator to TrustStrip"
```

---

## Task 6: Create SimTelemetryPane component

Dense flight-instrument-style panel showing sim observations, rewards, velocity commands, and robot state.

**Files:**
- Create: `src/client/components/SimTelemetryPane.jsx`
- Create: `src/client/styles/SimTelemetryPane.css`

**Step 1: Create SimTelemetryPane.jsx**

```jsx
import React from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/SimTelemetryPane.css';

const SPARKLINE_CHARS = '▁▂▃▄▅▆▇█';

const sparkline = (values, min = -1, max = 1) => {
  if (!values || values.length === 0) return '';
  const range = max - min || 1;
  return values.map(v => {
    const idx = Math.round(((v - min) / range) * (SPARKLINE_CHARS.length - 1));
    return SPARKLINE_CHARS[Math.max(0, Math.min(SPARKLINE_CHARS.length - 1, idx))];
  }).join('');
};

const MiniBar = ({ value, min = 0, max = 1, color = 'var(--color-accent-blue)' }) => {
  const pct = Math.max(0, Math.min(100, ((value - min) / (max - min || 1)) * 100));
  const isNeg = value < 0;
  return (
    <div className="sim-tel__minibar">
      <div
        className="sim-tel__minibar-fill"
        style={{
          width: `${Math.abs(pct)}%`,
          background: isNeg ? 'var(--color-accent-red)' : color,
          marginLeft: isNeg ? 'auto' : 0,
        }}
      />
    </div>
  );
};

const Vec3Row = ({ label, values }) => {
  if (!values || values.length < 3) return null;
  return (
    <div className="sim-tel__vec3-row">
      <span className="sim-tel__obs-label">{label}</span>
      <span className="sim-tel__obs-val">{values[0].toFixed(2)}</span>
      <span className="sim-tel__obs-val">{values[1].toFixed(2)}</span>
      <span className="sim-tel__obs-val">{values[2].toFixed(2)}</span>
    </div>
  );
};

const HighDimRow = ({ label, values, count }) => {
  if (!values || values.length === 0) return null;
  return (
    <div className="sim-tel__highdim-row">
      <span className="sim-tel__obs-label">{label}</span>
      <span className="sim-tel__obs-count">({count})</span>
      <span className="sim-tel__sparkline">{sparkline(values, -2, 2)}</span>
    </div>
  );
};

const SimTelemetryPane = ({ compact = false }) => {
  const {
    trainingMetrics,
    obsBreakdown,
    rewardBreakdown,
    velocityCommand,
  } = useGenesis();

  const step = trainingMetrics?.step_count ?? 0;
  const episode = trainingMetrics?.episode_count ?? 0;
  const fps = trainingMetrics?.fps ?? 0;
  const totalReward = trainingMetrics?.total_reward ?? 0;

  if (compact) {
    return (
      <div className="sim-tel sim-tel--compact">
        <div className="sim-tel__strip-row">
          <span className="sim-tel__strip-label">SIM</span>
          <span className="sim-tel__strip-val">EP {episode}</span>
          <span className="sim-tel__strip-val">S {step.toLocaleString()}</span>
          <span className="sim-tel__strip-val">R {totalReward.toFixed(1)}</span>
          <span className="sim-tel__strip-val">{fps.toFixed(0)} FPS</span>
        </div>
      </div>
    );
  }

  const obs = obsBreakdown || trainingMetrics?.obs_breakdown;
  const rew = rewardBreakdown || trainingMetrics?.reward_breakdown;
  const velCmd = velocityCommand || trainingMetrics?.velocity_command;

  return (
    <div className="sim-tel">
      {/* Header stats */}
      <div className="sim-tel__header">
        <span>EP {episode}</span>
        <span>STEP {step.toLocaleString()}</span>
        <span>{fps.toFixed(0)} FPS</span>
      </div>

      {/* Velocity Command */}
      {velCmd && (
        <div className="sim-tel__section">
          <div className="sim-tel__section-title">VELOCITY CMD</div>
          <div className="sim-tel__vel-row">
            <span className="sim-tel__vel-label">Vx</span>
            <MiniBar value={velCmd.lin_vel_x || 0} min={-1} max={1} />
            <span className="sim-tel__vel-val">{(velCmd.lin_vel_x || 0).toFixed(2)} m/s</span>
          </div>
          <div className="sim-tel__vel-row">
            <span className="sim-tel__vel-label">Vy</span>
            <MiniBar value={velCmd.lin_vel_y || 0} min={-0.5} max={0.5} />
            <span className="sim-tel__vel-val">{(velCmd.lin_vel_y || 0).toFixed(2)} m/s</span>
          </div>
          <div className="sim-tel__vel-row">
            <span className="sim-tel__vel-label">ωz</span>
            <MiniBar value={velCmd.ang_vel_z || 0} min={-1} max={1} />
            <span className="sim-tel__vel-val">{(velCmd.ang_vel_z || 0).toFixed(2)} r/s</span>
          </div>
        </div>
      )}

      {/* Observations */}
      {obs && (
        <div className="sim-tel__section">
          <div className="sim-tel__section-title">
            OBSERVATIONS
            <span className="sim-tel__dim-badge">48 dim</span>
          </div>
          <Vec3Row label="Ang Vel" values={obs.angular_velocity} />
          <Vec3Row label="Lin Vel" values={obs.linear_velocity} />
          <Vec3Row label="Gravity" values={obs.projected_gravity} />
          <Vec3Row label="Vel Cmd" values={obs.velocity_command} />
          <HighDimRow label="DoF Pos" values={obs.dof_position} count={12} />
          <HighDimRow label="DoF Vel" values={obs.dof_velocity} count={12} />
          <HighDimRow label="Actions" values={obs.actions} count={12} />
        </div>
      )}

      {/* Rewards */}
      {rew && (
        <div className="sim-tel__section">
          <div className="sim-tel__section-title">
            REWARDS
            <span className="sim-tel__reward-total">Σ = {totalReward.toFixed(1)}</span>
          </div>
          {Object.entries(rew).map(([name, value]) => (
            <div key={name} className="sim-tel__reward-row">
              <span className="sim-tel__reward-name">{name}</span>
              <MiniBar value={value} min={-1} max={1} color={value >= 0 ? 'var(--color-accent-green)' : 'var(--color-accent-red)'} />
              <span className="sim-tel__reward-val">{value.toFixed(3)}</span>
            </div>
          ))}
        </div>
      )}

      {/* Robot State */}
      <div className="sim-tel__section">
        <div className="sim-tel__section-title">ROBOT STATE</div>
        <div className="sim-tel__state-grid">
          <span className="sim-tel__state-label">Reward</span>
          <span className="sim-tel__state-val">{totalReward.toFixed(2)}</span>
          <span className="sim-tel__state-label">Episode</span>
          <span className="sim-tel__state-val">{episode}</span>
        </div>
      </div>
    </div>
  );
};

export default SimTelemetryPane;
```

**Step 2: Create SimTelemetryPane.css**

```css
/* ===== SimTelemetryPane ===== */
.sim-tel {
  display: flex;
  flex-direction: column;
  font-family: var(--font-mono);
  font-size: var(--font-size-xs);
  color: var(--color-text-primary);
  overflow-y: auto;
  flex: 1;
}

.sim-tel--compact {
  flex: 0 0 auto;
  padding: var(--space-2) var(--space-3);
  background: rgba(0, 0, 0, 0.1);
  border-bottom: 1px solid var(--color-glass-border);
}

.sim-tel__strip-row {
  display: flex;
  align-items: center;
  gap: var(--space-3);
}

.sim-tel__strip-label {
  font-weight: 700;
  color: var(--color-accent-blue);
  text-transform: uppercase;
}

.sim-tel__strip-val {
  color: var(--color-text-secondary);
}

/* Header */
.sim-tel__header {
  display: flex;
  justify-content: space-between;
  padding: var(--space-2) var(--space-3);
  border-bottom: 1px solid var(--color-glass-border);
  color: var(--color-text-secondary);
  font-weight: 600;
}

/* Sections */
.sim-tel__section {
  padding: var(--space-2) var(--space-3);
  border-bottom: 1px solid rgba(255, 255, 255, 0.04);
}

.sim-tel__section-title {
  font-weight: 700;
  color: var(--color-text-muted);
  text-transform: uppercase;
  letter-spacing: 0.05em;
  margin-bottom: var(--space-2);
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.sim-tel__dim-badge {
  font-weight: 400;
  color: var(--color-text-muted);
  font-size: 10px;
}

.sim-tel__reward-total {
  font-weight: 400;
  color: var(--color-accent-blue);
}

/* Velocity rows */
.sim-tel__vel-row {
  display: grid;
  grid-template-columns: 24px 1fr 72px;
  align-items: center;
  gap: var(--space-2);
  margin-bottom: 2px;
}

.sim-tel__vel-label {
  color: var(--color-text-muted);
  font-weight: 600;
}

.sim-tel__vel-val {
  text-align: right;
  color: var(--color-text-primary);
}

/* Mini bar */
.sim-tel__minibar {
  height: 6px;
  background: rgba(0, 0, 0, 0.3);
  border-radius: 3px;
  overflow: hidden;
}

.sim-tel__minibar-fill {
  height: 100%;
  border-radius: 3px;
  transition: width 0.1s ease;
}

/* Observation rows */
.sim-tel__vec3-row {
  display: grid;
  grid-template-columns: 56px repeat(3, 1fr);
  gap: var(--space-1);
  margin-bottom: 2px;
}

.sim-tel__obs-label {
  color: var(--color-text-muted);
  font-weight: 600;
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.sim-tel__obs-val {
  text-align: right;
  color: var(--color-text-primary);
}

.sim-tel__obs-count {
  color: var(--color-text-muted);
  font-size: 10px;
}

.sim-tel__highdim-row {
  display: grid;
  grid-template-columns: 56px 28px 1fr;
  gap: var(--space-1);
  margin-bottom: 2px;
  align-items: center;
}

.sim-tel__sparkline {
  color: var(--color-accent-blue);
  letter-spacing: -1px;
  overflow: hidden;
}

/* Reward rows */
.sim-tel__reward-row {
  display: grid;
  grid-template-columns: 80px 1fr 56px;
  align-items: center;
  gap: var(--space-2);
  margin-bottom: 2px;
}

.sim-tel__reward-name {
  color: var(--color-text-muted);
  white-space: nowrap;
  overflow: hidden;
  text-overflow: ellipsis;
}

.sim-tel__reward-val {
  text-align: right;
  color: var(--color-text-primary);
}

/* State grid */
.sim-tel__state-grid {
  display: grid;
  grid-template-columns: 1fr 1fr 1fr 1fr;
  gap: var(--space-1);
}

.sim-tel__state-label {
  color: var(--color-text-muted);
}

.sim-tel__state-val {
  color: var(--color-text-primary);
  font-weight: 600;
}
```

**Step 3: Build and verify**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`

Expected: Build succeeds.

**Step 4: Commit**

```bash
git add src/client/components/SimTelemetryPane.jsx src/client/styles/SimTelemetryPane.css
git commit -m "feat: add dense SimTelemetryPane component with obs/reward/velocity display"
```

---

## Task 7: Create TelemetryConsole with preset-based layouts

Replace the right EdgePanel content with a TelemetryConsole that auto-switches between Operate (ROS-first), Sim (SIM-first), and Split presets.

**Files:**
- Create: `src/client/components/TelemetryConsole.jsx`
- Create: `src/client/styles/TelemetryConsole.css`
- Modify: `src/client/components/App.jsx:201-221`

**Step 1: Create TelemetryConsole.jsx**

```jsx
import React, { useState, useEffect, useCallback } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import SimTelemetryPane from './SimTelemetryPane';
import TelemetryPanel from './TelemetryPanel';
import '../styles/TelemetryConsole.css';

const PRESETS = {
  operate: { label: 'OPERATE', desc: 'ROS-first' },
  sim: { label: 'SIM', desc: 'Simulation-first' },
  split: { label: 'SPLIT', desc: 'Side-by-side' },
};

const TelemetryHeader = ({ rosConnected, genesisConnected, rosFps, simFps }) => (
  <div className="tel-console__header">
    <div className="tel-console__conns">
      <span className={`tel-console__conn ${genesisConnected ? 'tel-console__conn--on' : ''}`}>
        SIM {genesisConnected ? `${simFps}fps` : ''}
      </span>
      <span className={`tel-console__conn ${rosConnected ? 'tel-console__conn--on' : ''}`}>
        ROS {rosConnected ? `${rosFps}Hz` : ''}
      </span>
    </div>
  </div>
);

const TelemetryConsole = ({ ros, rosConnected, appSettings }) => {
  const { genesisConnected, genesisMode, blendAlpha, trainingMetrics } = useGenesis();

  const [activePreset, setActivePreset] = useState('sim');
  const [pinned, setPinned] = useState(false);

  // Auto-switch preset based on connection state
  useEffect(() => {
    if (pinned) return;

    if (rosConnected && genesisConnected) {
      const isBlend = genesisMode === 'hil_blend' || genesisMode === 'blend' || genesisMode === 'online_finetune';
      setActivePreset(isBlend ? 'split' : 'sim');
    } else if (genesisConnected) {
      setActivePreset('sim');
    } else if (rosConnected) {
      setActivePreset('operate');
    } else {
      setActivePreset('sim');
    }
  }, [rosConnected, genesisConnected, genesisMode, pinned]);

  const handlePresetClick = useCallback((preset) => {
    setActivePreset(preset);
    setPinned(true);
  }, []);

  const handleUnpin = useCallback(() => {
    setPinned(false);
  }, []);

  const simFps = trainingMetrics?.fps?.toFixed(0) ?? '0';
  const rosFps = '10'; // ROS default rate

  return (
    <div className="tel-console">
      <TelemetryHeader
        rosConnected={rosConnected}
        genesisConnected={genesisConnected}
        rosFps={rosFps}
        simFps={simFps}
      />

      {/* Preset switcher */}
      <div className="tel-console__presets">
        {Object.entries(PRESETS).map(([key, { label }]) => (
          <button
            key={key}
            className={`tel-console__preset-btn ${activePreset === key ? 'tel-console__preset-btn--active' : ''}`}
            onClick={() => handlePresetClick(key)}
          >
            {label}
          </button>
        ))}
        {pinned && (
          <button className="tel-console__unpin-btn" onClick={handleUnpin} title="Auto-switch">
            ⊗
          </button>
        )}
      </div>

      {/* Layout content */}
      <div className="tel-console__content">
        {activePreset === 'operate' && (
          <>
            <div className="tel-console__primary">
              <TelemetryPanel
                ros={ros}
                updateInterval={appSettings?.telemetry?.updateInterval ?? 100}
                initialShowPanel={true}
              />
            </div>
            {genesisConnected && (
              <SimTelemetryPane compact={true} />
            )}
          </>
        )}

        {activePreset === 'sim' && (
          <>
            <div className="tel-console__primary">
              <SimTelemetryPane />
            </div>
            {rosConnected && (
              <div className="tel-console__strip">
                <span className="tel-console__strip-label">ROS</span>
                <span className={`tel-console__conn ${rosConnected ? 'tel-console__conn--on' : ''}`}>
                  Connected
                </span>
              </div>
            )}
          </>
        )}

        {activePreset === 'split' && (
          <div className="tel-console__split">
            <div className="tel-console__split-pane">
              <div className="tel-console__split-title">SIM</div>
              <SimTelemetryPane />
            </div>
            <div className="tel-console__split-divider" />
            <div className="tel-console__split-pane">
              <div className="tel-console__split-title">ROS</div>
              <TelemetryPanel
                ros={ros}
                updateInterval={appSettings?.telemetry?.updateInterval ?? 100}
                initialShowPanel={true}
              />
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default TelemetryConsole;
```

**Step 2: Create TelemetryConsole.css**

```css
/* ===== TelemetryConsole ===== */
.tel-console {
  display: flex;
  flex-direction: column;
  height: 100%;
  overflow: hidden;
}

/* Header */
.tel-console__header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: var(--space-2) var(--space-3);
  border-bottom: 1px solid var(--color-glass-border);
  flex-shrink: 0;
}

.tel-console__conns {
  display: flex;
  gap: var(--space-3);
}

.tel-console__conn {
  font-size: var(--font-size-xs);
  font-family: var(--font-mono);
  color: var(--color-text-muted);
}

.tel-console__conn--on {
  color: var(--color-accent-green);
}

/* Preset switcher */
.tel-console__presets {
  display: flex;
  gap: 1px;
  padding: var(--space-1) var(--space-3);
  border-bottom: 1px solid var(--color-glass-border);
  flex-shrink: 0;
}

.tel-console__preset-btn {
  flex: 1;
  height: 26px;
  background: rgba(0, 0, 0, 0.15);
  border: 1px solid var(--color-glass-border);
  border-radius: var(--radius-sm);
  color: var(--color-text-muted);
  font-size: 10px;
  font-weight: 700;
  font-family: var(--font-primary);
  text-transform: uppercase;
  letter-spacing: 0.05em;
  cursor: pointer;
  transition: all var(--transition-fast);
}

.tel-console__preset-btn:hover {
  background: rgba(91, 141, 239, 0.08);
  color: var(--color-text-primary);
}

.tel-console__preset-btn--active {
  background: rgba(91, 141, 239, 0.15);
  border-color: var(--color-accent-blue);
  color: var(--color-accent-blue);
}

.tel-console__unpin-btn {
  width: 26px;
  height: 26px;
  background: none;
  border: 1px solid var(--color-glass-border);
  border-radius: var(--radius-sm);
  color: var(--color-text-muted);
  font-size: 12px;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all var(--transition-fast);
  flex-shrink: 0;
  margin-left: var(--space-1);
}

.tel-console__unpin-btn:hover {
  border-color: var(--color-accent-amber);
  color: var(--color-accent-amber);
}

/* Content area */
.tel-console__content {
  flex: 1;
  overflow-y: auto;
  overflow-x: hidden;
  display: flex;
  flex-direction: column;
}

.tel-console__primary {
  flex: 1;
  overflow-y: auto;
}

/* Strip (compact secondary) */
.tel-console__strip {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  padding: var(--space-2) var(--space-3);
  border-top: 1px solid var(--color-glass-border);
  background: rgba(0, 0, 0, 0.08);
  flex-shrink: 0;
}

.tel-console__strip-label {
  font-size: var(--font-size-xs);
  font-weight: 700;
  color: var(--color-text-muted);
  text-transform: uppercase;
}

/* Split layout */
.tel-console__split {
  flex: 1;
  display: flex;
  flex-direction: column;
  overflow: hidden;
}

.tel-console__split-pane {
  flex: 1;
  overflow-y: auto;
  min-height: 0;
}

.tel-console__split-title {
  padding: var(--space-1) var(--space-3);
  font-size: 10px;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--color-text-muted);
  background: rgba(0, 0, 0, 0.1);
  border-bottom: 1px solid var(--color-glass-border);
  flex-shrink: 0;
}

.tel-console__split-divider {
  height: 1px;
  background: var(--color-glass-border);
  flex-shrink: 0;
}
```

**Step 3: Wire TelemetryConsole into App.jsx**

In `App.jsx`, replace the right EdgePanel content (lines 201-221).

Replace:
```jsx
{/* Show Genesis info when connected, otherwise ROS telemetry */}
{genesisConnected ? (
  <GenesisInfoPanel />
) : (
  <TelemetryPanel
    ros={ros}
    updateInterval={appSettings?.telemetry?.updateInterval ?? 100}
    initialShowPanel={true}
  />
)}
```

With:
```jsx
<TelemetryConsole
  ros={ros}
  rosConnected={rosConnected}
  appSettings={appSettings}
/>
```

Add the import at the top of App.jsx:
```jsx
import TelemetryConsole from './TelemetryConsole';
```

Remove the now-unused `GenesisInfoPanel` import (keep `TelemetryPanel` since `TelemetryConsole` uses it).

**Step 4: Build and verify**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`

Expected: Build succeeds.

**Step 5: Commit**

```bash
git add src/client/components/TelemetryConsole.jsx src/client/styles/TelemetryConsole.css src/client/components/App.jsx
git commit -m "feat: add TelemetryConsole with preset-based layouts, replace GenesisInfoPanel in right panel"
```

---

## Task 8: Add estop handler to bridge_server.py

Wire the E-Stop Socket.io event to immediately pause simulation and zero actions.

**Files:**
- Modify: `genesis_bridge/bridge_server.py`

**Step 1: Add estop handler in Socket.io setup**

After the `genesis_set_alpha` handler (around line 1317), add:

```python
@self.sio.on('genesis_estop')
async def on_estop(data=None):
    logger.warning("E-STOP triggered from UI")
    self.paused = True
    self.auto_stopped = True
    if self.forge_env is not None:
        # Zero out velocity commands
        self.forge_env.set_velocity_from_gamepad({
            'leftStickX': 0, 'leftStickY': 0,
            'rightStickX': 0, 'rightStickY': 0,
        })
    else:
        self.router.force_alpha_zero(reason="estop")
    # Emit status update
    await self.sio.emit('genesis_training_metrics', self.get_training_metrics())
```

**Step 2: Add estop to GENESIS_EVENTS in server.js (already done in Task 3)**

Verify `genesis_estop` is in the GENESIS_EVENTS array.

**Step 3: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat: add E-Stop handler to bridge server"
```

---

## Task 9: End-to-end verification

Verify the full stack works together.

**Files:** None (verification only)

**Step 1: Build frontend**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`

Expected: Build succeeds with no errors.

**Step 2: Verify Python imports**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -c "from genesis_bridge.envs.go2_env import Go2BridgeEnv, OBS_GROUPS; print(f'OBS_GROUPS: {len(OBS_GROUPS)} groups, {sum(s for _, s in OBS_GROUPS)} dims'); print('OK')"`

Expected: `OBS_GROUPS: 7 groups, 48 dims` and `OK`

**Step 3: Verify bridge_server.py syntax**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && python -m py_compile genesis_bridge/bridge_server.py && echo "OK"`

Expected: `OK`

**Step 4: Verify server.js syntax**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && node -c server.js && echo "OK"`

Expected: `OK`

**Step 5: Start server and check UI loads**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm start &`

Then in a browser (or curl): `curl -s http://localhost:3000 | head -5`

Expected: HTML response with the app.

**Step 6: Commit verification**

```bash
git add -A
git status
# Verify only expected files are staged
git commit -m "chore: end-to-end verification of genesis-forge integration"
```

---

## Summary

| Task | Component | Files |
|------|-----------|-------|
| 1 | Go2BridgeEnv | `genesis_bridge/envs/go2_env.py`, `genesis_bridge/envs/__init__.py` |
| 2 | bridge_server.py adaptation | `genesis_bridge/bridge_server.py` |
| 3 | server.js relay events | `server.js` |
| 4 | GenesisContext new state | `src/client/contexts/GenesisContext.jsx` |
| 5 | TrustStrip mode controls | `src/client/components/TrustStrip.jsx`, `src/client/styles/TrustStrip.css` |
| 6 | SimTelemetryPane | `src/client/components/SimTelemetryPane.jsx`, `src/client/styles/SimTelemetryPane.css` |
| 7 | TelemetryConsole + App wiring | `src/client/components/TelemetryConsole.jsx`, `src/client/styles/TelemetryConsole.css`, `src/client/components/App.jsx` |
| 8 | E-Stop handler | `genesis_bridge/bridge_server.py` |
| 9 | End-to-end verification | (verification only) |
