# Go2 Velocity Control: Training, ROS Integration & Safety

## Problem

The Go2 forge environment receives gamepad velocity commands via `VelocityCommandManager`, but `_step_forge_env()` passes **zero actions** to `env.step()`. Without a trained locomotion policy generating joint targets from observations (which include velocity commands), the robot is inert. Additionally, there is no ROS `/cmd_vel` input path, deadman/safety enforcement is bypassed, and there is no observability into the control loop.

## Architecture

### Data Flow (Target State)

```
[Gamepad]                          [ROS /cmd_vel]
    |                                    |
    v                                    v
Socket.io                         rclpy subscriber
    |                                    |
    +--- command_source gate ------------+
    |           |
    v           v
    (active)  (dropped)
    |
    v
VelocityCommandManager.set_command()
    |
    v
forge_env.get_observations()  →  48-dim tensor (includes velocity_command at dims 9-11)
    |
    v
policy(obs)  →  12-dim joint actions
    |
    v
forge_env.step(actions)  →  Go2 moves
```

Gamepad also publishes to ROS `/cmd_vel` for external visibility (when gamepad is the active source).

### Command Source Switching

A `command_source` field (`"gamepad"` | `"ros"`) determines which input reaches `VelocityCommandManager`. Switchable via:

- **Socket.io event**: `genesis_set_command_source` with payload `"gamepad"` or `"ros"`
- **ROS2 service**: `/set_command_source`

On switch, velocity commands are zeroed to prevent stale carryover. Default is `"gamepad"` on startup. The inactive source's inputs are silently dropped.

### ROS in Docker

ROS2 runs in a Docker container for this repo. The bridge's `rclpy` node runs inside the Docker container with DDS discovery configured. WebSocket communication (Socket.io, Genesis bridge) crosses the Docker boundary; ROS communication stays within the ROS Docker network.

## Step 1: Training Pipeline

Train a Go2 locomotion policy using the genesis-forge gait_trainer example.

### Training Script

Create `rl/scripts/train_go2_locomotion.py`:

- Initialize Genesis GPU backend (`gs.init(backend=gs.gpu, performance_mode=True)`)
- Create `Go2GaitTrainingEnv(num_envs=4096, headless=True)` using the gait_trainer environment (richer rewards: gait phase, foot height, contact penalties)
- Wrap with `RslRlWrapper`
- Run `OnPolicyRunner` with PPO config:
  - Actor/critic hidden dims: `[512, 256, 128]`
  - Learning rate: 0.001
  - Mini-batches: 4
  - Gamma: 0.99, lambda: 0.95
  - Clip: 0.2, entropy coef: 0.01
  - ~2000 iterations
- Output: `rl/checkpoints/go2_locomotion/model_XXXX.pt` + `cfgs.pkl`

### Eval Script

Create `rl/scripts/eval_go2_locomotion.py`:

- Load checkpoint and config
- Create env with `num_envs=1, headless=False`
- Run inference loop to visually verify walking gait

### Reference

- Training example: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/examples/gait_trainer/train.py`
- Eval example: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/examples/gait_trainer/eval.py`
- Environment: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/examples/gait_trainer/environment.py`

## Step 2: Policy Loading in Bridge

Modify `bridge_server.py` to load a trained policy and use it in `_step_forge_env()`.

### Changes to `bridge_server.py`

- Add `load_policy(checkpoint_path)` method:
  - Creates `OnPolicyRunner`, calls `runner.load(model)`, extracts `runner.get_inference_policy(device)`
  - Called once at startup (configurable checkpoint path via CLI arg or config, default `rl/checkpoints/go2_locomotion/latest.pt`)
  - Can also be triggered via Socket.io event for hot-swapping policies

- Modify `_step_forge_env()`:
  ```python
  # Before (current):
  actions = torch.zeros((num_envs, num_actions), device=gs.device)

  # After:
  if self.policy is not None:
      obs = self.forge_env.get_observations()
      with torch.no_grad():
          actions = self.policy(obs)
  else:
      actions = torch.zeros((num_envs, num_actions), device=gs.device)
  ```

- Fallback: if no checkpoint loaded, keep zero-action behavior (robot stands).

## Step 3: Dual Input with Source Switching

### Gamepad Path (existing, extended)

- Socket.io `controller_joystick_state` → `set_velocity_from_gamepad()` → `VelocityCommandManager.set_command()`
- **New**: Also publishes to ROS `/cmd_vel` topic (~30Hz throttled). Only publishes when `command_source == "gamepad"` to avoid echo loops.
- Only writes to `VelocityCommandManager` if `command_source == "gamepad"`.

### ROS Path (new)

- `rclpy` node subscribes to `/cmd_vel` (geometry_msgs/Twist)
- Callback maps:
  - `twist.linear.x` → `lin_vel_x` (clamped to [-1.0, 1.0])
  - `twist.linear.y` → `lin_vel_y` (clamped to [-0.5, 0.5])
  - `twist.angular.z` → `ang_vel_z` (clamped to [-1.0, 1.0])
- Only writes to `VelocityCommandManager` if `command_source == "ros"`.
- Optional dependency: if `rclpy` unavailable, bridge logs warning and runs gamepad-only.

### Source Switching

- **Socket.io**: `genesis_set_command_source` event with `"gamepad"` or `"ros"` payload
- **ROS2 service**: `/set_command_source` (callable from ROS side, e.g., `ros2 service call`)
- On switch: velocity commands zeroed, `last_command_source` updated
- Default: `"gamepad"` on startup

### Docker

ROS2 runs in Docker. Bridge's `rclpy` node runs inside the container with DDS discovery. WebSocket traffic crosses the Docker boundary.

## Step 4: Deadman Switch & Safety

### Deadman (L1)

- **Gamepad mode**: L1 must be held for velocity commands to pass through. Bridge tracks L1 state from `controller_button_states`. On L1 release, velocity commands zeroed.
- **ROS mode**: Deadman not enforced (ROS callers handle their own safety).

### E-STOP

Already implemented. Zeroes velocity and pauses sim. Works regardless of command source. No changes needed.

### Policy Gating

Safety lives at the velocity command layer. Zero velocity commands → policy produces standing pose (trained to track commands). No separate action-layer gating needed.

### Anomaly Detection

Wire existing `AnomalyDetector` into `_step_forge_env()`. On anomaly trigger: zero velocity commands, pause sim (same as E-STOP).

## Step 5: Observability

### Socket.io Status (extend `genesis_status`)

- `command_source`: `"gamepad"` | `"ros"`
- `velocity_command`: `{lin_vel_x, lin_vel_y, ang_vel_z}`
- `deadman_active`: boolean
- `policy_loaded`: boolean
- `policy_checkpoint`: filename or null

### ROS Debug Topic

Publish to `/go2_debug` (`std_msgs/String` with JSON) at sim step rate (~50Hz). Contains same fields as Socket.io status.

### Logging

`logger.info` on source switches, policy load/unload, deadman state changes.

### Frontend

New status fields displayed in existing `SimpleStatusPanel` / `SimpleGenesisPanel` via the genesis status path.

## Test Matrix

| # | Test | Expected |
|---|------|----------|
| 1 | Training produces walking gait | Eval script shows Go2 walking, following velocity commands |
| 2 | Browser gamepad → Go2 moves | Push left stick forward, Go2 walks forward in sim |
| 3 | ROS `/cmd_vel` → Go2 moves | `ros2 topic pub /cmd_vel ...`, Go2 walks forward |
| 4 | Source switching | Switch via Socket.io and ROS service, clean handoff, correct source honored |
| 5 | Deadman released → standing | In gamepad mode, release L1, Go2 returns to standing |
| 6 | E-STOP → full stop | Sim pauses, velocity zeroes, regardless of command source |
| 7 | Gamepad publishes to `/cmd_vel` | `ros2 topic echo /cmd_vel` shows matching values |
| 8 | No policy fallback | Bridge starts without checkpoint, zero-action standing, no crashes |

## Key Files

| File | Changes |
|------|---------|
| `rl/scripts/train_go2_locomotion.py` | New — training script |
| `rl/scripts/eval_go2_locomotion.py` | New — eval script |
| `genesis_bridge/bridge_server.py` | `load_policy()`, `_step_forge_env()` policy inference, ROS node, command source switching, deadman enforcement, anomaly detection wiring, observability fields |
| `genesis_bridge/envs/go2_env.py` | `get_observations()` accessor if not already exposed |
| `server.js` | Forward `genesis_set_command_source` event |
| `src/client/components/SimpleGenesisPanel.jsx` | Display command source, velocity, deadman, policy status |
| `src/client/contexts/GenesisContext.jsx` | Handle new status fields |
