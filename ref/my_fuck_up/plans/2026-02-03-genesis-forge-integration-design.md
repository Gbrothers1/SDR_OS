# Genesis-Forge Integration Design

**Goal:** Replace SDR_OS's custom RL infrastructure with genesis-forge as the simulation/RL backbone, using Go2 quadruped as the proof-of-concept robot. Wire genesis-forge into the existing bridge_server.py for web UI streaming, and redesign the cockpit telemetry panel for dense, mode-aware data display.

**Architecture:** genesis-forge `ManagedEnvironment` runs inside `bridge_server.py`, replacing manual Genesis scene management. The existing JPEG streaming + Socket.io event system stays. A new Telemetry Console replaces the right panel with preset-based layouts (Operate/Sim/Split) that auto-switch based on connection state. TrustStrip gains pipeline mode controls (TELEOP/POLICY/BLEND).

**Proof-of-concept robot:** Unitree Go2 quadruped (URDF at `urdf/go2/urdf/go2.urdf`)

---

## 1. System Architecture

```
Browser (React)
  ├── TrustStrip ─── Mode control (TELEOP/POLICY/BLEND) + α slider + E-Stop
  ├── ViewerLayer/SimViewer ─── JPEG frames from bridge WebSocket
  ├── Left EdgePanel/ControlOverlay ─── Gamepad input → Socket.io
  ├── Right EdgePanel/TelemetryConsole ─── Mode-aware telemetry display
  └── Bottom EdgePanel/CommandBar ─── Text commands
        │
        ↕ Socket.io (port 3000)
        │
  Node.js Server (Express + Socket.io relay)
        │
        ↕ Socket.io (port 9091)
        │
  bridge_server.py
    └── genesis-forge ManagedEnvironment (Go2)
          ├── ObservationManager (angular_vel, linear_vel, gravity, dof_pos, dof_vel, actions)
          ├── PositionActionManager (scale=0.25, PD control)
          ├── VelocityCommandManager (gamepad → lin_vel_x/y, ang_vel_z)
          ├── RewardManager (height, tracking_lin_vel, tracking_ang_vel, lin_vel_z, action_rate, similar_to_default)
          ├── TerminationManager (timeout + fall detection)
          └── Camera (render → JPEG → WebSocket stream)
```

**Data flow per tick:**
1. Bridge calls `env.step(actions)` → returns `(obs, rewards, terminated, truncated, extras)`
2. Bridge renders `env.camera.render()` → cv2 JPEG encode → WebSocket binary frame
3. Bridge emits structured metrics via Socket.io: obs breakdown, reward terms, velocity command, episode info
4. Gamepad input arrives via Socket.io → bridge feeds into `VelocityCommandManager`

---

## 2. Bridge Server Changes

### 2.1 New: `genesis_bridge/envs/go2_env.py`

A `ManagedEnvironment` subclass configured for Go2 locomotion:

```python
class Go2BridgeEnv(ManagedEnvironment):
    def __init__(self, num_envs=1, dt=1/50, max_episode_length_s=20, headless=True):
        super().__init__(num_envs=num_envs, dt=dt, ...)

        # Scene setup
        self.scene = gs.Scene(...)
        self.terrain = self.scene.add_entity(gs.morphs.Plane())
        self.robot = self.scene.add_entity(gs.morphs.URDF(
            file="urdf/go2/urdf/go2.urdf",
            pos=[0.0, 0.0, 0.4],
            quat=[1.0, 0.0, 0.0, 0.0],
        ))
        self.camera = self.scene.add_camera(
            pos=(-2.5, -1.5, 1.0), lookat=(0.0, 0.0, 0.0),
            res=(1280, 720), fov=40, env_idx=0,
        )

    def config(self):
        # EntityManager, ActuatorManager, PositionActionManager,
        # VelocityCommandManager, RewardManager, TerminationManager,
        # ObservationManager — matching genesis-forge examples/simple/environment.py
        ...

    def build(self):
        super().build()
        self.camera.follow_entity(self.robot)
```

Key configuration details (from genesis-forge Go2 example):
- **Joint names:** `FL_*_joint`, `FR_*_joint`, `RL_*_joint`, `RR_*_joint`
- **Default positions:** hip=0.0, FL/FR_thigh=0.8, RL/RR_thigh=1.0, calf=-1.5
- **PD gains:** kp=20, kv=0.5
- **Action scale:** 0.25, clip=(-100, 100), use_default_offset=True
- **Velocity ranges:** lin_vel_x=[-1.0, 1.0], lin_vel_y=[-0.5, 0.5], ang_vel_z=[-1.0, 1.0]

### 2.2 bridge_server.py Modifications

Replace manual scene/robot/camera management:
- `initialize_genesis()` → Create `Go2BridgeEnv`, call `env.build()`
- `step_simulation()` → `obs, rewards, terminated, truncated, extras = self.env.step(actions)`
- `render_frame()` → `self.env.camera.render(rgb=True)` → cv2 JPEG encode
- `load_robot()` → Recreate env with different config (or swap env class)
- Gamepad input → `env.velocity_command_manager.set_command(lin_vel_x, lin_vel_y, ang_vel_z)`

### 2.3 New Socket.io Events (bridge → UI)

| Event | Payload | Purpose |
|-------|---------|---------|
| `genesis_obs_breakdown` | `{ angular_vel: [3], linear_vel: [3], projected_gravity: [3], dof_pos: [12], dof_vel: [12], actions: [12] }` | Per-group observation values |
| `genesis_reward_breakdown` | `{ base_height: float, tracking_lin_vel: float, ... }` | Per-term reward values |
| `genesis_velocity_command` | `{ lin_vel_x: float, lin_vel_y: float, ang_vel_z: float }` | Current velocity command |

Existing events (`genesis_training_metrics`, `genesis_frame`, `genesis_status`) continue unchanged.

---

## 3. TrustStrip Mode Controls

TrustStrip gains pipeline mode control alongside existing confidence/safety indicators.

```
┌────────────────────────────────────────────────────────────────┐
│ ◉ GENESIS Go2  │ [TELEOP] [POLICY] [BLEND]  α ═══●══ 0.7  🛑 │
└────────────────────────────────────────────────────────────────┘
```

**Layout (left to right):**
1. **Connection + robot** — Green/red dot, "GENESIS" or "ROS", robot name
2. **Mode buttons** — Three compact toggles. Active = blue highlight. Click to switch.
3. **Alpha slider** — Inline, only visible in BLEND mode. Range 0.0–1.0.
4. **E-Stop button** — Red, always visible.

**Gamepad mapping:**
- D-pad left/right → cycle modes
- D-pad up/down → adjust alpha (in BLEND mode)
- Select → E-Stop

**Events emitted:** `genesis_set_mode`, `genesis_set_alpha`, `genesis_estop` via GenesisContext.

---

## 4. Telemetry Console (Right EdgePanel)

### 4.1 Design Philosophy

No tabs. Use **layout presets** that match operator intent, with auto-switching based on connection state.

The right panel becomes a **Telemetry Console** with:
- A persistent annunciator **header** (never scrolls)
- A **layout switcher** with three presets
- A **focus cursor** for cross-source metric highlighting

### 4.2 Component Tree

```
RightTelemetryConsole
├── TelemetryHeader (persistent annunciator — always visible)
│   ├── Connections: SIM ● / ROS ●
│   ├── Robot IDs: Go2 (sim) / Go2 (real)
│   └── Safety: Falls, Faults, Watchdog, Latency
│
├── PresetSwitcher (small button row or auto)
│
├── Layout Presets:
│   ├── Preset A: OPERATE (ROS-first)
│   │   ├── RosTelemetryPane (full — existing TelemetryPanel code)
│   │   └── SimStripPane (compact: authority, α, key metrics)
│   │
│   ├── Preset B: SIM/TRAIN (SIM-first)
│   │   ├── SimTelemetryPane (full dense telemetry)
│   │   └── RosStripPane (compact: connection health, key sensors)
│   │
│   └── Preset C: SPLIT/HIL (50/50)
│       ├── TelemetryDiffBar (sim vs real comparison)
│       ├── SimTelemetryPane (half height)
│       └── RosTelemetryPane (half height)
│
└── FocusCursor (cross-source highlight state)
```

### 4.3 Auto-Switching Rules

| Condition | Preset | Rationale |
|-----------|--------|-----------|
| Only ROS connected | Operate | Driving real hardware, need sensor verification |
| Only SIM connected | Sim/Train | Tuning rewards, debugging obs, evaluating policy |
| Both + BLEND/HIL mode | Split | Need side-by-side comparison |
| Both + TELEOP/POLICY | Sim/Train | Sim is primary, ROS strip for health |

User can **pin** a preset to prevent auto-switching during a session.

### 4.4 TelemetryHeader (Annunciator)

Aircraft-style persistent status line. Shows only metadata — no scrollable data.

```
┌───────────────────────────────────────┐
│ SIM ● 30fps   ROS ● 10Hz             │
│ Go2 (sim)     Go2 (real)             │
│ Falls: 0  Faults: 0  WD: OK  Δ: 3ms │
└───────────────────────────────────────┘
```

- Connection dots: green=connected, red=disconnected, gray=N/A
- FPS/Hz: data rate for each source
- Safety row: fall count, fault count, watchdog status, latency

### 4.5 SimTelemetryPane (Dense Telemetry)

Flight-instrument style. Monospace, compact, all values visible simultaneously.

```
┌─ SIM TELEMETRY ──────────────────────┐
│ EP 12    STEP 1,847         30 FPS   │
│ ─────────────────────────────────────│
│ VELOCITY CMD                         │
│  Vx  ▓▓▓▓░░░░  0.50 m/s            │
│  Vy  ░░░░░░░░  0.00 m/s            │
│  ωz  ▓▓░░░░░░  0.10 rad/s          │
│ ─────────────────────────────────────│
│ OBSERVATIONS              [48 dim]   │
│  Ang Vel   -0.02  0.01  0.15        │
│  Lin Vel    0.48  0.00  0.01        │
│  Gravity    0.00  0.00 -9.81        │
│  DoF Pos   (12) ▁▂▃▄▅▆▅▄▃▂▁▂      │
│  DoF Vel   (12) ▁▁▁▂▁▁▁▁▁▁▁▁      │
│  Actions   (12) ▃▃▃▃▃▃▃▃▃▃▃▃      │
│ ─────────────────────────────────────│
│ REWARDS               Σ = 14.7      │
│  height    ████░░  0.12             │
│  track_lv  ████████  0.85           │
│  track_av  ██████░  0.30            │
│  lin_vel_z ░░░░░░  -0.02           │
│  act_rate  ███░░░  0.08            │
│  default   █████░  0.21            │
│ ─────────────────────────────────────│
│ ROBOT STATE                          │
│  Height  0.34m  │  Fallen  NO       │
│  Uptime  23.4s  │  Resets  2        │
└──────────────────────────────────────┘
```

Design elements:
- **Monospace numbers** — right-aligned for scanning
- **Mini bar charts** — horizontal fill for rewards, velocity commands
- **Unicode sparklines** — `▁▂▃▄▅▆▇█` for high-dim groups (12 DoF values)
- **Color coding** — positive rewards green, negative red; velocity active blue, zero gray; fallen/anomaly red
- **Section headers** — uppercase, dimmed, thin separators

### 4.6 Strip Panes (Compact Secondary View)

When a source is secondary (e.g., SIM strip in Operate mode), show only:
- Connection status + data rate
- Current authority/mode + α
- 2–3 key metrics (total reward, episode, step count for SIM; IMU health, joint count for ROS)

Fits in ~60px height.

### 4.7 TelemetryDiffBar (Split Mode Only)

Side-by-side comparison of paired metrics:

```
┌─ DIFF ──────────────────────────────────┐
│ Base Orient   SIM: 0.02°  ROS: 0.03°  Δ: 0.01° │
│ Joint Pos     SIM: [12]   ROS: [12]    RMS: 0.04 │
│ Base Vel      CMD: 0.50   MEAS: 0.48   Δ: 0.02  │
│ Latency       SIM→UI: 3ms ROS→UI: 8ms            │
└─────────────────────────────────────────┘
```

### 4.8 Focus Cursor

Click or d-pad-select any metric → highlights the same concept across all visible panes.

Example: select "base angular velocity" → lights up in SIM obs, ROS IMU, and DIFF row (if split).

Implementation: shared state in `RightTelemetryConsole` that panes subscribe to. Each metric has a semantic key (e.g., `base_angular_velocity`) that maps across sources.

---

## 5. What Gets Replaced in SDR_OS

With genesis-forge as the RL backbone, these SDR_OS files become obsolete:

| SDR_OS file | Replaced by genesis-forge |
|-------------|--------------------------|
| `rl/actions/action_spec.py` | `BaseActionManager` + `PositionActionManager` |
| `rl/actions/action_router.py` | `ManagedEnvironment.step()` action routing |
| `rl/actions/teleop_bridge.py` | `VelocityCommandManager.use_gamepad()` |
| `rl/actions/cartesian_controller.py` | genesis-forge action managers |
| `rl/actions/joint_controller.py` | `ActuatorManager` + `PositionActionManager` |
| `rl/envs/genesis_vecenv.py` | `ManagedEnvironment` + `RslRlWrapper` |
| `rl/scripts/train_rsl_rl.py` | genesis-forge training scripts |
| `rl/scripts/eval_policy.py` | genesis-forge eval with `RslRlWrapper` |
| `genesis_bridge/robot_registry.py` | genesis-forge env configs per robot |

**Kept / adapted:**
| SDR_OS file | Status |
|-------------|--------|
| `genesis_bridge/bridge_server.py` | Adapted (wraps `ManagedEnvironment` instead of raw Genesis) |
| `genesis_bridge/gs_ros_bridge.py` | Kept (ROS2 sensor data for real hardware) |
| `genesis_bridge/profile_manager.py` | Adapted (profiles map to env configs) |
| `rl/scripts/teleop_record.py` | Adapted (uses genesis-forge obs/action format) |
| `rl/scripts/train_bc.py` | Adapted (uses genesis-forge data format) |
| `rl/configs/` | Replaced with genesis-forge config pattern (Python dicts or YAML) |

---

## 6. Implementation Order

### Phase 1: Bridge + Viewer (get a Go2 on screen)
1. Create `genesis_bridge/envs/go2_env.py` — Go2 ManagedEnvironment
2. Adapt `bridge_server.py` — use Go2BridgeEnv, step, render, stream JPEG
3. Verify SimViewer shows Go2 walking/standing in browser

### Phase 2: Gamepad Teleop (control the Go2)
4. Wire gamepad `controller_joystick_state` → `VelocityCommandManager`
5. Verify: left stick = walk direction, right stick = turn
6. Add structured metric events (obs_breakdown, reward_breakdown, velocity_command)

### Phase 3: TrustStrip Mode Controls
7. Add mode buttons (TELEOP/POLICY/BLEND) + alpha slider + E-Stop to TrustStrip
8. Wire to GenesisContext → Socket.io → bridge mode switching

### Phase 4: Telemetry Console (Right Panel Redesign)
9. TelemetryHeader (annunciator)
10. SimTelemetryPane (dense telemetry)
11. RosTelemetryPane (existing code wrapped)
12. PresetSwitcher + auto-switching logic
13. Strip panes (compact secondary views)
14. TelemetryDiffBar (split mode comparison)
15. FocusCursor (cross-source highlighting)

### Phase 5: Cleanup
16. Remove obsolete SDR_OS RL files
17. Update CLAUDE.md and documentation
18. Wire training/eval scripts to genesis-forge

---

## 7. Genesis-Forge Gaps to Address

| Gap | Solution |
|-----|----------|
| No mock mode without Genesis installed | Bridge falls back to diagnostic frames (existing behavior) |
| No web UI bridge built-in | Bridge_server.py IS the bridge layer |
| Configs are Python dicts, not YAML | Use Python config pattern for now; YAML adapter optional |
| VelocityCommandManager gamepad integration | Wire `use_gamepad()` to Socket.io joystick events |
| No teleop recording built-in | Adapt existing `teleop_record.py` to use genesis-forge obs format |

---

## 8. Key genesis-forge References

| File | What it provides |
|------|------------------|
| `genesis-forge/examples/simple/environment.py` | Complete Go2 env setup |
| `genesis-forge/genesis_forge/managed_env.py` | ManagedEnvironment.step() flow |
| `genesis-forge/genesis_forge/managers/command/velocity_command.py` | VelocityCommandManager |
| `genesis-forge/genesis_forge/managers/action/base.py` | BaseActionManager |
| `genesis-forge/genesis_forge/managers/observation_manager.py` | ObservationManager |
| `genesis-forge/genesis_forge/wrappers/video.py` | VideoWrapper for frame capture |
| `genesis-forge/genesis_forge/wrappers/rsl_rl_wrapper.py` | rsl_rl v2/v3 compatibility |

Genesis-forge root: `/home/ethan/dev/Genesis/genesis_forge_work/genesis-forge/`
