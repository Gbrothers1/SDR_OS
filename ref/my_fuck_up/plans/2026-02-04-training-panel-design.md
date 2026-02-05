# Training Panel GUI Design

**Date**: 2026-02-04
**Status**: Draft
**Author**: Claude + Ethan (brainstorming session)

## Overview

The Training Panel is the core module of SDR_OS, providing an intuitive interface for training robots via the REAL→SIM ↔ SIM→REAL pipeline. It supports live RL training, teleop demo recording, SLAM environment capture, and policy evaluation.

### Design Goals

- **Steam Deck optimized**: D-pad + face button navigation, no mouse required
- **Context-sensitive**: Controls adapt to current phase (Recording, Training, SLAM)
- **Zero-friction iteration**: Clone sessions instantly, adjust params live, preview results
- **Multi-robot support**: Carousel of training sessions with 3D URDF previews

---

## Entry & Layout

### Entry Point

- Press **Train** button in trust-strip → Training panel opens (75% of screen)
- Press **Train** again → Panel closes
- No minimize button, no glass effects, just a clean toggle

### Layout Structure

```
┌─────────────────────────────────────────────────────────┐
│ [PiP 22%]              [Preset Strip]                   │
│ Top-left               Sit/Stand | Locomotion | Transfer│
│ Episode                                                 │
│ highlight              ┌─────────────────────────────┐  │
│                        │                             │  │
│                        │   CAROUSEL (centerpiece)    │  │
│                        │   Floating 3D URDF cards    │  │
│                        │   D-pad/swipe navigation    │  │
│                        │   Last card = "+ New"       │  │
│                        │                             │  │
│                        └─────────────────────────────┘  │
│                                                         │
│ ┌─────────────────────────────────────────────────────┐ │
│ │ HOT ROW (always visible, 6 knobs)                   │ │
│ │ [Decimation] [Cmd Range] [Slip] [Clearance] [Push] [μ]│
│ └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### PiP Behavior

- **Position**: Top-left, 22% screen width, 16:9 aspect
- **Content**: Recorded episode highlight (2-3s loop) from focused session
- **Auto-dim**: Opacity drops when adjusting controls
- **Peek expand**: Y to expand to 40% for 2 seconds
- **Audio**: Off by default

---

## Carousel & Session Cards

### Carousel Behavior

- Centerpiece of the panel, dominates visual hierarchy
- **D-pad left/right** or **swipe** to navigate between session cards
- Round-robin: wraps from last to first
- Last card is always **"+ New Session"**

### Session Card Contents

Each card displays:

- **Floating 3D URDF model** (interactive, slow rotation, rendered with Three.js)
- **Robot name** (Go2, Franka, etc.)
- **Session type badge** (RL Training / Env Capture)
- **Preset badge** (Locomotion, Transfer Prep, Custom)
- **Status** (Running / Paused / Done)
- **Last run time** (e.g., "2h ago")
- **Override indicator** (small dot if params differ from preset)
- **Hover/focus** triggers video thumbnail preview (most recent episode highlight)

### "+ New Session" Card

- Minimal wizard, no environment picker
- Steps: Robot → Type (RL Training / Env Capture) → Preset → Name
- Default environment = last-used for that robot (or Flat Ground)
- Drops you directly into the new session

### Clone Flow (L2 on any session card)

- Instant clone: same robot, type, preset, overrides, environment
- Small overlay: editable name with timestamp default (e.g., `Go2 Locomotion (clone) 02-04 19:22`)
- **A** = Create, **B** = Create with default name
- Missing environment falls back to Flat Ground with "Missing env (reverted)" flag

---

## Context-Sensitive Controls (3 Phases)

Controls adapt based on the selected session's current phase.

### Phase 1: Teleop Recording

When capturing demonstrations from gamepad/real robot:

| Control | Description |
|---------|-------------|
| Record Start/Stop | Big toggle button, A to trigger |
| Demo counter | "12 demos captured (4.2 min total)" |
| Quality indicator | Success rate, avg episode length |
| Demo health flags | too_short, fell, stalled, low_motion, cmd_mismatch, high_slip |
| Discard last | L2 + A (no confirm) |
| Trim last demo | Y opens timeline scrubber |
| Quick tags | sit, stand, trot, recover, fail, terrain:rough |
| Dataset preview | Thumbnail grid with health/tag badges |
| Export dataset | Save to `rl/datasets/` |

### Phase 2: Training (BC/PPO)

When learning is active:

| Control Area | Contents |
|--------------|----------|
| **Hot Row** (always visible) | Decimation, Cmd Range, Slip, Clearance, Push, Friction |
| **Preset Strip** (above Hot Row) | Sit/Stand Focus, Locomotion, Transfer Prep |
| **Metrics Dock** (side) | Reward curve, success rate, episode length, FPS, step count, termination breakdown, KL/entropy |
| **Advanced** (R1 toggle) | Task, Termination, Contact, Actuation, Curriculum, Noise |
| **Quick Views** (inside Advanced) | Stability, Speed, Transfer (curated param subsets) |
| **Checkpoints** | Save now, auto-save interval, rollback, compare |

#### BC vs PPO Mode Awareness

Hot Row adapts based on training algorithm:

| Slot | PPO Mode | BC Mode |
|------|----------|---------|
| 1 | Decimation | Decimation |
| 2 | Cmd Range | Demo mix ratio |
| 3 | Slip weight | BC loss weight |
| 4 | Clearance | Action scale |
| 5 | Push | Augmentation strength |
| 6 | Friction | Noise injection |

### Phase 3: SLAM/Mapping

When robot is exploring with webcam + LiDAR:

| Control | Description |
|---------|-------------|
| Sensor feeds | Webcam, LiDAR point cloud, depth map (expandable tiles) |
| Tracking quality | Good / Degraded / Failing (early warning) |
| Loop closure | Stable / Risky / Lost (drift status) |
| Coverage | % of defined bounds |
| Capture controls | A = Start/Stop, X = Pause/Resume |
| Anchors | Y to drop spatial reference |
| Snapshots | State capture with thumbnail + note |
| Spawn points | Mark training spawn locations |
| Relocalize | Recovery options when Lost |
| Export to library | Save as USD with collision mode + spawn selection |

---

## Hot Row (6 Essential Knobs)

### Essential Live-Tunables for Go2

1. **Policy rate / decimation** — control frequency
2. **Command range** — vx/vy/yaw min/max (tap to cycle)
3. **Foot slip weight** — standing stability
4. **Foot clearance** — target + weight (toggle sub-field)
5. **Push disturbance** — magnitude/probability
6. **Friction / motor strength** — μ range

### Navigation

| Input | Action |
|-------|--------|
| D-pad left/right | Move between knobs |
| D-pad up/down | Adjust value |
| A | Toggle sub-field (vx/vy/yaw, target/weight) or enter edit |
| Y | Reset to preset value |
| L1 (hold) | Fine adjust (0.2x) |
| L2 (hold) | Coarse adjust (10x) |

### Param-Aware Step Sizes

| Param Type | Fine (L1) | Normal | Coarse (L2) |
|------------|-----------|--------|-------------|
| Float | ±0.2x with rounding | ±1x | ±10x |
| Int | ±1 | ±1 | ±5 or ±10 |
| Enum | Cycle options | Cycle options | Jump ±3 or wrap |

---

## Advanced Drawer

### Access

- **R1** opens/closes
- Default focus = Quick Views (not raw categories)

### Categories

1. **Task** — command ranges, mixing schedule, slew rate
2. **Termination** — fall thresholds, grace window, reset distribution
3. **Contact** — slip penalty, clearance, impact, timing
4. **Actuation** — action scale, latency, PD gains, torque limits
5. **Curriculum** — terrain, push, friction, command expansion, **Environment**
6. **Noise** — IMU, encoder, filtering

### Quick Views (Curated Subsets)

- **Stability** — termination thresholds, base height/orientation, slip, action scale/PD
- **Speed** — command ranges, decimation, friction
- **Transfer** — noise, latency, motor strength randomization

### Navigation

| Input | Action |
|-------|--------|
| D-pad left/right | Switch category |
| D-pad up/down | Navigate param list |
| A | Edit param |
| Y | Reset param to preset |
| X | Toggle "Overridden only" filter |
| B | Close Advanced |

---

## Presets & Overrides

### Preset Strip

Located above Hot Row:

- **Sit/Stand Focus**
- **Locomotion**
- **Transfer Prep**

### Preset Behavior

- **L1 (hold 300ms)** when preset strip focused = cycle preset
- Override badges: any param differing from preset shows dot
- Override filter: X in Advanced shows "Overridden Only"
- Reset param: Y on any param reverts to preset
- **Revert all overrides**: Start menu action

---

## Environment System

### Environment as Session Property

- Every session has exactly one active environment
- Editable anytime via Advanced → Curriculum → Environment
- Cloning copies the environment
- Missing environment falls back to Flat Ground with warning

### Mid-Run Environment Switch

Confirmation overlay:
- "Switching environments will reset the episode."
- Options: **Switch Now** / **Queue Next Reset** (default) / **Cancel**

### Environment Library

Access: Start menu → "Environment Library"

#### Tabs

- **Downloaded**
- **Captured (SLAM)**
- **Procedural**

#### Card Contents

- Thumbnail
- Name
- Source tag
- File size
- Perf indicator (Low / Med / High poly)
- Status: Ready / Processing / Error

#### Controls

| Button | Action |
|--------|--------|
| D-pad left/right | Switch tabs |
| D-pad up/down | Navigate grid |
| A | Preview (3D viewer) |
| X | Set as default for current robot |
| Y (hold + A) | Delete |
| R1 | Import |
| L1 | Filter/sort |
| B | Close |

#### Import Formats

- GLB/GLTF, USD, OBJ, FBX
- Converts to USD (canonical) on import
- Collision mode: Simple / Hybrid / Detailed

### Environment Sets

Ordered list with weights for curriculum sampling:

```
Transfer Prep Set
├── Flat Ground      (40%)
├── Rough Terrain v1 (30%)
├── SLAM Garage      (20%)
└── Stairs Demo      (10%)
```

Access: Advanced → Curriculum → Environment → "Manage Sets"

---

## Checkpoints

### Controls

| Action | Description |
|--------|-------------|
| Save checkpoint now | Immediate save with timestamp |
| Auto-save interval | Every N steps or N minutes |
| Rollback to checkpoint | Pick from list |
| Best checkpoint | Auto-saved on new reward high (⭐) |

### Checkpoint List

```
⭐ step_48000 (best)     reward: 12.4   2m ago
   step_45000            reward: 11.8   8m ago
   step_40000            reward: 10.2  15m ago
```

### Compare Mode (X)

Select two checkpoints, shows side-by-side:
- Reward
- Success rate
- Termination breakdown
- Episode length

---

## SLAM/Mapping Phase Details

### Sensor Feed Display

Collapsible row, each ~20% width:

| Feed | Description |
|------|-------------|
| Webcam | RGB stream |
| LiDAR | Point cloud |
| Depth map | Fusion output |

- A on feed = expand to 45%
- L1 cycles preview quality (Low/Med/High)

### Status Indicators

| Indicator | States |
|-----------|--------|
| Tracking quality | Good ✓ / Degraded ⚠️ / Failing ❌ |
| Loop closure | Stable ✓ / Risky ⚠️ / Lost ❌ |
| Coverage | 0-100% of bounds |
| Confidence | Low / Medium / High |

### Markers

| Marker | Purpose | Button |
|--------|---------|--------|
| Anchor | Spatial reference | Y |
| Snapshot | State + thumbnail + note | Start menu |
| Spawn point | Training spawn | Start menu |

### Export Flow

1. Start → "Export to library"
2. Configure: Name, Collision mode (Simple/Hybrid/Detailed), Spawn points, Continue vs Finalize
3. Processing: Generating mesh → Baking textures → Building collisions → Converting to USD → Ready

---

## Blend/Eval Edge Panel

**Location**: Right edge panel (separate from Training Panel)
**Access**: R2 opens/closes

### Layout

```
┌──────────────────────────┐
│ BLEND / EVAL        [R2] │
├──────────────────────────┤
│ Mode: ○ Teleop           │
│       ○ Policy           │
│       ● Blend            │
├──────────────────────────┤
│ Alpha         [████░░] 0.6│
│ ├─ Ramp rate      0.1    │
│ └─ Confidence gate  ON   │
├──────────────────────────┤
│ SAFETY                   │
│ ├─ Max vel       1.0 m/s │
│ ├─ Max accel     5.0 m/s²│
│ ├─ Max jerk     50.0 m/s³│
│ ├─ Confidence thresh 0.5 │
│ └─ Clamp count     3 ⚠️  │
├──────────────────────────┤
│ POLICY                   │
│ ├─ Loaded: Go2_Loco_48k  │
│ ├─ Checkpoint: step_48000│
│ └─ [Browse policies...]  │
├──────────────────────────┤
│ STATUS                   │
│ ├─ Risk: Nominal ✓       │
│ ├─ Deadman: Active       │
│ ├─ Confidence: 0.87      │
│ └─ Anomaly: None         │
├──────────────────────────┤
│ A: E-STOP                │
└──────────────────────────┘
```

### Risk Levels

| Level | Trigger | Display |
|-------|---------|---------|
| Nominal | All systems normal | Green ✓ |
| Clamped | Safety bounds triggered | Yellow ⚠️ |
| Anomaly | Unusual behavior | Red ❌ + auto alpha reduction |

---

## Steam Deck Controls

### Global Controls

| Button | Action |
|--------|--------|
| Train (trust-strip) | Toggle panel open/close |
| Start/Menu | Open Quick Actions (context-aware) |
| B | Back one layer (never exits panel) |
| X | Phase-aware Pause/Resume |
| Y | PiP peek (browsing) / Reset to preset (editing) |
| L1 | Fine adjust (hold) / Preset cycle (hold 300ms when focused) |
| L2 | Clone (carousel) / Coarse adjust (hold while editing) |
| R1 | Open/Close Advanced drawer |
| R2 | Open/Close Blend panel |

### A = Primary Action (Context-Aware)

| Context | A Action | Footer |
|---------|----------|--------|
| Carousel | Select session | `A: Select` |
| Hot Row knob | Toggle sub-field / edit | `A: Toggle` / `A: Edit` |
| Sensor feed tile | Expand/collapse | `A: Expand` |
| Main SLAM panel | Start/Stop capture | `A: Start` / `A: Stop` |
| Teleop (not editing) | Record Start/Stop | `A: Record` |
| E-STOP focused | Trigger E-STOP | `A: E-STOP` |

### X = Phase-Aware Pause/Resume

| Phase | X Action |
|-------|----------|
| Teleop | Pause/resume recording stream |
| Training | Pause/resume training |
| SLAM | Pause/resume mapping |

### Editing Mode

| Action | Result |
|--------|--------|
| A on focused control | Enter editing mode |
| A while editing | Confirm value |
| B while editing | Cancel, revert |
| D-pad while editing | Adjust value |

### Vertical Navigation

```
     ┌──────────────┐
     │   Advanced   │  ← D-pad up from Hot Row (if open)
     └──────────────┘
            ↑↓
     ┌──────────────┐
     │   Hot Row    │  ← D-pad up from Carousel
     └──────────────┘
            ↑↓
     ┌──────────────┐
     │   Carousel   │  (default focus)
     └──────────────┘
```

### Focus Indicator (Footer)

Always shows current state:

```
Focus: Carousel          A: Select    Y: Peek    L2: Clone
Focus: Hot Row           A: Toggle    Y: Peek    L1: Fine
Editing: Slip Weight     A: Confirm   Y: Reset   L1: Fine  L2: Coarse
Focus: Advanced/Contact  A: Edit      Y: Reset   X: Overrides Only
```

### Quick Actions Menu (Start/Menu)

| Always | Teleop | Training | SLAM |
|--------|--------|----------|------|
| New Session | Export dataset | Save checkpoint | Export to library |
| Clone Current | Tag demos | Rollback checkpoint | Drop anchor |
| Rename Session | Trim last demo | Auto-save settings | Collision mode |
| Export Config | | | |
| Environment Library | | | |
| Set Environment... | | | |
| Notes / Tags | | | |
| Revert all overrides | | | |
| Delete (hold Y + A) | | | |

---

## Implementation

### New Components

| Component | Purpose |
|-----------|---------|
| `TrainingPanel.jsx` | Main 75% overlay container |
| `SessionCarousel.jsx` | Carousel with 3D URDF cards |
| `SessionCard.jsx` | Individual card with URDF, status, video |
| `HotRow.jsx` | 6-knob control strip |
| `AdvancedDrawer.jsx` | R1 drawer with categories + Quick Views |
| `PresetStrip.jsx` | Preset buttons with override badges |
| `TeleopRecordingControls.jsx` | Phase 1 controls |
| `TrainingControls.jsx` | Phase 2 controls (BC/PPO aware) |
| `SLAMControls.jsx` | Phase 3 controls |
| `EnvironmentLibrary.jsx` | Environment browser/manager |
| `EnvironmentSetEditor.jsx` | Curriculum set builder |
| `BlendEvalPanel.jsx` | Right edge panel (R2) |
| `CheckpointManager.jsx` | Checkpoint list, compare, rollback |
| `DemoTagger.jsx` | Tag/filter demos |
| `DemoTrimmer.jsx` | Trim start/end of demos |

### New Contexts

| Context | Purpose |
|---------|---------|
| `TrainingPanelContext.jsx` | Panel open/close, focused session, current phase |
| `SessionContext.jsx` | Per-session state: params, overrides, preset, environment |

### Backend Events (bridge_server.py)

| Feature | Socket Events |
|---------|---------------|
| Session CRUD | `genesis_create_session`, `genesis_clone_session`, `genesis_delete_session` |
| Param updates | `genesis_set_param` |
| Checkpoints | `genesis_save_checkpoint`, `genesis_load_checkpoint`, `genesis_list_checkpoints` |
| Episode recording | `genesis_save_episode_highlight` |
| Demo management | `genesis_tag_demo`, `genesis_trim_demo`, `genesis_export_dataset` |
| Environment | `genesis_list_environments`, `genesis_import_environment`, `genesis_set_environment` |
| SLAM | `genesis_start_slam`, `genesis_stop_slam`, `genesis_drop_anchor`, `genesis_export_map` |

### Config Files

| File | Contents |
|------|----------|
| `rl/configs/presets/sit_stand.yaml` | Sit/Stand Focus preset |
| `rl/configs/presets/locomotion.yaml` | Locomotion preset |
| `rl/configs/presets/transfer.yaml` | Transfer Prep preset |

---

## Open Questions

1. **SLAM library integration**: Which SLAM backend? (ORB-SLAM3, RTAB-Map, custom?)
2. **3D URDF rendering**: Use existing Three.js URDF loader or custom?
3. **Video recording format**: MP4 with H.264, or WebM for browser compat?
4. **Episode highlight storage**: Per-session folder or central cache with LRU eviction?
5. **Multi-GPU**: Can sessions run on different GPUs simultaneously?
