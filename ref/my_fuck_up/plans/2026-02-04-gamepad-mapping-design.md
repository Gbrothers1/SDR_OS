# Gamepad Button Mapping Design

## Problem

Only joystick axes drive the Go2 sim robot. All gamepad buttons are published but ignored by the bridge. The controls panel in the left EdgePanel has overflow/sizing bugs — the mappings section is hidden and the button layout doesn't fit.

## Solution

Map gamepad buttons to sim functions with auto-detection of controller type (Steam Deck vs Xbox). Add posture nudges, speed scaling, gait presets, and skill command plumbing. Fix the left panel CSS overflow chain.

## Controller Detection

Detect by `gamepad.buttons.length` on connect:
- **18+ buttons** → Steam Deck (has back paddles L4/R4)
- **16 buttons** → Xbox

Returns a mapping object referenced by name throughout the codebase.

## Button Mappings

### Steam Deck Layout

| Function | Button | Index | Type |
|----------|--------|-------|------|
| **deadman** | L4 | 8 | hold (>300ms) |
| resetPostureQuick | L4 | 8 | tap (<300ms) |
| freezeStance | R4 | 9 | tap |
| pitchForward | L1 | 4 | tap |
| pitchBackward | R1 | 5 | tap |
| speedPrecision | L2 | 6 | analog |
| speedFull | R2 | 7 | analog |
| stand | A | 0 | tap |
| sit | B | 1 | tap |
| resetPosture | X | 2 | tap |
| emergencyFreeze | Y | 3 | tap |
| heightUp | D-Up | 12 | tap |
| heightDown | D-Down | 13 | tap |
| leanLeft | D-Left | 14 | tap |
| leanRight | D-Right | 15 | tap |
| gaitCycle | R2 | 7 | double-tap (<400ms) |

Steam Deck: L4 hold = deadman (nothing moves without it). L4 quick tap = reset posture.

### Xbox Layout

Same as Steam Deck except:
- **No deadman** — all inputs always active
- **No L4/R4** — resetPostureQuick and freezeStance unavailable
- L2 available for speedPrecision (not consumed by deadman)

All D-pad, bumper, trigger, and face button indices are identical across both layouts.

## Sticks (Unchanged)

| Input | Controls | Range |
|-------|----------|-------|
| Left Stick Y (inverted) | Forward/backward (lin_vel_x) | -1.0 to 1.0 m/s |
| Left Stick X | Lateral strafing (lin_vel_y) | -0.5 to 0.5 m/s |
| Right Stick X | Yaw rate (ang_vel_z) | -1.0 to 1.0 rad/s |

## Speed Scaling (Triggers)

| State | speed_scale | Effect |
|-------|-------------|--------|
| Neither held | 0.6 | Default walking speed |
| L2 held | 0.3 | Precision mode |
| R2 held | 1.0 | Full speed |
| Both held | 0.3 | Precision wins |

Applied: `velocity = stick_value * max_velocity * speed_scale`

## Gait Presets (Double-tap R2)

| Preset | Frequency | Duty Cycle | Feel |
|--------|-----------|------------|------|
| walk | 1.0 Hz | 0.65 | Slow, stable |
| trot | 2.0 Hz | 0.50 | Medium, diagonal pairs |
| run | 3.0 Hz | 0.35 | Fast, flight phase |

Detection: Two R2 presses within 400ms = cycle. Timer expires = was a speed hold.

## Posture Nudges (D-pad + Bumpers)

Each press = fixed delta, rising edge only:

| Input | Offset | Delta | Clamp |
|-------|--------|-------|-------|
| D-Up | height | +0.03m | -0.10 to +0.10m |
| D-Down | height | -0.03m | -0.10 to +0.10m |
| D-Left | roll | -0.05 rad | -0.3 to +0.3 rad |
| D-Right | roll | +0.05 rad | -0.3 to +0.3 rad |
| L1 | pitch | +0.05 rad | -0.3 to +0.3 rad |
| R1 | pitch | -0.05 rad | -0.3 to +0.3 rad |

Offsets stored in bridge as `posture_offset: {height, roll, pitch}`. Reset to zero via X button or L4 quick-tap.

## Skill Commands (Plumbing Only)

| Button | Skill | Behavior Now | Behavior Later |
|--------|-------|-------------|----------------|
| A | stand | Sets active_skill, clears posture offsets | Triggers stand-up policy |
| B | sit | Sets active_skill, zeros velocity | Triggers sit-down policy |
| Y | freeze | Zeros velocity + posture, soft e-stop | Same |

Cancellation: any face button cancels current skill. Moving sticks above deadzone auto-cancels skill.

Event flow: button press → `genesis_skill_command` Socket.io event → bridge stores `active_skill` → included in `training_metrics`.

## Bridge State Additions

```python
# New state in bridge_server.py
self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
self.active_skill = None       # null | "stand" | "sit" | "freeze"
self.gait_preset = "walk"      # "walk" | "trot" | "run"
self.speed_scale = 0.6         # 0.3 | 0.6 | 1.0
self.mapping_type = "xbox"     # "steam_deck" | "xbox"
self._prev_buttons = {}        # for edge detection
self._r2_tap_time = 0.0        # for double-tap detection
self._l4_press_time = 0.0      # for hold vs tap detection
```

Added to `get_training_metrics()`:
```python
"active_skill": self.active_skill,
"gait_preset": self.gait_preset,
"posture_offset": self.posture_offset,
"speed_scale": self.speed_scale,
```

## Data Flow

```
ControlOverlay (browser)
  ├── controller_button_states → Socket.io → bridge button handler
  ├── controller_joystick_state → Socket.io → bridge velocity handler
  └── controller_mapping_type → Socket.io → bridge (once on connect)

Bridge button handler:
  ├── edge detection (rising edge only)
  ├── D-pad/bumpers → posture_offset += delta, clamp
  ├── triggers → speed_scale
  ├── R2 double-tap → gait_preset cycle
  ├── face buttons → active_skill, genesis_skill_command event
  ├── L4 hold/tap → deadman / resetPosture (Steam Deck)
  └── Y → emergency freeze

Bridge velocity handler (existing, modified):
  ├── if Steam Deck and not deadman_active → zero velocity, return
  ├── apply speed_scale to stick values
  └── call set_velocity_from_gamepad()
```

## UI Panel Fixes

### Root Cause

Three nested `overflow: hidden` layers clip the controls panel:

1. `.edge-panel__content` (EdgePanel.css) — `overflow: hidden`, `height: 100%`
2. `.controls-container` (ControlOverlay.css) — `overflow: hidden`, `height: 100%`
3. `.control-section` (ControlOverlay.css) — `overflow: hidden`

The `.mapping-section` sits outside the scroll container and gets clipped.

### Fix

1. **EdgePanel.css**: `.edge-panel__content` → `overflow-y: auto` (let content scroll)
2. **ControlOverlay.css**: `.controls-container` → `overflow-y: auto` instead of `hidden`
3. **ControlOverlay.css**: `.control-section` → `overflow: visible` or remove the property
4. **ControlOverlay.css**: `.mapping-section.visible` → remove `max-height: 40vh` cap, let it size naturally within the scroll container

### Button Layout Display Update

Update the button visualization in ControlOverlay.jsx to show the new mapping names instead of raw button indices. Show the active mapping type (Steam Deck / Xbox) in the panel header. Display current state: active skill, gait preset, posture offsets, speed scale.

## Files Modified

- `src/client/components/ControlOverlay.jsx` — controller detection, mapping constants, emit mapping type, button display update
- `src/client/styles/ControlOverlay.css` — overflow fixes, mapping section sizing
- `src/client/styles/EdgePanel.css` — overflow fix on content wrapper
- `genesis_bridge/bridge_server.py` — button handler, posture/skill/gait/speed state, edge detection, double-tap/hold timers, metrics additions

## Not Built Now

- Actual policy behaviors for stand/sit/freeze
- GaitCommandManager parameter injection (presets stored, not wired to 14-dim gait vector)
- Posture offset application to env (stored in bridge only)
- Follow-me command
- Touchpad mappings
- UI skill badge display
