# SDR_OS Cockpit Redesign — Design Document

## Problem

SDR_OS is organized around **modes** (Real Robot vs Genesis) instead of around **the workflow** (teleop → record → train → evaluate → deploy). This causes:

- **Context switching**: swapping modes swaps the entire UI instead of shifting focus
- **Poor visibility**: you can only see one mode's state at a time, no cross-pipeline awareness
- **Clunky control**: each mode has its own panel ecosystem, toggle buttons, modals
- **Missing pieces**: glue between stages requires dropping to terminal

## Vision

SDR_OS becomes a **unified cockpit** — a single adaptive HUD where the viewer fills the screen, a trust strip gives instant situational awareness, and edge panels provide contextual control. No modes. The system knows what's active and surfaces what matters.

Primary target: Steam Deck (1280×800, 7", gamepad + touchscreen). Secondary: desktop with mouse/keyboard.

---

## Architecture: Three Layers

### Layer 1: Viewer (fullscreen background)

The viewer is the entire screen. What it shows is determined by what's running, not by a mode toggle.

**Automatic source selection:**
- ROS connected, no Genesis → Three.js 3D robot view (IMU-driven orientation)
- Genesis bridge connected → sim JPEG/WebRTC stream fills the viewer
- Both connected → sim stream primary, 3D robot becomes PiP thumbnail in bottom-left
- Nothing connected → dark canvas, strip shows red health dots

**Picture-in-picture:**
- Multiple feeds (sim camera, ROS camera, 3D model) stack as small draggable thumbnails
- Double-tap thumbnail to promote to fullscreen (previous fullscreen demotes)
- Steam Deck: L4/R4 back grips cycle through feeds
- No camera modal — feeds are layered on the viewer

**Camera controls (on the viewer):**
- Orbit/pan/zoom via mouse (left drag = orbit, shift+drag = pan, scroll = zoom)
- Touch: 1-finger orbit, 2-finger pan, pinch zoom
- Steam Deck sticks: context-aware — deadman held = sticks control robot; released = sticks control camera. Trackpads always control camera
- Keyboard: Z reset, A auto-rotate, F fullscreen, C toggle frame fit

### Layer 2: Trust Strip (top, ~36-40px, always visible)

The strip answers one question: **"Is the system doing what I think it's doing, and am I about to regret it?"**

It is a glass bar at the top of the screen, semi-transparent over the viewer. It contains stage cells and an E-STOP button.

**Stage cells:**

```
[ ● TELEOP  human ◎ ]  [ ● TRAIN  ↗ 4.2r ]  [ ● EVAL  ⚠ clamped ]  [ E-STOP ]
```

Each cell has three elements:

1. **Health pulse** (the dot): Not a static indicator — it pulses at the heartbeat rate of that stage's connection. ROS at 30Hz → dot breathes. Bridge silent for 2s → dot freezes, turns yellow. Process died → red, static. You read rhythm, not labels.

2. **Signal** (context-dependent per stage):
   - TELEOP: actor tag (`human` / `policy` / `blend 0.6`), step counter when recording (`42/200`)
   - TRAIN: trend arrow + reward (`↗ 4.2r` rising, `→` flat, `↘` collapsing). Arrow gets wobble animation (`↗~`) when variance is high (rising but unstable). Computed from rolling window, not instantaneous.
   - EVAL: risk state (`nominal`, `⚠ clamped`, `⚠ clamped ×3`). Urgency color increases with frequency of safety interventions.

3. **Inline errors**: When a stage turns red, the cell expands to show one sentence: `TRAIN ● bridge timeout`. Tap for full story in a slide-in panel.

**Authority indicator**: A color wash across the entire strip background — not per-cell. Green = human driving. Blue = policy. Split gradient = blended. This is peripheral vision — you feel it, you don't read it.

**Adaptive behaviors:**

- **Priority promotion**: Cells compete for space based on urgency. When TRAIN reward drops 20% over a rolling window, that cell grows (brighter border, larger arrow), others shrink. Attention allocation, not information display.
- **Phase-aware visibility**: If training isn't running and eval isn't queued, those cells collapse to just their health dot: `[ ● TELEOP human ◎ ] [ ● ] [ ● ] [ E-STOP ]`. Dots still pulse. When training kicks off, TRAIN smoothly expands into the strip.
- **Projection signals**: Trend arrows carry short projections. TRAIN `↗~` means rising but unstable. EVAL `⚠ clamped ×3` with increasing urgency means "getting worse." The strip models the SA loop: perception (health dots) → comprehension (authority + signal) → projection (trends + instability).

**Tappable cells**: Tapping a strip cell opens a quick-action dropdown — tap TRAIN → "pause / view dashboard / load checkpoint". Tap TELEOP → "start recording / set blend alpha / toggle deadman".

**E-STOP**: Rightmost element, always visible, big red button. One tap halts everything.

### Layer 3: Edge Panels (slide in on demand)

Panels slide over the viewer, never beside it. On 1280×800, no permanent sidebars.

**Left edge — Control panel** (L1 hold or swipe from left):
Contextual action palette. Never more than 5-6 controls. Shows only what's actionable *right now*:
- During teleop: deadman status, blend alpha slider, recording controls, velocity limits
- During training: pause/resume, LR override, checkpoint save-now
- During eval: safety overrides, alpha ramp, episode reset

Not a settings dump — a phase-aware action palette.

**Right edge — Detail panel** (R1 hold or swipe from right):
Where you investigate what the strip flagged. Depth lives here:
- TELEOP detail: joint states, IMU readout, camera feed thumbnail, latency graph
- TRAIN detail: reward curve (last 100 epochs), loss breakdown, gradient norms
- EVAL detail: episode log, safety event timeline, per-joint tracking error

The strip says "training is unstable." The right panel shows *why*.

**Bottom edge — Command bar** (trackpad double-tap or swipe up):
Replaces the bottom toolbar entirely. Contextual command palette with autocomplete:
- `load robot franka_panda`
- `start recording --steps 200`
- `train bc --steps 1000`
- `eval --checkpoint latest`
- `view cameras`
- `open settings`

Surfaces the "missing pieces" — things that currently need terminal become first-class cockpit commands. Recent commands pinned. Always dismisses after executing.

**Dismissal behavior**: Left/right panels dismiss on bumper release (peek mode) or tap to pin open (pin mode). On desktop, panels pin by default. Command bar always dismisses after action.

---

## Interaction Flow (Complete Example)

1. SDR_OS launches fullscreen on Steam Deck. Dark viewer. Strip: `[ ● ] [ ● ] [ ● ] [ E-STOP ]` — three red dots.

2. ROS bridge connects. First dot pulses green. 3D robot appears in viewer. Strip: `[ ● TELEOP human ◎ ] [ ● ] [ ● ] [ E-STOP ]`. Green authority tint.

3. Swipe up, type `connect genesis`. Bridge connects. Sim stream takes over viewer, 3D robot shrinks to PiP. Strip: `[ ● TELEOP human ◎ ] [ ● SIM connected ] [ ● ] [ E-STOP ]`.

4. Hold L1 → left panel slides in with teleop controls. Tap "Record", set 200 steps. Release L1. Strip: `[ ● TELEOP human ◎ recording 0/200 ]`. Step counter ticks as you drive.

5. Recording finishes. TELEOP cell pulses, shrinks: `idle`. Swipe up, type `train bc --steps 1000`. TRAIN wakes: `[ ● TRAIN → 0.0r ]`. Arrow tilts up as reward climbs.

6. Keep teleoperating while training runs. Both cells active. Strip stays green (you're authority). Set blend alpha 0.5 → strip tint shifts to green-blue gradient, TELEOP reads `blend 0.5`.

7. Training reward oscillates. TRAIN cell grows, border brightens amber, arrow: `↗~ 3.8r`. Hold R1 → right panel shows reward curve with variance spike. Hold L1 → left panel shows training controls, tap "reduce LR". Release both.

8. Training finishes. `✓ done 4.2r`, dims. Swipe up, `eval --checkpoint latest`. EVAL wakes: `[ ● EVAL nominal ]`. Strip tint shifts blue — policy driving.

9. Safety clamp fires. EVAL expands: `⚠ clamped ×1`. Hold R1 → safety event timeline, jerk limit on joint 3. Hold L1 → adjust threshold. Release.

10. E-STOP at any point. One tap. Everything halts. Strip goes red. Recovery is explicit.

---

## Component Architecture

### New Components

| Component | Purpose |
|-----------|---------|
| `ViewerLayer` | Fullscreen auto-source viewer with PiP management |
| `TrustStrip` | Adaptive pipeline status bar with SA model |
| `StageCell` | Individual strip cell with health pulse, signal, priority |
| `AuthorityIndicator` | Strip background color wash based on who's in control |
| `EdgePanel` | Slide-in panel container (left/right/bottom) with peek/pin modes |
| `ControlPanel` | Left edge content — phase-aware action palette |
| `DetailPanel` | Right edge content — investigation/depth view |
| `CommandBar` | Bottom edge command palette with autocomplete |
| `PipThumbnail` | Draggable picture-in-picture video/3D thumbnail |
| `usePhase()` | Hook that determines current pipeline phase from all contexts |
| `useHealth()` | Hook that computes health pulse rate per stage |
| `useTrend()` | Hook that computes rolling trend + variance for metrics |

### Refactored Components (become edge panel content)

| Current Component | Becomes |
|-------------------|---------|
| `ControlOverlay` (gamepad viz) | `ControlPanel` teleop section |
| `TelemetryPanel` | `DetailPanel` TELEOP detail |
| `TrainingDashboard` | `DetailPanel` TRAIN detail |
| `SafetyIndicator` | Absorbed into `StageCell` EVAL signal |
| `EvalControlPanel` | `ControlPanel` eval section |
| `HudBar` | Replaced by `TrustStrip` |
| `SimpleModeSelector` | Eliminated — phase is auto-detected |
| `App.jsx` bottom toolbar | Replaced by `CommandBar` |
| `GenesisRobotSelector` | `CommandBar` action: `load robot <name>` |
| `SimulationStarter` | `CommandBar` action: `start sim` |
| `StatusModal` | Absorbed into `DetailPanel` |
| `GenesisPanelModal` (wrapper) | Eliminated — edge panels replace all modals |
| `SettingsModal` | `CommandBar` action: `open settings` → right panel |

### State Architecture

```
<SettingsProvider>
  <GenesisProvider>
    <PhaseProvider>        ← NEW: determines active phase, drives adaptive behavior
      <ViewerLayer>        ← NEW: fullscreen auto-source viewer
        <PipThumbnail/>    ← NEW: draggable feed thumbnails
      </ViewerLayer>
      <TrustStrip>         ← NEW: replaces HudBar + mode selector
        <StageCell stage="teleop"/>
        <StageCell stage="train"/>
        <StageCell stage="eval"/>
        <EStopButton/>
      </TrustStrip>
      <EdgePanel side="left">   ← NEW
        <ControlPanel/>
      </EdgePanel>
      <EdgePanel side="right">  ← NEW
        <DetailPanel/>
      </EdgePanel>
      <EdgePanel side="bottom"> ← NEW
        <CommandBar/>
      </EdgePanel>
    </PhaseProvider>
  </GenesisProvider>
</SettingsProvider>
```

### New Hook: `usePhase()`

Reads from GenesisContext, ROS state, and training metrics to determine:

```javascript
{
  activePhase: 'teleop' | 'train' | 'eval' | 'idle',
  authority: 'human' | 'policy' | 'blend',
  blendAlpha: 0.0-1.0,
  stages: {
    teleop: { health: 'green'|'yellow'|'red', heartbeatHz: number, recording: bool, stepCount: [current, total] },
    train:  { health: ..., trend: 'rising'|'flat'|'falling', unstable: bool, reward: number, epoch: [current, total] },
    eval:   { health: ..., riskLevel: 'nominal'|'clamped'|'anomaly', clampCount: number, safetyFlags: {} }
  }
}
```

All adaptive UI behavior derives from this single hook.

---

## Steam Deck Input Mapping

| Input | Action |
|-------|--------|
| Left stick | Robot control (deadman held) / Camera orbit (deadman released) |
| Right stick | Robot control (deadman held) / Camera pan (deadman released) |
| Left trackpad | Camera orbit (always) |
| Right trackpad | Camera pan (always) / Double-tap → command bar |
| L1 (hold) | Peek left control panel |
| R1 (hold) | Peek right detail panel |
| L1 (tap) | Pin/unpin left panel |
| R1 (tap) | Pin/unpin right panel |
| L4/R4 (back grips) | Cycle PiP feeds |
| L2 | Deadman switch (teleop authority) |
| A/B/X/Y | Quick actions (context-dependent, shown in left panel) |
| Start | E-STOP |
| Select | Toggle strip expanded/compact |

---

## Design Tokens (additions to theme.css)

```css
/* Authority colors */
--color-authority-human: var(--color-accent-green);
--color-authority-policy: var(--color-accent-blue);
--color-authority-blend: linear-gradient(90deg, var(--color-accent-green), var(--color-accent-blue));

/* Health states */
--color-health-ok: var(--color-accent-green);
--color-health-degraded: var(--color-accent-amber);
--color-health-dead: var(--color-accent-red);

/* Urgency scale */
--color-urgency-low: var(--color-text-muted);
--color-urgency-medium: var(--color-accent-amber);
--color-urgency-high: var(--color-accent-red);

/* Strip dimensions */
--strip-height: 40px;
--strip-cell-min: 40px;
--strip-cell-expanded: 200px;

/* Edge panel dimensions */
--panel-width: 320px;
--panel-peek-delay: 150ms;
--panel-slide-duration: var(--transition-base);

/* Missing tokens from audit */
--backdrop-blur: 12px;
--border-width: 1px;
--opacity-hover: 0.08;
--opacity-active: 0.12;
--opacity-disabled: 0.4;
--focus-ring: 0 0 0 2px var(--color-accent-blue);
```

---

## What Gets Deleted

- `SimpleModeSelector` component + CSS
- `GenesisPanelModal` component + CSS (all modals become edge panel content)
- `SimpleSimViewer` component (replaced by ViewerLayer)
- `SimpleGenesisPanel` component + CSS
- `SimpleStatusPanel` component + CSS
- Bottom toolbar in App.jsx (10 toggle buttons + boolean state)
- Mode-conditional rendering in App.jsx (`isGenesisMode ? ... : ...`)
- 10+ `useState` boolean toggles in AppContent (isRobotSelectorVisible, isMultiCameraVisible, isSimStarterVisible, etc.)
