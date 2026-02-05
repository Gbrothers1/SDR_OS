# Gamepad Button Mapping Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Map gamepad buttons to sim functions (posture, speed, gait, skills) with Steam Deck/Xbox auto-detection, and fix CSS overflow bugs in the controls panel.

**Architecture:** Browser detects controller type and emits mapping type once on connect. Bridge handles button events with edge detection, updating posture/speed/gait/skill state. CSS overflow chain is fixed so the mapping panel is visible and scrollable.

**Tech Stack:** React (ControlOverlay.jsx), CSS, Python (bridge_server.py), Socket.io, Node.js (server.js)

---

### Task 1: CSS Overflow Fixes

**Files:**
- Modify: `src/client/styles/EdgePanel.css:77-82`
- Modify: `src/client/styles/ControlOverlay.css:122-141,143-163`

**Step 1: Fix EdgePanel.css overflow**

In `src/client/styles/EdgePanel.css`, change `.edge-panel__content` `overflow: hidden` to `overflow-y: auto`:

```css
/* Panel content */
.edge-panel__content {
  display: flex;
  flex-direction: column;
  height: 100%;
  overflow-y: auto;
}
```

**Step 2: Fix ControlOverlay.css controls-container overflow**

In `src/client/styles/ControlOverlay.css`, change `.controls-container` `overflow: hidden` to `overflow-y: auto`:

```css
/* Container for control overlay + mapping panel */
.controls-container {
  display: flex;
  flex-direction: column;
  width: 100%;
  height: 100%;
  overflow-y: auto;
}
```

**Step 3: Fix control-section overflow**

In `src/client/styles/ControlOverlay.css`, change `.control-section` `overflow: hidden` to `overflow: visible`:

```css
.control-section {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: var(--space-4);
  flex: 1;
  padding: var(--space-4);
  overflow: visible;
  margin-left: auto;
  transition: margin-left 0.3s ease;
  font-size: var(--font-size-sm);
}
```

**Step 4: Fix mapping-section sizing**

In `src/client/styles/ControlOverlay.css`, remove `max-height: 40vh` from `.mapping-section.visible` so it sizes naturally:

```css
.mapping-section.visible {
  opacity: 1;
  pointer-events: auto;
  display: block;
}
```

**Step 5: Verify visually**

Run: `npm run build`

Open browser, open left panel, click "Show Mappings" — mappings should now be visible and the entire panel should scroll.

**Step 6: Commit**

```bash
git add src/client/styles/EdgePanel.css src/client/styles/ControlOverlay.css
git commit -m "fix: CSS overflow chain so controls panel scrolls and mappings are visible"
```

---

### Task 2: Controller Detection + Mapping Constants in ControlOverlay.jsx

**Files:**
- Modify: `src/client/components/ControlOverlay.jsx`

**Step 1: Add mapping constants and detection function**

At the top of `ControlOverlay.jsx`, after the imports (line 5), add controller mapping constants:

```javascript
// Controller type detection: 18+ buttons = Steam Deck (has L4/R4 back paddles), else Xbox
const CONTROLLER_TYPES = {
  STEAM_DECK: 'steam_deck',
  XBOX: 'xbox',
};

const detectControllerType = (gamepad) => {
  if (gamepad && gamepad.buttons.length >= 18) {
    return CONTROLLER_TYPES.STEAM_DECK;
  }
  return CONTROLLER_TYPES.XBOX;
};

// Button mapping: function name → { button name, index, description }
// Shared mappings (identical on Steam Deck and Xbox)
const SHARED_BUTTON_MAP = {
  A: { index: 0, function: 'stand', description: 'Stand up' },
  B: { index: 1, function: 'sit', description: 'Sit down' },
  X: { index: 2, function: 'resetPosture', description: 'Reset posture' },
  Y: { index: 3, function: 'emergencyFreeze', description: 'Emergency freeze' },
  L1: { index: 4, function: 'pitchForward', description: 'Pitch forward' },
  R1: { index: 5, function: 'pitchBackward', description: 'Pitch backward' },
  L2: { index: 6, function: 'speedPrecision', description: 'Precision speed (hold)' },
  R2: { index: 7, function: 'speedFull', description: 'Full speed (hold) / Gait cycle (double-tap)' },
  DpadUp: { index: 12, function: 'heightUp', description: 'Height +0.03m' },
  DpadDown: { index: 13, function: 'heightDown', description: 'Height -0.03m' },
  DpadLeft: { index: 14, function: 'leanLeft', description: 'Roll -0.05 rad' },
  DpadRight: { index: 15, function: 'leanRight', description: 'Roll +0.05 rad' },
};

// Steam Deck extra buttons (back paddles)
const STEAM_DECK_EXTRA = {
  L4: { index: 8, function: 'deadman / resetPostureQuick', description: 'Hold=deadman, Tap=reset posture' },
  R4: { index: 9, function: 'freezeStance', description: 'Freeze stance' },
};
```

**Step 2: Add controllerType state and detection on connect**

Inside the component, after `const [isGamepadConnected, setIsGamepadConnected] = useState(false);` (line 73), add:

```javascript
const [controllerType, setControllerType] = useState(CONTROLLER_TYPES.XBOX);
```

**Step 3: Detect and emit controller type on gamepad connect**

In the `handleGamepadConnected` handler (line 131), after `setIsGamepadConnected(true);` (line 135), add:

```javascript
const type = detectControllerType(e.gamepad);
setControllerType(type);
console.log('Controller type detected:', type, '(buttons:', e.gamepad.buttons.length, ')');
```

Do the same in `tryAttachFirstGamepad` after `setIsGamepadConnected(true);` (line 164):

```javascript
const type = detectControllerType(firstGamepad);
setControllerType(type);
console.log('Controller type detected:', type, '(buttons:', firstGamepad.buttons.length, ')');
```

And in the polling fallback (line 257) after `setIsGamepadConnected(true);`:

```javascript
const type = detectControllerType(firstGamepad);
setControllerType(type);
```

**Step 4: Emit mapping type to Socket.io when controller type changes**

Add a `useEffect` after the gamepad connection handlers:

```javascript
// Emit controller mapping type to bridge when detected
useEffect(() => {
  if (socket && isGamepadConnected) {
    socket.emit('controller_mapping_type', { type: controllerType });
    console.log('Emitted controller mapping type:', controllerType);
  }
}, [socket, isGamepadConnected, controllerType]);
```

**Step 5: Commit**

```bash
git add src/client/components/ControlOverlay.jsx
git commit -m "feat: add controller type detection and mapping constants"
```

---

### Task 3: Add New Socket.io Events to server.js

**Files:**
- Modify: `server.js:26-67`

**Step 1: Add new events to GENESIS_EVENTS array**

In `server.js`, add these two events to the `GENESIS_EVENTS` array (before the closing `];` on line 67):

```javascript
  'genesis_skill_command',
  'genesis_mapping_type',
```

**Step 2: Add controller_mapping_type forwarding**

After the `controller_joystick_state` handler (line 135), add:

```javascript
  // Forward controller mapping type to all clients (including bridge)
  socket.on('controller_mapping_type', (data) => {
    console.log(`[${new Date().toISOString()}] Controller mapping type from ${clientId}:`, data.type);
    io.emit('controller_mapping_type', data);
  });
```

**Step 3: Commit**

```bash
git add server.js
git commit -m "feat: add genesis_skill_command and controller_mapping_type Socket.io events"
```

---

### Task 4: Bridge Button Handler with Edge Detection and State

**Files:**
- Modify: `genesis_bridge/bridge_server.py`

**Step 1: Add new state variables**

In `bridge_server.py`, after `self._cached_velocity_cmd = None` (line 491), add:

```python
        # Gamepad button mapping state
        self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
        self.active_skill = None       # None | "stand" | "sit" | "freeze"
        self.gait_preset = "walk"      # "walk" | "trot" | "run"
        self.speed_scale = 0.6         # 0.3 | 0.6 | 1.0
        self.mapping_type = "xbox"     # "steam_deck" | "xbox"
        self._prev_buttons = {}        # for rising-edge detection
        self._r2_tap_time = 0.0        # for double-tap detection
        self._l4_press_time = 0.0      # for hold vs tap detection
        self._l4_held = False          # tracks L4 held state for deadman
```

**Step 2: Add button handler method**

After `_zero_gamepad_command()` (line 1397), add the button handler:

```python
    def _handle_button_states(self, buttons: dict):
        """Process button states with rising-edge detection.

        Called from the sync gamepad thread on every button update.
        Only acts on rising edges (button was False, now True).
        """
        prev = self._prev_buttons
        now_time = time.time()

        def rising(key):
            return buttons.get(key, False) and not prev.get(key, False)

        def held(key):
            return buttons.get(key, False)

        # --- L4: deadman / quick-reset (Steam Deck only) ---
        if self.mapping_type == "steam_deck":
            if rising("L4"):
                self._l4_press_time = now_time
                self._l4_held = True
            elif prev.get("L4", False) and not buttons.get("L4", False):
                # L4 released
                self._l4_held = False
                if (now_time - self._l4_press_time) < 0.3:
                    # Quick tap → reset posture
                    self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
                    logger.info("[BUTTONS] L4 tap → posture reset")
            self.deadman_active = self._l4_held
        else:
            # Xbox: no deadman, always active
            self.deadman_active = True

        # --- R4: freeze stance (Steam Deck only) ---
        if self.mapping_type == "steam_deck" and rising("R4"):
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            self.active_skill = "freeze"
            logger.info("[BUTTONS] R4 → freeze stance")

        # --- D-pad: posture nudges ---
        if rising("DpadUp"):
            self.posture_offset["height"] = min(
                self.posture_offset["height"] + 0.03, 0.10)
            logger.debug(f"[BUTTONS] D-Up → height={self.posture_offset['height']:.2f}")
        if rising("DpadDown"):
            self.posture_offset["height"] = max(
                self.posture_offset["height"] - 0.03, -0.10)
            logger.debug(f"[BUTTONS] D-Down → height={self.posture_offset['height']:.2f}")
        if rising("DpadLeft"):
            self.posture_offset["roll"] = max(
                self.posture_offset["roll"] - 0.05, -0.30)
            logger.debug(f"[BUTTONS] D-Left → roll={self.posture_offset['roll']:.2f}")
        if rising("DpadRight"):
            self.posture_offset["roll"] = min(
                self.posture_offset["roll"] + 0.05, 0.30)
            logger.debug(f"[BUTTONS] D-Right → roll={self.posture_offset['roll']:.2f}")

        # --- Bumpers: pitch nudges ---
        if rising("L1"):
            self.posture_offset["pitch"] = min(
                self.posture_offset["pitch"] + 0.05, 0.30)
            logger.debug(f"[BUTTONS] L1 → pitch={self.posture_offset['pitch']:.2f}")
        if rising("R1"):
            self.posture_offset["pitch"] = max(
                self.posture_offset["pitch"] - 0.05, -0.30)
            logger.debug(f"[BUTTONS] R1 → pitch={self.posture_offset['pitch']:.2f}")

        # --- Triggers: speed scaling ---
        if held("L2") and held("R2"):
            self.speed_scale = 0.3  # precision wins
        elif held("L2"):
            self.speed_scale = 0.3
        elif held("R2"):
            self.speed_scale = 1.0
        else:
            self.speed_scale = 0.6

        # --- R2 double-tap: gait cycle ---
        if rising("R2"):
            if (now_time - self._r2_tap_time) < 0.4:
                # Double-tap detected → cycle gait
                gaits = ["walk", "trot", "run"]
                idx = gaits.index(self.gait_preset)
                self.gait_preset = gaits[(idx + 1) % len(gaits)]
                logger.info(f"[BUTTONS] R2 double-tap → gait={self.gait_preset}")
                self._r2_tap_time = 0.0  # reset so triple-tap doesn't re-trigger
            else:
                self._r2_tap_time = now_time

        # --- Face buttons: skill commands ---
        if rising("A"):
            self.active_skill = "stand"
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            logger.info("[BUTTONS] A → stand")
        if rising("B"):
            self.active_skill = "sit"
            logger.info("[BUTTONS] B → sit")
        if rising("X"):
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            self.active_skill = None
            logger.info("[BUTTONS] X → reset posture")
        if rising("Y"):
            self.active_skill = "freeze"
            self.posture_offset = {"height": 0.0, "roll": 0.0, "pitch": 0.0}
            logger.info("[BUTTONS] Y → emergency freeze")

        self._prev_buttons = dict(buttons)
```

**Step 3: Wire button handler into sync gamepad thread**

In `_start_gamepad_thread()` (line 2724), after the existing `on_joystick` handler, add a button handler:

```python
            @sio_sync.on('controller_button_states')
            def on_buttons(data):
                self._handle_button_states(data)

            @sio_sync.on('controller_mapping_type')
            def on_mapping_type(data):
                new_type = data.get('type', 'xbox')
                self.mapping_type = new_type
                logger.info(f"[GAMEPAD-THREAD] controller mapping type: {new_type}")
```

**Step 4: Modify velocity handler to apply speed_scale**

In `_apply_gamepad_command()` (line 1381), apply speed_scale before passing to forge_env:

```python
    def _apply_gamepad_command(self, joy_data: dict) -> bool:
        if self.forge_env is None or joy_data is None:
            return False
        # Apply speed scaling to stick values
        scaled = {
            'leftStickX': joy_data.get('leftStickX', 0.0) * self.speed_scale,
            'leftStickY': joy_data.get('leftStickY', 0.0) * self.speed_scale,
            'rightStickX': joy_data.get('rightStickX', 0.0) * self.speed_scale,
            'rightStickY': joy_data.get('rightStickY', 0.0) * self.speed_scale,
        }
        with self._command_update_lock:
            self.forge_env.set_velocity_from_gamepad(scaled)
        return True
```

**Step 5: Add new state to get_training_metrics()**

In `get_training_metrics()`, add the new fields to the returned dict. After the `"command_source"` line (line 1352), add:

```python
            "active_skill": self.active_skill,
            "gait_preset": self.gait_preset,
            "posture_offset": self.posture_offset.copy(),
            "speed_scale": self.speed_scale,
            "mapping_type": self.mapping_type,
```

**Step 6: Remove deadman bypass**

Change line 481 from:
```python
        self.deadman_active = True  # TESTING: bypass deadman switch
```
to:
```python
        self.deadman_active = True  # Set by button handler (L4 on Steam Deck, always True on Xbox)
```

**Step 7: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat: bridge button handler with edge detection, posture/speed/gait/skill state"
```

---

### Task 5: Update Button Display in ControlOverlay.jsx

**Files:**
- Modify: `src/client/components/ControlOverlay.jsx`

**Step 1: Replace rosMappings with new mapping display**

Replace the `rosMappings` object (lines 505-530) with:

```javascript
  // Build mapping display from controller type
  const currentMappings = useMemo(() => {
    const base = { ...SHARED_BUTTON_MAP };
    if (controllerType === CONTROLLER_TYPES.STEAM_DECK) {
      Object.assign(base, STEAM_DECK_EXTRA);
    }
    return base;
  }, [controllerType]);
```

**Step 2: Update mapping section JSX**

Replace the mapping section (lines 796-837) with mapping display showing function names:

```jsx
      <div className={`mapping-section ${showMappings ? 'visible' : ''}`} onClick={(e) => e.stopPropagation()}>
        <div className="mapping-border"></div>
              {showMappings && (
          <div className="mapping-content">
                  <div className="mapping-title">
                    {controllerType === CONTROLLER_TYPES.STEAM_DECK ? 'Steam Deck' : 'Xbox'} Mappings
                  </div>

                  <div className="mapping-info">
                    <div className="mapping-title">Sticks</div>
                    <div>Left Y (inverted) → Forward/backward (-1.0 to 1.0 m/s)</div>
                    <div>Left X → Lateral strafe (-0.5 to 0.5 m/s)</div>
                    <div>Right X → Yaw rate (-1.0 to 1.0 rad/s)</div>
                  </div>

                  <div className="mapping-info">
                    <div className="mapping-title">Buttons</div>
                    {Object.entries(currentMappings).map(([name, info]) => (
                      <div key={name}>
                        <strong>{name}</strong> → {info.description}
                      </div>
                    ))}
                  </div>
          </div>
      )}
      </div>
```

**Step 3: Update panel header to show controller type**

Change the header line (line 613) from:
```jsx
<h3>CONTROLS {isSafari && !isGamepadConnected && '(Press controller button)'}</h3>
```
to:
```jsx
<h3>CONTROLS {isGamepadConnected ? `(${controllerType === CONTROLLER_TYPES.STEAM_DECK ? 'Steam Deck' : 'Xbox'})` : isSafari ? '(Press controller button)' : ''}</h3>
```

**Step 4: Add the `useMemo` import if missing**

Confirm `useMemo` is already imported on line 1 — it is: `import React, { useEffect, useRef, useState, useCallback, useMemo } from 'react';`

**Step 5: Build and verify**

Run: `npm run build`

Open browser, connect gamepad, verify:
- Header shows "CONTROLS (Xbox)" or "CONTROLS (Steam Deck)"
- Mappings show function names not raw ROS topics
- Panel scrolls properly

**Step 6: Commit**

```bash
git add src/client/components/ControlOverlay.jsx
git commit -m "feat: update button display with controller type and mapping names"
```

---

### Task 6: End-to-End Verification

**Step 1: Build frontend**

Run: `npm run build`

**Step 2: Start server**

Run: `npm start`

**Step 3: Start Genesis bridge**

Run: `python genesis_bridge/bridge_server.py`

**Step 4: Connect gamepad and verify**

Open browser at `http://localhost:3000`:

1. Connect gamepad → verify console logs controller type
2. Open left panel → verify it scrolls, no clipping
3. Press D-pad buttons → verify bridge logs posture nudges
4. Hold L2 → verify speed_scale changes in training_metrics
5. Double-tap R2 → verify gait cycles (walk → trot → run)
6. Press A/B/Y → verify skill commands logged
7. Click "Show Mappings" → verify mapping section is visible with function names

**Step 5: Verify training_metrics includes new fields**

Check the WebSocket messages include `active_skill`, `gait_preset`, `posture_offset`, `speed_scale`, `mapping_type`.
