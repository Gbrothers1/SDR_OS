# UI Integration Plan — First Tagged Release

**Date:** 2026-02-05
**Branch:** ui-reimplement
**Target:** Merge-ready UI with all panels functional and terminology correct

---

## 1. Trust Strip Rework

### 1.1 Layout

```
LEFT (stage cells):   [●SIM] [●TELEOP] [●TRAIN] [●EVAL]
RIGHT (actions):      [TELEM] [POLICY] [BLEND] [LAB] [E-STOP]
```

### 1.2 Renames

| Current | New | File | Why |
|---------|-----|------|-----|
| TELEOP mode button | TELEM | TrustStrip.jsx | Opens telemetry, not teleop controls |
| TRAIN action button | LAB | TrustStrip.jsx, App.jsx | Avoids collision with TRAIN stage cell |
| OPERATE tab (TelemetryConsole) | ROS | TelemetryConsole.jsx | Actual data source name |

### 1.3 TELEOP Stage Cell — Actor Label

**Current:** Shows "teleop" (the mode name)
**Change:** Show the active control input method: "gamepad" or "ros"

Source: `commandSource` from GenesisContext training metrics. Values: `"gamepad"`, `"ros"`, or `"keyboard"`.

**Files:** TrustStrip.jsx lines 52-58

### 1.4 TRAIN Stage Cell — New Pop-Down Panel

Add a `TrainDetailPopup` (same pattern as `SimDetailPopup`) triggered on click of the TRAIN stage cell.

**Metrics to display (from GenesisContext `trainingMetrics`):**

| Section | Metrics |
|---------|---------|
| **Progress** | Current step, total reward, SPS (steps/sec) |
| **Episode** | Mean/min/max episode length |
| **Reward Components** | Gait phase, foot height, velocity tracking, action rate, body acceleration |
| **Policy** | Loaded checkpoint name, algorithm (PPO/BC) |
| **Gait Distribution** | Trot/Pace/Bound/Pronk environment counts |
| **Loss** | Value loss, surrogate loss, action noise std |

**Visual:** Compact sparkline for reward history, same glass-morphism style as SimDetailPopup.

### 1.5 Mode Buttons — Equal Panel Opening

**Current bug:** Only POLICY auto-opens the right edge panel.
**Fix:** All three mode buttons (TELEM/POLICY/BLEND) open the right panel on click.

**Implementation in App.jsx:**
- Remove the `showPolicyPanel`-only auto-open useEffect
- Add `setRightOpen(true)` to the mode change handler for all modes
- Right panel content switches based on active mode:
  - TELEM → TelemetryConsole (with ROS/SIM/SPLIT tabs)
  - POLICY → PolicyBrowserPanel
  - BLEND → TelemetryConsole in SPLIT preset

### 1.6 E-STOP Tooltip

Add `title="Emergency stop — halt all motion"` to E-STOP button.

---

## 2. Policy Panel Fixes

### 2.1 Broken `setDt` Function

**Bug:** `PolicyBrowserPanel` destructures `setDt` from `useGenesis()` but it is never exported from GenesisContext.

**Fix:** Add to GenesisContext.jsx:
```javascript
const setDt = useCallback((newDt) => {
  if (socket) {
    socket.emit('genesis_settings', { dt: newDt });
  }
}, [socket]);
```

Export `setDt` in the context value object.

### 2.2 Policy Panel — Already Complete

The PolicyBrowserPanel already implements the full live-swap system from ~/dev/Genesis/SDR_OS:
- Filter tabs (All/PPO/BC)
- Policy cards with metadata
- Checkpoint-level selection
- Loading status feedback
- Refresh scan

No additional work needed beyond the `setDt` fix.

---

## 3. Lost Code Recovery

### 3.1 ErrorBoundary (CRITICAL)

**Lost from main repo.** Must be re-added for production stability.

**What it did:**
- React error boundary wrapping the app
- Caught render crashes and showed recovery UI
- "Reload" button for users to recover from fatal errors

**Action:** Port `ErrorBoundary` class component from main repo's App.jsx into the ui worktree. Wrap the provider tree in App.jsx.

### 3.2 getRoslib() Safe Accessor (LOW PRIORITY)

**Lost from main repo.** The ui worktree uses webpack `import ROSLIB from 'roslib'` instead of CDN loading with `getRoslib()` fallback. Webpack approach is better — no action needed.

---

## 4. Terminology Standardization

### 4.1 "Genesis" vs "Sim" in User-Facing Text

Standardize all user-visible text to "Sim" or "Simulation". Reserve "Genesis" for code internals only.

| Location | Current | Change To |
|----------|---------|-----------|
| ViewerLayer placeholder | "Genesis bridge not connected" | "Sim bridge not connected" |
| SplashScreen boot line | "Genesis bridge: will connect on demand" | "Sim bridge: will connect on demand" |
| GenesisControlPanel title | "Genesis Control" | "Sim Control" |
| GenesisInfoPanel title | "Genesis Simulation" | "Simulation Info" |

Context names, filenames, and props stay as `genesis*` internally — rename only user-facing strings.

### 4.2 Stage Label Capitalization

Unify to ALL CAPS to match mode buttons:

| Current | Change To |
|---------|-----------|
| Teleop | TELEOP |
| Train | TRAIN |
| Eval | EVAL |

**File:** TrustStrip.jsx `STAGE_LABELS` constant.

### 4.3 Stream Abbreviations

| Current | Change To |
|---------|-----------|
| wRTC | WebRTC |
| ScK | WS |

**File:** TrustStrip.jsx SimHoverPopup.

### 4.4 Alpha Slider Label

**Current:** Shows only Greek letter `a`
**Change:** Add tooltip: `"Policy blend: 0 = human, 1 = policy"`

---

## 5. CSS Token Cleanup

### 5.1 Hardcoded Colors

Replace all hardcoded color values with CSS variables:

| Hardcoded | Variable | Files |
|-----------|----------|-------|
| `#3ecf8e` | `var(--color-accent-green)` | App.css, TrustStrip.css |
| `#00e5ff` | `var(--color-accent-cyan)` | ControlOverlay.css |
| `#dfe7ff`, `#5b7099`, `#3a4a65` | `var(--color-text-primary)`, `var(--color-text-secondary)`, `var(--color-text-muted)` | SplashScreen.css |
| `#4ade80` | `var(--color-accent-green)` | SplashScreen.css |

### 5.2 Z-Index Fix

**SplashScreen.css:** Change `z-index: 9999` to `z-index: var(--z-modal)` (1000).

### 5.3 Transition Standardization

Replace hardcoded durations with design tokens:
- `0.15s` → `var(--transition-fast)` (120ms)
- `0.2s` / `0.25s` → `var(--transition-base)` (200ms)
- `0.3s` / `0.35s` → `var(--transition-slow)` (350ms)

---

## 6. Version Alignment

### 6.1 Version Numbers

| File | Current | Change To |
|------|---------|-----------|
| pyproject.toml | 0.1.0 | 0.2.0 |
| package.json | 1.0.0 | 0.2.0 |
| SplashScreen boot text | v1.0 (hardcoded) | Dynamic from package.json or `0.2.0` |

### 6.2 Splash Version

Make boot text version dynamic. Import version from a shared constant or read from a build-time injected variable via webpack DefinePlugin.

---

## 7. Server Alignment

The ui worktree server is MORE complete than main. Key features already present:
- Health check endpoint (`/api/status`)
- Graceful shutdown (SIGTERM/SIGINT)
- Socket.io CORS configuration
- Smart HTTP logging (filters static assets)
- SPA fallback route (`app.get('*', ...)`)
- 104 Genesis events (vs 44 in main)

### 7.1 Dangling Reference

**package.json** references `src/server/simple-server.js` in `start:simple` script but the file doesn't exist. Remove the script or create the file.

---

## 8. Build & Bundle

### 8.1 Bundle Size Warning

Webpack reports `bundle.js` at 1.04 MiB (limit 244 KiB). Consider:
- Code splitting (dynamic imports for heavy panels)
- Tree shaking (Three.js selective imports)
- Not blocking for first release, but flag for v0.3.0

### 8.2 Missing Frontend Lint in CI

`ci.yml` only runs Python ruff lint. Add:
- `npx webpack --mode production` build check
- Optional: ESLint for JSX

---

## 9. Documentation Updates

### 9.1 CHANGELOG.md

Create `CHANGELOG.md` for v0.2.0 release notes covering:
- UI reimplementation (40 components, 5 contexts)
- Test mode for offline development
- StreamStats monitoring component
- WebRTC DataChannel gamepad support
- Policy live-swap browser
- H.264 decoder with new wire format

### 9.2 README.md

Expand from 6 lines to include:
- Feature summary
- Screenshot
- Quick start instructions
- Link to MkDocs site

### 9.3 MkDocs Site

Add `docs/ui.md` covering:
- Component architecture
- Settings system
- Control flow (gamepad → server → bridge)
- Trust strip modes and stages

---

## 10. Implementation Priority

### Phase 1 — Functional Fixes (blocks release)
1. Trust strip renames (TELEM, LAB, ROS tab, actor labels)
2. Mode buttons equally open right panel
3. Fix broken `setDt` in GenesisContext
4. Re-add ErrorBoundary
5. TRAIN stage cell pop-down panel

### Phase 2 — Polish (should ship with release)
6. Terminology standardization (Genesis → Sim in UI text)
7. Stage label capitalization (ALL CAPS)
8. Stream abbreviation cleanup (wRTC → WebRTC, ScK → WS)
9. Alpha slider tooltip
10. E-STOP tooltip
11. CSS hardcoded color → variable migration
12. Z-index and transition token fixes

### Phase 3 — Release Prep
13. Version alignment (both files → 0.2.0)
14. Dynamic splash version
15. Remove dangling `start:simple` script
16. Create CHANGELOG.md
17. Expand README.md
18. Add webpack build step to CI

---

## Files Touched

### Phase 1
- `src/client/components/TrustStrip.jsx` — renames, pop-down, actor label
- `src/client/components/App.jsx` — mode button panel opening, ErrorBoundary
- `src/client/components/TelemetryConsole.jsx` — OPERATE → ROS rename
- `src/client/contexts/GenesisContext.jsx` — export setDt
- `src/client/styles/TrustStrip.css` — pop-down styles

### Phase 2
- `src/client/components/ViewerLayer.jsx` — "Genesis" → "Sim"
- `src/client/components/SplashScreen.jsx` — "Genesis" → "Sim", version
- `src/client/components/GenesisControlPanel.jsx` — title rename
- `src/client/components/GenesisInfoPanel.jsx` — title rename
- `src/client/styles/SplashScreen.css` — hardcoded colors
- `src/client/styles/ControlOverlay.css` — hardcoded colors
- `src/client/styles/App.css` — hardcoded colors

### Phase 3
- `package.json` — version, remove script
- `pyproject.toml` — version
- `CHANGELOG.md` — new file
- `README.md` — expand
- `.github/workflows/ci.yml` — add webpack build
- `docs/ui.md` — new file
