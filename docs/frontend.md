# Frontend

The frontend is a React 18 single-page application bundled with Webpack + Babel, located in `src/client/`.

## Entry points

| File | Purpose |
|------|---------|
| `index.js` | Main React entry, renders App into `#root` |
| `index.html` | HTML template; loads ROSLIB from CDN, Webpack injects `bundle.js` |
| `simple.js` | Standalone non-React version with raw gamepad polling |
| `simple.html` | Minimal HTML wrapper for `simple.js` |

## Webpack build

- **Entry**: `src/client/index.js`
- **Output**: `dist/bundle.js`
- **Loaders**: Babel (JSX/ES6), CSS, style, asset/resource (images)
- **Externals**: `roslib` mapped to global `ROSLIB` (loaded from CDN, not bundled)
- **Dev server**: port 3000, proxies rosbridge websocket

```bash
pnpm run build      # production build
pnpm run dev        # development watch mode (unminified)
```

## Component hierarchy

```
SettingsProvider
  GenesisProvider (binary WS)
    SessionProvider
      PhaseProvider
        TrainingPanelProvider
          AppContent
            genesisMode ? SimViewer : RobotViewer
            ControlOverlay
            TrustStrip / EdgePanel
            (mode-specific panels)
```

## Components (40+)

### Core layout

| Component | Purpose |
|-----------|---------|
| `App.jsx` | Root component; mode switching, layout, settings migration |
| `ViewerLayer.jsx` | Container for 3D/sim viewer |
| `TrustStrip.jsx` | Trust/safety strip UI |
| `EdgePanel.jsx` | Edge panel layout |
| `CommandBar.jsx` | Command bar overlay |
| `FloatingPanel.jsx` | Reusable floating panel container |
| `SplashScreen.jsx` | Initial loading screen |

### Control

| Component | Purpose |
|-----------|---------|
| `ControlOverlay.jsx` | Gamepad polling (every animationFrame), publishes to ROS + Socket.io |
| `Controller.jsx` | Controller UI visualization |
| `HotRow.jsx` | Quick-access control row |
| `HotRowKnob.jsx` | Rotary knob control |

### Visualization

| Component | Purpose |
|-----------|---------|
| `RobotViewer.jsx` | Three.js 3D robot model (Real Robot mode); IMU quaternion to Euler, TF subscriptions |
| `SimViewer.jsx` | H.264/JPEG stream viewer (Genesis mode); WebCodecs decoder for H.264, Blob URL for JPEG, VIDEO LOST overlay on stale frames |
| `CameraViewer.jsx` | Single camera feed (ROS webcam topic) |
| `MultiCameraViewer.jsx` | Multiple camera feeds from Genesis |
| `URDFPreview.jsx` | URDF model preview |

### Genesis simulation

| Component | Purpose |
|-----------|---------|
| `GenesisControlPanel.jsx` | Simulation control (load/reset/pause, mode, alpha) |
| `GenesisInfoPanel.jsx` | Genesis status and info display |
| `GenesisRobotSelector.jsx` | Robot selection from registry |
| `SimulationStarter.jsx` | Simulation initialization UI |
| `SimControlPanel.jsx` | Extended sim controls |

### Telemetry

| Component | Purpose |
|-----------|---------|
| `TelemetryPanel.jsx` | Main telemetry dashboard (ROS topics) |
| `TelemetryConsole.jsx` | Raw telemetry log console |
| `TelemetryDisplay.jsx` | Formatted telemetry readouts |
| `SimTelemetryPane.jsx` | Genesis-specific telemetry pane |

### Training / RL

| Component | Purpose |
|-----------|---------|
| `TrainingPanel.jsx` | Training control panel |
| `TrainingDashboard.jsx` | Training metrics visualization (rewards, losses) |
| `EvalControlPanel.jsx` | Policy evaluation controls |
| `PolicyBrowserPanel.jsx` | Browse and load trained policies |
| `GoalComposer.jsx` | Compose goals for training |

### Safety

| Component | Purpose |
|-----------|---------|
| `SafetyIndicator.jsx` | Blend alpha, deadman status, safety flags overlay |
| `SafetyStatusPanel.jsx` | Detailed safety status panel |
| `TrustStrip.jsx` | Safety badge (HOLD/ESTOP mode), E-STOP / RE-ARM toggle button |

### Settings and UI

| Component | Purpose |
|-----------|---------|
| `Settings.jsx` | Settings page |
| `SettingsModal.jsx` | Modal settings dialog |
| `SettingsIcon.jsx` | Settings gear icon |
| `CameraIcon.jsx` | Camera icon |
| `LogViewer.jsx` | Real-time log viewer |
| `SessionCard.jsx` | Session info card |
| `SessionCarousel.jsx` | Session carousel browser |

## Contexts (5 React Contexts)

### SettingsContext (useSettings hook)

7 setting sections, all auto-persisted to localStorage:

| Section | Examples |
|---------|---------|
| `ui` | Theme, layout preferences |
| `telemetry` | Update rates, display options |
| `visualization` | 3D rendering settings |
| `control` | maxLinearSpeed, deadzone, sensitivity |
| `topics` | ROS topic names (/cmd_vel, /odom, etc.) |
| `voice` | Voice control settings |
| `genesis` | Genesis-specific settings |

```javascript
const { getSetting, updateSettings } = useSettings();
const speed = getSetting('control', 'maxLinearSpeed', 1.0);
updateSettings({ control: { maxLinearSpeed: 2.0 } });
```

### GenesisContext (useGenesis hook)

Manages the Genesis simulation connection via binary WebSocket:

- Connects to `/stream/ws` (transport-server via Caddy or Node proxy)
- Receives `0x01` VIDEO frames — routes to H264Decoder (WebCodecs) or JPEG Blob URL based on codec byte
- Receives `0x02` TELEMETRY frames — parses NATS subject + JSON, updates training metrics / safety state
- Sends `0x04` COMMAND frames — JSON `{action, cmd_seq, data, ttl_ms?}` for velocity, E-STOP, RE-ARM, settings
- Tracks `videoHealthy` (false after 500ms without frames) and `safetyState` (`{mode, state_id}`)
- Blob URL lifecycle management (revokes old URLs to prevent memory leaks)
- Actions: loadRobot, reset, pause, setMode, setAlpha, setCameraPos, sendCommand, estop, estopClear, etc.

### PhaseContext

ROS phase management for the control pipeline.

### SessionContext

Session state management (session cards, carousel).

### TrainingPanelContext

Training panel UI state (which panels are open, training progress).

## Utilities

| File | Purpose |
|------|---------|
| `utils/H264Decoder.js` | WebCodecs VideoDecoder wrapper — parses 32-byte LE frame header, extracts Annex-B NALUs, decodes H.264 to canvas. Used by GenesisContext for `0x01` VIDEO frames with codec=1. |

## Audio

| File | Purpose |
|------|---------|
| `audio/SoundEffects.js` | Synthesizes sound effects at runtime using Web Audio API oscillators (no audio files). Audio context initializes on first user interaction (browser requirement). |

## Styles

Each component has a corresponding `.css` file in `src/client/styles/`. Shared styles:

| File | Purpose |
|------|---------|
| `theme.css` | Color variables, typography |
| `shared.css` | Shared utility classes |
| `App.css` | Root app layout |

## Key patterns

- **useCallback** wraps ROS subscription handlers to prevent re-subscription on re-render (specific bug fix).
- **lodash.throttle** for subscription rate limiting (e.g. TF at 10 Hz).
- **ResizeObserver** for responsive Three.js canvas sizing.
- **Gamepad polling** uses requestAnimationFrame loop in ControlOverlay, not event-based.
- **IMU display**: quaternion to Euler angles to Three.js rotation.
