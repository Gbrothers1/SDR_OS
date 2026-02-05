# Cockpit Redesign Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the mode-based SDR_OS UI with a unified cockpit: fullscreen auto-source viewer, adaptive trust strip, and contextual edge panels — primary target Steam Deck 1280x800.

**Architecture:** New `PhaseProvider` context determines pipeline state from ROS + Genesis connections + training metrics. `ViewerLayer` auto-selects source (Three.js or sim stream). `TrustStrip` shows adaptive stage cells with health/signal/authority. `EdgePanel` containers slide from left/right/bottom with phase-aware content. Existing components (ControlOverlay, TelemetryPanel, etc.) become edge panel content. App.jsx drops from 620 lines / 10+ boolean toggles to ~80 lines of provider nesting.

**Tech Stack:** React 18, Three.js 0.159, Socket.io, CSS custom properties (existing design system in `theme.css`)

**Design doc:** `docs/plans/2026-02-02-cockpit-redesign-design.md`

---

## Phase 1: Foundation (usePhase + ViewerLayer)

### Task 1: Extend theme.css with new design tokens

**Files:**
- Modify: `src/client/styles/theme.css`

**Step 1: Add new tokens to theme.css**

Append these inside the `:root` block, after the existing `--transition-slow` line (line 72):

```css
  /* Authority colors */
  --color-authority-human: var(--color-accent-green);
  --color-authority-policy: var(--color-accent-blue);

  /* Health states */
  --color-health-ok: var(--color-accent-green);
  --color-health-degraded: var(--color-accent-amber);
  --color-health-dead: var(--color-accent-red);

  /* Strip */
  --strip-height: 40px;
  --strip-cell-min: 40px;
  --strip-cell-expanded: 200px;

  /* Edge panels */
  --panel-width: 320px;
  --panel-slide-duration: var(--transition-base);

  /* Missing tokens from audit */
  --backdrop-blur: 12px;
  --border-width: 1px;
  --opacity-hover: 0.08;
  --opacity-active: 0.12;
  --focus-ring: 0 0 0 2px var(--color-accent-blue);
```

**Step 2: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 3: Commit**

```bash
git add src/client/styles/theme.css
git commit -m "feat: add cockpit design tokens to theme.css"
```

---

### Task 2: Create PhaseContext with usePhase hook

This is the brain of the cockpit. It reads from GenesisContext and ROS state to compute pipeline phase, authority, and per-stage health/signal.

**Files:**
- Create: `src/client/contexts/PhaseContext.jsx`

**Step 1: Create PhaseContext.jsx**

```jsx
import React, { createContext, useContext, useState, useEffect, useRef, useCallback } from 'react';
import { useGenesis } from './GenesisContext';

const PhaseContext = createContext();

export const usePhase = () => {
  const context = useContext(PhaseContext);
  if (!context) {
    throw new Error('usePhase must be used within a PhaseProvider');
  }
  return context;
};

// Compute trend direction from a rolling window of values
const computeTrend = (history, windowSize = 10) => {
  if (history.length < 2) return { direction: 'flat', unstable: false };

  const recent = history.slice(-windowSize);
  const first = recent[0];
  const last = recent[recent.length - 1];
  const delta = last - first;

  // Compute variance
  const mean = recent.reduce((a, b) => a + b, 0) / recent.length;
  const variance = recent.reduce((a, b) => a + (b - mean) ** 2, 0) / recent.length;
  const coeffOfVariation = mean !== 0 ? Math.sqrt(variance) / Math.abs(mean) : 0;

  const threshold = Math.abs(mean) * 0.05; // 5% change counts as a trend
  let direction = 'flat';
  if (delta > threshold) direction = 'rising';
  else if (delta < -threshold) direction = 'falling';

  return {
    direction,
    unstable: coeffOfVariation > 0.3, // >30% CoV = unstable
  };
};

export const PhaseProvider = ({ children, ros, rosConnected }) => {
  const {
    genesisConnected,
    bridgeConnected,
    trainingMetrics,
    genesisMode,
    blendAlpha,
    deadmanActive,
    safetyFlags,
    actorTag,
    isPaused,
    scriptStatus,
    scriptError,
    currentFrame,
    frameStats,
  } = useGenesis();

  // Reward history for trend computation
  const rewardHistoryRef = useRef([]);
  const [rewardTrend, setRewardTrend] = useState({ direction: 'flat', unstable: false });

  // Last-message timestamps for health pulse
  const rosLastMsgRef = useRef(0);
  const genesisLastFrameRef = useRef(0);
  const trainLastMetricRef = useRef(0);

  const [rosHealth, setRosHealth] = useState('dead');
  const [genesisHealth, setGenesisHealth] = useState('dead');
  const [trainHealth, setTrainHealth] = useState('dead');

  // Track ROS message timestamps
  useEffect(() => {
    if (rosConnected) {
      rosLastMsgRef.current = Date.now();
      setRosHealth('ok');
    } else {
      setRosHealth('dead');
    }
  }, [rosConnected]);

  // Track Genesis frame timestamps
  useEffect(() => {
    if (currentFrame) {
      genesisLastFrameRef.current = Date.now();
      setGenesisHealth('ok');
    }
  }, [currentFrame]);

  // Track training metrics timestamps + reward history
  useEffect(() => {
    if (trainingMetrics) {
      trainLastMetricRef.current = Date.now();
      setTrainHealth('ok');

      if (trainingMetrics.total_reward != null) {
        rewardHistoryRef.current = [
          ...rewardHistoryRef.current.slice(-99),
          trainingMetrics.total_reward,
        ];
        setRewardTrend(computeTrend(rewardHistoryRef.current));
      }
    }
  }, [trainingMetrics]);

  // Health degradation check (run every second)
  useEffect(() => {
    const interval = setInterval(() => {
      const now = Date.now();

      // ROS health
      if (!rosConnected) {
        setRosHealth('dead');
      } else if (now - rosLastMsgRef.current > 5000) {
        setRosHealth('degraded');
      } else {
        setRosHealth('ok');
      }

      // Genesis health
      if (!genesisConnected && !bridgeConnected) {
        setGenesisHealth('dead');
      } else if (genesisConnected && now - genesisLastFrameRef.current > 3000) {
        setGenesisHealth('degraded');
      } else if (genesisConnected) {
        setGenesisHealth('ok');
      }

      // Training health
      const isTraining = scriptStatus === 'running' &&
        (genesisMode === 'hil_blend' || genesisMode === 'online_finetune');
      if (!isTraining) {
        setTrainHealth(trainingMetrics ? 'ok' : 'dead');
      } else if (now - trainLastMetricRef.current > 10000) {
        setTrainHealth('degraded');
      }
    }, 1000);

    return () => clearInterval(interval);
  }, [rosConnected, genesisConnected, bridgeConnected, scriptStatus, genesisMode, trainingMetrics]);

  // Compute authority
  let authority = 'human';
  if (blendAlpha >= 0.95) authority = 'policy';
  else if (blendAlpha > 0.05) authority = 'blend';

  // Compute active phase
  let activePhase = 'idle';
  if (genesisMode === 'eval') {
    activePhase = 'eval';
  } else if (scriptStatus === 'running' &&
    (genesisMode === 'hil_blend' || genesisMode === 'online_finetune')) {
    activePhase = 'train';
  } else if (rosConnected || genesisConnected) {
    activePhase = 'teleop';
  }

  // Compute safety clamp count from training metrics
  const safetyClampCount = trainingMetrics?.safety_clamp_count ?? 0;
  const safetyClampActive = Object.values(safetyFlags).some(Boolean);

  // Eval risk level
  let evalRisk = 'nominal';
  if (safetyClampActive && safetyClampCount >= 3) evalRisk = 'anomaly';
  else if (safetyClampActive) evalRisk = 'clamped';

  // Is recording?
  const isRecording = genesisMode === 'teleop_record' && scriptStatus === 'running';
  const recordingStep = trainingMetrics?.step_count ?? null;
  const recordingTotal = trainingMetrics?.total_steps ?? null;

  // Training state
  const trainEpoch = trainingMetrics?.epoch ?? null;
  const trainTotalEpochs = trainingMetrics?.total_epochs ?? null;
  const trainReward = trainingMetrics?.total_reward ?? null;

  // Error surfacing: pick the most relevant error
  let stageError = null;
  if (scriptError) {
    stageError = scriptError;
  }

  const value = {
    activePhase,
    authority,
    blendAlpha,

    stages: {
      teleop: {
        health: rosConnected ? rosHealth : (genesisConnected ? genesisHealth : 'dead'),
        actor: actorTag,
        deadmanActive,
        recording: isRecording,
        recordingStep,
        recordingTotal,
      },
      train: {
        health: trainHealth,
        trend: rewardTrend.direction,
        unstable: rewardTrend.unstable,
        reward: trainReward,
        epoch: trainEpoch,
        totalEpochs: trainTotalEpochs,
        paused: isPaused,
        error: scriptStatus === 'error' ? scriptError : null,
      },
      eval: {
        health: genesisMode === 'eval' ? genesisHealth : 'dead',
        riskLevel: evalRisk,
        clampCount: safetyClampCount,
        safetyFlags,
        error: genesisMode === 'eval' && scriptError ? scriptError : null,
      },
    },

    // Connection state (for ViewerLayer source selection)
    rosConnected: !!rosConnected,
    genesisConnected,
    bridgeConnected,
  };

  return (
    <PhaseContext.Provider value={value}>
      {children}
    </PhaseContext.Provider>
  );
};
```

**Step 2: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds (PhaseContext not imported anywhere yet, but file should parse)

**Step 3: Commit**

```bash
git add src/client/contexts/PhaseContext.jsx
git commit -m "feat: add PhaseContext with usePhase hook for pipeline state"
```

---

### Task 3: Create ViewerLayer component

Auto-selects between Three.js 3D (RobotViewer) and sim stream (SimViewer) based on connection state. Manages PiP thumbnails.

**Files:**
- Create: `src/client/components/ViewerLayer.jsx`
- Create: `src/client/styles/ViewerLayer.css`

**Step 1: Create ViewerLayer.css**

```css
/* ===== ViewerLayer ===== */
.viewer-layer {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
  background-color: var(--color-bg-primary);
  overflow: hidden;
  z-index: var(--z-base);
}

.viewer-layer__primary {
  position: absolute;
  top: 0;
  left: 0;
  width: 100%;
  height: 100%;
}

/* PiP thumbnail */
.viewer-layer__pip {
  position: absolute;
  bottom: var(--space-9);
  left: var(--space-4);
  width: 240px;
  height: 180px;
  border-radius: var(--radius-md);
  border: var(--border-width) solid var(--color-glass-border);
  overflow: hidden;
  cursor: pointer;
  z-index: var(--z-panel);
  box-shadow: var(--shadow-lg);
  transition: transform var(--transition-base), opacity var(--transition-base);
}

.viewer-layer__pip:hover {
  transform: scale(1.05);
  border-color: var(--color-glass-border-hover);
}

.viewer-layer__pip > * {
  width: 100%;
  height: 100%;
  pointer-events: none;
}

/* Idle state — dark with subtle message */
.viewer-layer__idle {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  width: 100%;
  height: 100%;
  color: var(--color-text-muted);
  font-family: var(--font-primary);
  gap: var(--space-3);
}

.viewer-layer__idle-label {
  font-size: var(--font-size-base);
  letter-spacing: 0.05em;
  text-transform: uppercase;
}

.viewer-layer__idle-hint {
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
  opacity: 0.6;
}

/* Responsive: smaller PiP on small screens */
@media (max-width: 768px) {
  .viewer-layer__pip {
    width: 160px;
    height: 120px;
    bottom: var(--space-7);
    left: var(--space-2);
  }
}
```

**Step 2: Create ViewerLayer.jsx**

```jsx
import React, { useState, useCallback } from 'react';
import { usePhase } from '../contexts/PhaseContext';
import { useGenesis } from '../contexts/GenesisContext';
import RobotViewer from './RobotViewer';
import SimViewer from './SimViewer';
import '../styles/ViewerLayer.css';

const ViewerLayer = ({ ros, appSettings }) => {
  const { rosConnected, genesisConnected } = usePhase();
  const { currentFrame, mediaStream, streamBackend } = useGenesis();

  // Track which source is primary
  // Default: genesis takes priority when connected (richer feed)
  const [primaryOverride, setPrimaryOverride] = useState(null);

  const hasGenesis = genesisConnected && (currentFrame || mediaStream);
  const hasRos = rosConnected;

  let primarySource = 'idle';
  if (primaryOverride) {
    primarySource = primaryOverride;
  } else if (hasGenesis) {
    primarySource = 'genesis';
  } else if (hasRos) {
    primarySource = 'ros';
  }

  // Show PiP when both sources are available and one isn't primary
  const showPip = hasGenesis && hasRos;
  const pipSource = primarySource === 'genesis' ? 'ros' : 'genesis';

  // Swap primary/pip on PiP click
  const handlePipClick = useCallback(() => {
    setPrimaryOverride(prev => {
      if (prev === 'genesis') return 'ros';
      if (prev === 'ros') return 'genesis';
      // If no override, swap from auto-selected
      return primarySource === 'genesis' ? 'ros' : 'genesis';
    });
  }, [primarySource]);

  const renderSource = (source, isPip = false) => {
    if (source === 'genesis') {
      return <SimViewer />;
    }
    if (source === 'ros') {
      return (
        <RobotViewer
          ros={ros}
          tfThrottleRate={appSettings?.visualization?.tfThrottleRate ?? 10}
          settings={appSettings}
        />
      );
    }
    return null;
  };

  return (
    <div className="viewer-layer">
      {/* Primary viewer */}
      <div className="viewer-layer__primary">
        {primarySource === 'idle' ? (
          <div className="viewer-layer__idle">
            <div className="viewer-layer__idle-label">SDR_OS</div>
            <div className="viewer-layer__idle-hint">Waiting for connections...</div>
          </div>
        ) : (
          renderSource(primarySource)
        )}
      </div>

      {/* PiP thumbnail */}
      {showPip && (
        <div className="viewer-layer__pip" onClick={handlePipClick} title="Click to swap">
          {renderSource(pipSource, true)}
        </div>
      )}
    </div>
  );
};

export default ViewerLayer;
```

**Step 3: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 4: Commit**

```bash
git add src/client/components/ViewerLayer.jsx src/client/styles/ViewerLayer.css
git commit -m "feat: add ViewerLayer with auto-source selection and PiP"
```

---

### Task 4: Create EdgePanel container component

Reusable slide-in panel from left/right/bottom with peek (hold) and pin (tap) modes.

**Files:**
- Create: `src/client/components/EdgePanel.jsx`
- Create: `src/client/styles/EdgePanel.css`

**Step 1: Create EdgePanel.css**

```css
/* ===== EdgePanel ===== */
.edge-panel {
  position: fixed;
  z-index: var(--z-overlay);
  background: var(--color-glass-bg);
  border: var(--border-width) solid var(--color-glass-border);
  backdrop-filter: blur(var(--backdrop-blur));
  -webkit-backdrop-filter: blur(var(--backdrop-blur));
  box-shadow: var(--shadow-lg);
  transition: transform var(--panel-slide-duration);
  overflow-y: auto;
  overflow-x: hidden;
}

/* Scrollbar styling */
.edge-panel::-webkit-scrollbar {
  width: 4px;
}

.edge-panel::-webkit-scrollbar-thumb {
  background: var(--color-glass-border-hover);
  border-radius: 2px;
}

/* Left panel */
.edge-panel--left {
  top: var(--strip-height);
  left: 0;
  bottom: 0;
  width: var(--panel-width);
  border-left: none;
  border-top: none;
  border-bottom: none;
  border-radius: 0 var(--radius-md) var(--radius-md) 0;
  transform: translateX(-100%);
}

.edge-panel--left.edge-panel--open {
  transform: translateX(0);
}

/* Right panel */
.edge-panel--right {
  top: var(--strip-height);
  right: 0;
  bottom: 0;
  width: var(--panel-width);
  border-right: none;
  border-top: none;
  border-bottom: none;
  border-radius: var(--radius-md) 0 0 var(--radius-md);
  transform: translateX(100%);
}

.edge-panel--right.edge-panel--open {
  transform: translateX(0);
}

/* Bottom panel */
.edge-panel--bottom {
  bottom: 0;
  left: 0;
  right: 0;
  max-height: 50vh;
  border-bottom: none;
  border-left: none;
  border-right: none;
  border-radius: var(--radius-md) var(--radius-md) 0 0;
  transform: translateY(100%);
}

.edge-panel--bottom.edge-panel--open {
  transform: translateY(0);
}

/* Panel content */
.edge-panel__content {
  padding: var(--space-4);
}

/* Pin indicator */
.edge-panel__pin {
  position: absolute;
  top: var(--space-3);
  right: var(--space-3);
  width: 24px;
  height: 24px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: none;
  border: var(--border-width) solid var(--color-glass-border);
  border-radius: var(--radius-sm);
  color: var(--color-text-muted);
  cursor: pointer;
  font-size: var(--font-size-xs);
  transition: all var(--transition-fast);
}

.edge-panel__pin:hover {
  border-color: var(--color-glass-border-hover);
  color: var(--color-text-primary);
}

.edge-panel__pin--active {
  background: rgba(91, 141, 239, 0.15);
  border-color: var(--color-accent-blue);
  color: var(--color-accent-blue);
}

/* Left panel pin goes top-right */
.edge-panel--left .edge-panel__pin {
  right: var(--space-3);
}

/* Right panel pin goes top-left */
.edge-panel--right .edge-panel__pin {
  right: auto;
  left: var(--space-3);
}

/* Scrim behind panel when open (click to dismiss) */
.edge-panel__scrim {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  z-index: calc(var(--z-overlay) - 1);
  /* Transparent — just catches clicks */
}

/* Responsive: full width on mobile */
@media (max-width: 768px) {
  .edge-panel--left,
  .edge-panel--right {
    width: 100%;
    border-radius: 0;
  }
}
```

**Step 2: Create EdgePanel.jsx**

```jsx
import React, { useState, useCallback, useEffect } from 'react';
import '../styles/EdgePanel.css';

const EdgePanel = ({ side = 'left', isOpen, onClose, pinned, onTogglePin, children }) => {
  const handleScrimClick = useCallback(() => {
    if (!pinned && onClose) {
      onClose();
    }
  }, [pinned, onClose]);

  // Close on Escape
  useEffect(() => {
    if (!isOpen) return;

    const handleKey = (e) => {
      if (e.key === 'Escape' && onClose) {
        onClose();
      }
    };
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, [isOpen, onClose]);

  return (
    <>
      {/* Scrim to catch outside clicks (only when open and not pinned) */}
      {isOpen && !pinned && (
        <div className="edge-panel__scrim" onClick={handleScrimClick} />
      )}

      <div
        className={`edge-panel edge-panel--${side}${isOpen ? ' edge-panel--open' : ''}`}
        role="complementary"
        aria-hidden={!isOpen}
      >
        {/* Pin button */}
        {onTogglePin && (
          <button
            className={`edge-panel__pin${pinned ? ' edge-panel__pin--active' : ''}`}
            onClick={onTogglePin}
            title={pinned ? 'Unpin panel' : 'Pin panel open'}
          >
            {pinned ? '▪' : '▫'}
          </button>
        )}

        <div className="edge-panel__content">
          {children}
        </div>
      </div>
    </>
  );
};

export default EdgePanel;
```

**Step 3: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 4: Commit**

```bash
git add src/client/components/EdgePanel.jsx src/client/styles/EdgePanel.css
git commit -m "feat: add EdgePanel container with slide-in, peek/pin, and scrim"
```

---

### Task 5: Create TrustStrip component (static version)

Initial version with stage cells, E-STOP, and authority tint. Health pulse and adaptive sizing come in Phase 2.

**Files:**
- Create: `src/client/components/TrustStrip.jsx`
- Create: `src/client/styles/TrustStrip.css`

**Step 1: Create TrustStrip.css**

```css
/* ===== TrustStrip ===== */
.trust-strip {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  height: var(--strip-height);
  z-index: var(--z-modal);
  display: flex;
  align-items: center;
  gap: var(--space-2);
  padding: 0 var(--space-3);
  border-bottom: var(--border-width) solid var(--color-glass-border);
  backdrop-filter: blur(var(--backdrop-blur));
  -webkit-backdrop-filter: blur(var(--backdrop-blur));
  box-shadow: var(--shadow-md);
  transition: background var(--transition-slow);
}

/* Authority tint (background color wash) */
.trust-strip--human {
  background: linear-gradient(180deg,
    rgba(62, 207, 142, 0.08) 0%,
    var(--color-glass-bg) 100%);
}

.trust-strip--policy {
  background: linear-gradient(180deg,
    rgba(91, 141, 239, 0.08) 0%,
    var(--color-glass-bg) 100%);
}

.trust-strip--blend {
  background: linear-gradient(180deg,
    rgba(62, 207, 142, 0.06) 0%,
    rgba(91, 141, 239, 0.06) 50%,
    var(--color-glass-bg) 100%);
}

.trust-strip--idle {
  background: var(--color-glass-bg);
}

/* ===== Stage Cell ===== */
.stage-cell {
  display: flex;
  align-items: center;
  gap: var(--space-2);
  padding: var(--space-1) var(--space-3);
  border-radius: var(--radius-sm);
  cursor: pointer;
  transition: all var(--transition-base);
  min-width: var(--strip-cell-min);
  white-space: nowrap;
  position: relative;
}

.stage-cell:hover {
  background: rgba(255, 255, 255, var(--opacity-hover));
}

/* Collapsed cell (just the dot) */
.stage-cell--collapsed {
  padding: var(--space-1);
  min-width: auto;
}

.stage-cell--collapsed .stage-cell__label,
.stage-cell--collapsed .stage-cell__signal {
  display: none;
}

/* Expanded cell (priority promotion) */
.stage-cell--expanded {
  flex-grow: 1;
  max-width: var(--strip-cell-expanded);
  border: var(--border-width) solid var(--color-glass-border-hover);
  background: rgba(255, 255, 255, 0.03);
}

/* Health dot */
.stage-cell__dot {
  width: 8px;
  height: 8px;
  border-radius: 50%;
  flex-shrink: 0;
  transition: background var(--transition-fast);
}

.stage-cell__dot--ok {
  background: var(--color-health-ok);
  box-shadow: 0 0 6px rgba(62, 207, 142, 0.4);
}

.stage-cell__dot--degraded {
  background: var(--color-health-degraded);
  box-shadow: 0 0 6px rgba(240, 180, 41, 0.4);
}

.stage-cell__dot--dead {
  background: var(--color-health-dead);
  opacity: 0.5;
}

/* Health pulse animation */
@keyframes pulse-ok {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.6; }
}

.stage-cell__dot--ok {
  animation: pulse-ok 2s ease-in-out infinite;
}

.stage-cell__dot--degraded {
  animation: pulse-ok 4s ease-in-out infinite;
}

/* Stage label */
.stage-cell__label {
  font-size: var(--font-size-xs);
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--color-text-secondary);
  font-family: var(--font-primary);
}

/* Signal text */
.stage-cell__signal {
  font-size: var(--font-size-xs);
  font-family: var(--font-mono);
  color: var(--color-text-primary);
}

/* Error text inline */
.stage-cell__error {
  font-size: var(--font-size-xs);
  color: var(--color-accent-red);
  font-family: var(--font-mono);
  overflow: hidden;
  text-overflow: ellipsis;
  max-width: 160px;
}

/* Trend arrows */
.stage-cell__trend {
  font-size: var(--font-size-sm);
}

.stage-cell__trend--rising { color: var(--color-accent-green); }
.stage-cell__trend--falling { color: var(--color-accent-red); }
.stage-cell__trend--flat { color: var(--color-text-muted); }

/* Wobble animation for unstable trends */
@keyframes wobble {
  0%, 100% { transform: rotate(0deg); }
  25% { transform: rotate(-5deg); }
  75% { transform: rotate(5deg); }
}

.stage-cell__trend--unstable {
  animation: wobble 0.5s ease-in-out infinite;
}

/* Risk badge */
.stage-cell__risk {
  font-size: var(--font-size-xs);
  padding: 1px 6px;
  border-radius: var(--radius-sm);
  font-weight: 600;
}

.stage-cell__risk--nominal {
  color: var(--color-accent-green);
}

.stage-cell__risk--clamped {
  color: var(--color-accent-amber);
  background: rgba(240, 180, 41, 0.15);
}

.stage-cell__risk--anomaly {
  color: var(--color-accent-red);
  background: rgba(239, 83, 80, 0.15);
}

/* Divider between cells */
.trust-strip__divider {
  width: 1px;
  height: 20px;
  background: var(--color-glass-border);
  flex-shrink: 0;
}

/* Spacer */
.trust-strip__spacer {
  flex: 1 1 auto;
}

/* E-STOP */
.trust-strip__estop {
  min-width: 72px;
  height: 32px;
  background: var(--color-accent-red);
  color: #fff;
  border: none;
  border-radius: var(--radius-sm);
  font-size: var(--font-size-xs);
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  cursor: pointer;
  font-family: var(--font-primary);
  flex-shrink: 0;
  min-height: var(--touch-min);
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all var(--transition-fast);
}

.trust-strip__estop:hover {
  filter: brightness(1.15);
}

.trust-strip__estop:active {
  filter: brightness(0.9);
}
```

**Step 2: Create TrustStrip.jsx**

```jsx
import React from 'react';
import { usePhase } from '../contexts/PhaseContext';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/TrustStrip.css';

const TREND_ARROWS = {
  rising: '\u2197',   // ↗
  falling: '\u2198',  // ↘
  flat: '\u2192',     // →
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

          {/* Stage-specific signal */}
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
                {TREND_ARROWS[data.trend] || '→'}
              </span>
              <span className="stage-cell__signal">
                {data.reward != null ? data.reward.toFixed(1) + 'r' : ''}
              </span>
            </>
          )}

          {stage === 'eval' && data.health !== 'dead' && (
            <span className={`stage-cell__risk stage-cell__risk--${data.riskLevel}`}>
              {data.riskLevel === 'nominal' ? 'nominal' :
                `${data.riskLevel}${data.clampCount > 0 ? ` ×${data.clampCount}` : ''}`}
            </span>
          )}

          {/* Inline error */}
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

const TrustStrip = ({ onStageClick, onEstop }) => {
  const { activePhase, authority, stages } = usePhase();
  const { pause } = useGenesis();

  const stripClass = `trust-strip trust-strip--${authority}`;

  // Collapse cells that aren't active and have dead health
  const isTeleopCollapsed = stages.teleop.health === 'dead' && activePhase !== 'teleop';
  const isTrainCollapsed = stages.train.health === 'dead' && activePhase !== 'train';
  const isEvalCollapsed = stages.eval.health === 'dead' && activePhase !== 'eval';

  const handleEstop = () => {
    // Pause simulation and fire callback
    pause(true);
    if (onEstop) onEstop();
  };

  return (
    <div className={stripClass}>
      <StageCell
        stage="teleop"
        data={stages.teleop}
        collapsed={isTeleopCollapsed}
        onClick={() => onStageClick?.('teleop')}
      />

      <div className="trust-strip__divider" />

      <StageCell
        stage="train"
        data={stages.train}
        collapsed={isTrainCollapsed}
        onClick={() => onStageClick?.('train')}
      />

      <div className="trust-strip__divider" />

      <StageCell
        stage="eval"
        data={stages.eval}
        collapsed={isEvalCollapsed}
        onClick={() => onStageClick?.('eval')}
      />

      <div className="trust-strip__spacer" />

      <button className="trust-strip__estop" onClick={handleEstop}>
        E-STOP
      </button>
    </div>
  );
};

export default TrustStrip;
```

**Step 3: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 4: Commit**

```bash
git add src/client/components/TrustStrip.jsx src/client/styles/TrustStrip.css
git commit -m "feat: add TrustStrip with stage cells, authority tint, and E-STOP"
```

---

### Task 6: Create CommandBar component

Bottom edge panel with command input, autocomplete, and recent commands.

**Files:**
- Create: `src/client/components/CommandBar.jsx`
- Create: `src/client/styles/CommandBar.css`

**Step 1: Create CommandBar.css**

```css
/* ===== CommandBar ===== */
.command-bar {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
}

.command-bar__input-row {
  display: flex;
  align-items: center;
  gap: var(--space-2);
}

.command-bar__input {
  flex: 1;
  height: var(--touch-min);
  background: rgba(0, 0, 0, 0.3);
  border: var(--border-width) solid var(--color-glass-border);
  border-radius: var(--radius-md);
  padding: 0 var(--space-4);
  color: var(--color-text-primary);
  font-family: var(--font-mono);
  font-size: var(--font-size-sm);
  outline: none;
  transition: border-color var(--transition-fast);
}

.command-bar__input:focus {
  border-color: var(--color-accent-blue);
  box-shadow: var(--focus-ring);
}

.command-bar__input::placeholder {
  color: var(--color-text-muted);
}

/* Suggestions dropdown */
.command-bar__suggestions {
  display: flex;
  flex-direction: column;
  gap: 1px;
  max-height: 200px;
  overflow-y: auto;
}

.command-bar__suggestion {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: var(--space-2) var(--space-3);
  background: rgba(0, 0, 0, 0.2);
  border-radius: var(--radius-sm);
  cursor: pointer;
  transition: background var(--transition-fast);
  min-height: var(--touch-min);
}

.command-bar__suggestion:hover,
.command-bar__suggestion--selected {
  background: rgba(255, 255, 255, var(--opacity-hover));
}

.command-bar__suggestion-label {
  font-family: var(--font-mono);
  font-size: var(--font-size-sm);
  color: var(--color-text-primary);
}

.command-bar__suggestion-desc {
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
}

/* Recent commands */
.command-bar__recent {
  display: flex;
  flex-wrap: wrap;
  gap: var(--space-2);
  padding-top: var(--space-2);
  border-top: var(--border-width) solid var(--color-glass-border);
}

.command-bar__recent-label {
  font-size: var(--font-size-xs);
  color: var(--color-text-muted);
  width: 100%;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.command-bar__recent-item {
  padding: var(--space-1) var(--space-3);
  background: rgba(0, 0, 0, 0.2);
  border: var(--border-width) solid var(--color-glass-border);
  border-radius: var(--radius-sm);
  font-family: var(--font-mono);
  font-size: var(--font-size-xs);
  color: var(--color-text-secondary);
  cursor: pointer;
  transition: all var(--transition-fast);
  min-height: var(--touch-min);
  display: flex;
  align-items: center;
}

.command-bar__recent-item:hover {
  border-color: var(--color-glass-border-hover);
  color: var(--color-text-primary);
}
```

**Step 2: Create CommandBar.jsx**

```jsx
import React, { useState, useRef, useCallback, useEffect } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/CommandBar.css';

// Registry of available commands
const COMMANDS = [
  { cmd: 'load robot', args: '<name>', desc: 'Load a robot by name', action: 'loadRobot' },
  { cmd: 'unload robot', args: '', desc: 'Unload current robot', action: 'unloadRobot' },
  { cmd: 'reset', args: '', desc: 'Reset environment', action: 'reset' },
  { cmd: 'pause', args: '', desc: 'Pause simulation', action: 'pause' },
  { cmd: 'resume', args: '', desc: 'Resume simulation', action: 'resume' },
  { cmd: 'set mode', args: '<mode>', desc: 'Set pipeline mode', action: 'setMode' },
  { cmd: 'set alpha', args: '<0-1>', desc: 'Set blend alpha', action: 'setAlpha' },
  { cmd: 'estop', args: '', desc: 'Emergency stop', action: 'estop' },
];

const CommandBar = ({ onClose, onOpenSettings }) => {
  const [input, setInput] = useState('');
  const [selectedIdx, setSelectedIdx] = useState(0);
  const [recentCommands, setRecentCommands] = useState(() => {
    try {
      return JSON.parse(localStorage.getItem('sdr_recent_commands') || '[]');
    } catch { return []; }
  });
  const inputRef = useRef(null);
  const { loadRobot, unloadRobot, reset, pause, setMode, setAlpha, robotList } = useGenesis();

  // Auto-focus input
  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  // Filter commands based on input
  const suggestions = input.length === 0
    ? COMMANDS
    : COMMANDS.filter(c =>
        c.cmd.includes(input.toLowerCase()) ||
        c.desc.toLowerCase().includes(input.toLowerCase())
      );

  const executeCommand = useCallback((cmdStr) => {
    const trimmed = cmdStr.trim().toLowerCase();

    // Save to recent
    const updated = [cmdStr, ...recentCommands.filter(c => c !== cmdStr)].slice(0, 8);
    setRecentCommands(updated);
    localStorage.setItem('sdr_recent_commands', JSON.stringify(updated));

    // Parse and execute
    if (trimmed.startsWith('load robot ')) {
      const name = cmdStr.trim().slice('load robot '.length).trim();
      loadRobot(name);
    } else if (trimmed === 'unload robot') {
      unloadRobot();
    } else if (trimmed === 'reset') {
      reset();
    } else if (trimmed === 'pause') {
      pause(true);
    } else if (trimmed === 'resume') {
      pause(false);
    } else if (trimmed === 'estop') {
      pause(true);
    } else if (trimmed.startsWith('set mode ')) {
      const mode = cmdStr.trim().slice('set mode '.length).trim();
      setMode(mode);
    } else if (trimmed.startsWith('set alpha ')) {
      const alpha = parseFloat(cmdStr.trim().slice('set alpha '.length));
      if (!isNaN(alpha)) setAlpha(alpha);
    } else if (trimmed === 'settings') {
      if (onOpenSettings) onOpenSettings();
    }

    setInput('');
    if (onClose) onClose();
  }, [recentCommands, loadRobot, unloadRobot, reset, pause, setMode, setAlpha, onClose, onOpenSettings]);

  const handleKeyDown = useCallback((e) => {
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      setSelectedIdx(i => Math.min(i + 1, suggestions.length - 1));
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      setSelectedIdx(i => Math.max(i - 1, 0));
    } else if (e.key === 'Enter') {
      e.preventDefault();
      if (input.trim()) {
        executeCommand(input);
      } else if (suggestions[selectedIdx]) {
        setInput(suggestions[selectedIdx].cmd + ' ');
      }
    } else if (e.key === 'Tab' && suggestions[selectedIdx]) {
      e.preventDefault();
      setInput(suggestions[selectedIdx].cmd + ' ');
    } else if (e.key === 'Escape') {
      if (onClose) onClose();
    }
  }, [input, suggestions, selectedIdx, executeCommand, onClose]);

  return (
    <div className="command-bar">
      <div className="command-bar__input-row">
        <input
          ref={inputRef}
          className="command-bar__input"
          type="text"
          value={input}
          onChange={(e) => { setInput(e.target.value); setSelectedIdx(0); }}
          onKeyDown={handleKeyDown}
          placeholder="Type a command..."
          autoComplete="off"
          spellCheck={false}
        />
      </div>

      {/* Suggestions */}
      {suggestions.length > 0 && (
        <div className="command-bar__suggestions">
          {suggestions.map((s, i) => (
            <div
              key={s.cmd}
              className={`command-bar__suggestion${i === selectedIdx ? ' command-bar__suggestion--selected' : ''}`}
              onClick={() => {
                if (s.args) {
                  setInput(s.cmd + ' ');
                  inputRef.current?.focus();
                } else {
                  executeCommand(s.cmd);
                }
              }}
            >
              <span className="command-bar__suggestion-label">
                {s.cmd}{s.args ? ` ${s.args}` : ''}
              </span>
              <span className="command-bar__suggestion-desc">{s.desc}</span>
            </div>
          ))}
        </div>
      )}

      {/* Recent commands */}
      {recentCommands.length > 0 && (
        <div className="command-bar__recent">
          <span className="command-bar__recent-label">Recent</span>
          {recentCommands.map((cmd, i) => (
            <div
              key={i}
              className="command-bar__recent-item"
              onClick={() => executeCommand(cmd)}
            >
              {cmd}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default CommandBar;
```

**Step 3: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 4: Commit**

```bash
git add src/client/components/CommandBar.jsx src/client/styles/CommandBar.css
git commit -m "feat: add CommandBar with autocomplete, recent commands, and action dispatch"
```

---

### Task 7: Wire everything together in App.jsx

Replace the current mode-switching architecture with the cockpit layout. This is the big integration step.

**Files:**
- Modify: `src/client/components/App.jsx`
- Modify: `src/client/styles/App.css`

**Step 1: Rewrite App.css**

Replace the entire file:

```css
/* ===== App Layout (Cockpit) ===== */
.app {
  position: relative;
  height: 100vh;
  width: 100vw;
  overflow: hidden;
  background-color: var(--color-bg-primary);
  font-family: var(--font-primary);
}

/* Error banner floats above everything */
.app__error-banner {
  position: fixed;
  top: calc(var(--strip-height) + var(--space-3));
  left: 50%;
  transform: translateX(-50%);
  background: rgba(239, 83, 80, 0.15);
  border: var(--border-width) solid var(--color-accent-red);
  padding: var(--space-3) var(--space-5);
  border-radius: var(--radius-md);
  color: var(--color-text-primary);
  z-index: var(--z-modal);
  display: flex;
  align-items: center;
  gap: var(--space-5);
  backdrop-filter: blur(var(--backdrop-blur));
  font-size: var(--font-size-sm);
}

.app__error-banner button {
  background: transparent;
  border: var(--border-width) solid var(--color-glass-border-hover);
  color: var(--color-text-primary);
  padding: var(--space-1) var(--space-4);
  border-radius: var(--radius-sm);
  cursor: pointer;
  font-family: var(--font-primary);
  font-size: var(--font-size-sm);
  transition: background var(--transition-fast);
}

.app__error-banner button:hover {
  background: rgba(255, 255, 255, var(--opacity-hover));
}
```

**Step 2: Rewrite App.jsx**

Replace the entire file:

```jsx
import React, { useEffect, useState, useCallback } from 'react';
import SplashScreen from './SplashScreen';
import ViewerLayer from './ViewerLayer';
import TrustStrip from './TrustStrip';
import EdgePanel from './EdgePanel';
import CommandBar from './CommandBar';
import ControlOverlay from './ControlOverlay';
import TelemetryPanel from './TelemetryPanel';
import LogViewer from './LogViewer';
import SettingsModal from './SettingsModal';
import EvalControlPanel from './EvalControlPanel';
import { SettingsProvider, useSettings } from '../contexts/SettingsContext';
import { GenesisProvider, useGenesis } from '../contexts/GenesisContext';
import { PhaseProvider, usePhase } from '../contexts/PhaseContext';
import ROSLIB from 'roslib';
import io from 'socket.io-client';
import '../styles/App.css';

const CockpitContent = ({ socket }) => {
  const [isLoading, setIsLoading] = useState(true);
  const [ros, setRos] = useState(null);
  const [rosConnected, setRosConnected] = useState(false);
  const [error, setError] = useState(null);
  const [appSettings, setAppSettings] = useState(null);

  // Edge panel state
  const [leftOpen, setLeftOpen] = useState(false);
  const [leftPinned, setLeftPinned] = useState(false);
  const [rightOpen, setRightOpen] = useState(false);
  const [rightPinned, setRightPinned] = useState(false);
  const [bottomOpen, setBottomOpen] = useState(false);
  const [settingsOpen, setSettingsOpen] = useState(false);

  // Control state for ControlOverlay
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });

  const { getSetting } = useSettings();

  const handleControlChange = useCallback((newState) => {
    setControlState(newState);
  }, []);

  // Initialize connections
  const initializeConnections = async () => {
    try {
      const rosBridgeUrl = localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090';
      const newRos = new ROSLIB.Ros({ url: rosBridgeUrl });

      let resolved = false;

      newRos.on('connection', () => {
        setRosConnected(true);
        setError(null);
        resolved = true;
      });

      newRos.on('error', () => {
        if (!resolved) { resolved = true; }
      });

      newRos.on('close', () => {
        setRosConnected(false);
      });

      // Wait up to 5s for connection
      await Promise.race([
        new Promise(resolve => setTimeout(resolve, 5000)),
        new Promise(resolve => {
          const check = setInterval(() => {
            if (resolved) { clearInterval(check); resolve(); }
          }, 100);
        })
      ]);

      setRos(newRos);
      return true;
    } catch (err) {
      setError(`Connection failed: ${err.message}`);
      return false;
    }
  };

  const handleSplashComplete = async () => {
    await initializeConnections();
    // Load settings
    try {
      const saved = localStorage.getItem('robotControllerSettings');
      if (saved) setAppSettings(JSON.parse(saved));
    } catch (e) { /* ignore */ }
    setIsLoading(false);
  };

  // Keyboard shortcuts for panels
  useEffect(() => {
    const handleKey = (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      if (e.key === '`' || e.key === '/') {
        e.preventDefault();
        setBottomOpen(prev => !prev);
      }
    };
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, []);

  // Stage cell click handler — opens relevant edge panel
  const handleStageClick = useCallback((stage) => {
    setRightOpen(true);
  }, []);

  const handleSettingsSave = useCallback((newSettings) => {
    setAppSettings(newSettings);
    localStorage.setItem('robotControllerSettings', JSON.stringify(newSettings));
  }, []);

  if (isLoading) {
    return <SplashScreen onComplete={handleSplashComplete} />;
  }

  return (
    <PhaseProvider ros={ros} rosConnected={rosConnected}>
      <div className="app">
        {error && (
          <div className="app__error-banner">
            {error}
            <button onClick={() => window.location.reload()}>Retry</button>
          </div>
        )}

        {/* Layer 1: Fullscreen viewer */}
        <ViewerLayer ros={ros} appSettings={appSettings} />

        {/* Layer 2: Trust strip */}
        <TrustStrip onStageClick={handleStageClick} />

        {/* Layer 3: Edge panels */}

        {/* Left: Controls */}
        <EdgePanel
          side="left"
          isOpen={leftOpen}
          onClose={() => { if (!leftPinned) setLeftOpen(false); }}
          pinned={leftPinned}
          onTogglePin={() => setLeftPinned(prev => !prev)}
        >
          <ControlOverlay
            ros={ros}
            socket={socket}
            controlState={controlState}
            onControlChange={handleControlChange}
          />
        </EdgePanel>

        {/* Right: Details */}
        <EdgePanel
          side="right"
          isOpen={rightOpen}
          onClose={() => { if (!rightPinned) setRightOpen(false); }}
          pinned={rightPinned}
          onTogglePin={() => setRightPinned(prev => !prev)}
        >
          <TelemetryPanel
            ros={ros}
            updateInterval={appSettings?.telemetry?.updateInterval ?? 100}
            initialShowPanel={true}
          />
        </EdgePanel>

        {/* Bottom: Command bar */}
        <EdgePanel
          side="bottom"
          isOpen={bottomOpen}
          onClose={() => setBottomOpen(false)}
        >
          <CommandBar
            onClose={() => setBottomOpen(false)}
            onOpenSettings={() => { setBottomOpen(false); setSettingsOpen(true); }}
          />
        </EdgePanel>

        {/* Settings modal (keeps existing component) */}
        <SettingsModal
          isOpen={settingsOpen}
          onClose={() => setSettingsOpen(false)}
          onSave={handleSettingsSave}
          initialSettings={appSettings}
        />
      </div>
    </PhaseProvider>
  );
};

const AppWithGenesis = () => {
  const [socket, setSocket] = useState(null);

  useEffect(() => {
    const socketUrl = localStorage.getItem('socketUrl') || 'http://localhost:3000';
    const newSocket = io(socketUrl);
    setSocket(newSocket);
    return () => { if (newSocket) newSocket.disconnect(); };
  }, []);

  return (
    <GenesisProvider socket={socket}>
      <CockpitContent socket={socket} />
    </GenesisProvider>
  );
};

const App = () => (
  <SettingsProvider>
    <AppWithGenesis />
  </SettingsProvider>
);

export default App;
```

**Step 3: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds. Some warnings about unused imports from old components are expected and fine.

**Step 4: Smoke test in browser**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm start`
Open `http://localhost:3000`. Expected:
- Splash screen → dark viewer with "SDR_OS / Waiting for connections..."
- Trust strip at top with three stage dots (all red/dead) and E-STOP
- Press `/` or backtick → bottom command bar slides up
- No old HudBar, no bottom toolbar, no mode selector

**Step 5: Commit**

```bash
git add src/client/components/App.jsx src/client/styles/App.css
git commit -m "feat: replace mode-based App with cockpit layout (viewer + strip + panels)"
```

---

## Phase 2: Polish & Touch

### Task 8: Add SimViewer camera controls (keyboard + touch)

Port the Genesis-style camera controls into SimViewer. This is Tasks 2-5 from the earlier camera controls plan, condensed.

**Files:**
- Modify: `src/client/components/SimViewer.jsx`
- Modify: `src/client/styles/SimViewer.css`

**Step 1: Add keyboard shortcuts and touch handlers to SimViewer**

In `SimViewer.jsx`, add after the existing `cameraState` ref (line 42):

```javascript
const defaultCameraState = useRef({
  radius: 3.0, theta: Math.PI / 4, phi: Math.PI / 4,
  target: { x: 0, y: 0, z: 0.5 }, up: { x: 0, y: 0, z: 1 }
});
const [containMode, setContainMode] = useState(false);
const touchStartRef = useRef([]);
const initialPinchDistance = useRef(null);
const initialPinchRadius = useRef(null);
```

Add `resetCamera` after `updateCamera`:

```javascript
const resetCamera = useCallback(() => {
  const d = defaultCameraState.current;
  Object.assign(cameraState.current, {
    radius: d.radius, theta: d.theta, phi: d.phi,
    target: { ...d.target }
  });
  updateCamera();
}, [updateCamera]);
```

Add keyboard handler:

```javascript
const handleKeyDown = useCallback((e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
  switch (e.key.toLowerCase()) {
    case 'z': resetCamera(); break;
    case 'c': setContainMode(prev => !prev); break;
    case 'f':
      if (containerRef.current) {
        document.fullscreenElement ? document.exitFullscreen() : containerRef.current.requestFullscreen();
      }
      break;
  }
}, [resetCamera]);
```

Add touch handlers:

```javascript
const handleTouchStart = useCallback((e) => {
  const touches = Array.from(e.touches);
  touchStartRef.current = touches.map(t => ({ x: t.clientX, y: t.clientY }));
  if (touches.length === 1) {
    setIsDragging(true); setDragMode('orbit');
    lastMousePos.current = { x: touches[0].clientX, y: touches[0].clientY };
  } else if (touches.length === 2) {
    setIsDragging(true);
    const dx = touches[1].clientX - touches[0].clientX;
    const dy = touches[1].clientY - touches[0].clientY;
    initialPinchDistance.current = Math.hypot(dx, dy);
    initialPinchRadius.current = cameraState.current.radius;
    setDragMode('pan');
    lastMousePos.current = {
      x: (touches[0].clientX + touches[1].clientX) / 2,
      y: (touches[0].clientY + touches[1].clientY) / 2
    };
  }
}, []);

const handleTouchMove = useCallback((e) => {
  e.preventDefault();
  const touches = Array.from(e.touches);
  if (touches.length === 2 && initialPinchDistance.current != null) {
    const dx = touches[1].clientX - touches[0].clientX;
    const dy = touches[1].clientY - touches[0].clientY;
    const scale = initialPinchDistance.current / Math.hypot(dx, dy);
    cameraState.current.radius = Math.max(0.5, Math.min(50, initialPinchRadius.current * scale));
    updateCamera();
  } else if (touches.length === 1 && isDragging) {
    const deltaX = touches[0].clientX - lastMousePos.current.x;
    const deltaY = touches[0].clientY - lastMousePos.current.y;
    lastMousePos.current = { x: touches[0].clientX, y: touches[0].clientY };
    cameraState.current.theta -= deltaX * 0.01;
    cameraState.current.phi = Math.max(0.1, Math.min(Math.PI - 0.1, cameraState.current.phi - deltaY * 0.01));
    updateCamera();
  }
}, [isDragging, updateCamera]);

const handleTouchEnd = useCallback((e) => {
  if (e.touches.length === 0) {
    setIsDragging(false); setDragMode(null);
    initialPinchDistance.current = null;
  }
}, []);
```

Register all new listeners in the existing `useEffect` (line 152) alongside mouse listeners:

```javascript
container.addEventListener('touchstart', handleTouchStart, { passive: false });
container.addEventListener('touchmove', handleTouchMove, { passive: false });
container.addEventListener('touchend', handleTouchEnd);
window.addEventListener('keydown', handleKeyDown);
```

And clean them up in the return function. Add to dependency array.

Update container div className:

```jsx
<div className={`sim-viewer${containMode ? ' contain-mode' : ''}`} ref={containerRef}>
```

Update camera hint:

```jsx
{hasVideo && (
  <div className="sim-camera-hint">
    <span>Drag: orbit</span>
    <span>Shift+drag: pan</span>
    <span>Scroll: zoom</span>
    <span>Z: reset</span>
  </div>
)}
```

**Step 2: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 3: Commit**

```bash
git add src/client/components/SimViewer.jsx
git commit -m "feat: add keyboard shortcuts and touch gesture support to SimViewer"
```

---

### Task 9: Add gamepad panel triggers

On Steam Deck, L1/R1 should open/close left/right panels. This listens for gamepad button presses.

**Files:**
- Modify: `src/client/components/App.jsx` (the new CockpitContent)

**Step 1: Add gamepad polling effect**

Inside `CockpitContent`, add after the keyboard shortcut `useEffect`:

```javascript
// Gamepad panel triggers: L1=leftPanel, R1=rightPanel
useEffect(() => {
  let animFrame;
  const prevButtons = { 4: false, 5: false }; // L1=4, R1=5

  const poll = () => {
    const gamepads = navigator.getGamepads?.() || [];
    const gp = gamepads[0];
    if (gp) {
      // L1 (button 4)
      if (gp.buttons[4]?.pressed && !prevButtons[4]) {
        setLeftOpen(prev => !prev);
      }
      prevButtons[4] = gp.buttons[4]?.pressed || false;

      // R1 (button 5)
      if (gp.buttons[5]?.pressed && !prevButtons[5]) {
        setRightOpen(prev => !prev);
      }
      prevButtons[5] = gp.buttons[5]?.pressed || false;
    }
    animFrame = requestAnimationFrame(poll);
  };
  animFrame = requestAnimationFrame(poll);
  return () => cancelAnimationFrame(animFrame);
}, []);
```

**Step 2: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 3: Commit**

```bash
git add src/client/components/App.jsx
git commit -m "feat: add gamepad L1/R1 triggers for edge panels"
```

---

## Phase 3: Cleanup

### Task 10: Remove old mode-based components and dead code

Remove components and styles that are no longer imported.

**Files:**
- Delete: `src/client/components/SimpleSimViewer.jsx`
- Delete: `src/client/components/HudBar.jsx`
- Delete: `src/client/components/GenesisPanelModal.jsx`
- Delete: `src/client/components/StatusModal.jsx`
- Delete: `src/client/components/SimpleModeSelector.jsx` (if exists — check `GenesisModeSelector.jsx`)
- Delete: `src/client/styles/HudBar.css`
- Delete: `src/client/styles/GenesisPanelModal.css`
- Delete: `src/client/styles/StatusModal.css`
- Delete: `src/client/styles/GenesisModeSelector.css`

**Step 1: Verify no imports reference these files**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && grep -r "SimpleSimViewer\|HudBar\|GenesisPanelModal\|StatusModal\|GenesisModeSelector\|SimpleModeSelector" src/client/ --include="*.jsx" --include="*.js" -l`

Expected: Only the files being deleted themselves appear. If any other file imports them, update that file first.

**Step 2: Delete the files**

```bash
cd /home/ethan/dev/Genesis/SDR_OS
rm -f src/client/components/SimpleSimViewer.jsx
rm -f src/client/components/HudBar.jsx
rm -f src/client/components/GenesisPanelModal.jsx
rm -f src/client/components/StatusModal.jsx
rm -f src/client/components/GenesisModeSelector.jsx
rm -f src/client/styles/HudBar.css
rm -f src/client/styles/GenesisPanelModal.css
rm -f src/client/styles/StatusModal.css
rm -f src/client/styles/GenesisModeSelector.css
```

**Step 3: Verify build**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds with no missing module errors

**Step 4: Commit**

```bash
git add -A
git commit -m "cleanup: remove old mode-based components replaced by cockpit layout"
```

---

## Summary

| Task | What | Files Created/Modified |
|------|------|----------------------|
| 1 | Design tokens | theme.css |
| 2 | PhaseContext + usePhase | PhaseContext.jsx (new) |
| 3 | ViewerLayer | ViewerLayer.jsx + .css (new) |
| 4 | EdgePanel | EdgePanel.jsx + .css (new) |
| 5 | TrustStrip | TrustStrip.jsx + .css (new) |
| 6 | CommandBar | CommandBar.jsx + .css (new) |
| 7 | App.jsx rewrite | App.jsx + App.css (rewrite) |
| 8 | SimViewer controls | SimViewer.jsx (modify) |
| 9 | Gamepad panel triggers | App.jsx (modify) |
| 10 | Delete old code | 9 files removed |

**Build after each task.** Each task produces a working (if incomplete) UI. Task 7 is the big integration step — after that, you have a functional cockpit.
