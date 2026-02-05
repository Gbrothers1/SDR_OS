# Training Panel Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build the Training Panel GUI — the core module for REAL→SIM ↔ SIM→REAL RL training pipeline with session carousel, context-sensitive controls, and Steam Deck optimization.

**Architecture:** Training Panel is a 75% overlay triggered by Train button in TrustStrip. Uses TrainingPanelContext for panel state and SessionContext for per-session state. Communicates with bridge_server.py via Socket.io events through GenesisContext.

**Tech Stack:** React 18, Three.js (URDF rendering), Socket.io, CSS (no Tailwind), existing EdgePanel pattern.

**Design Reference:** `docs/plans/2026-02-04-training-panel-design.md`

---

## Phase 1: Foundation (MVP)

Build the minimal vertical slice: panel toggle, one session card, Hot Row with 2 knobs.

---

### Task 1: Create TrainingPanelContext

**Files:**
- Create: `src/client/contexts/TrainingPanelContext.jsx`

**Step 1: Create context file with basic state**

```jsx
import React, { createContext, useContext, useState, useCallback } from 'react';

const TrainingPanelContext = createContext(null);

export function TrainingPanelProvider({ children }) {
  // Panel state
  const [isOpen, setIsOpen] = useState(false);
  const [focusedSessionId, setFocusedSessionId] = useState(null);
  const [focusZone, setFocusZone] = useState('carousel'); // 'carousel' | 'hotrow' | 'advanced'
  const [editingParam, setEditingParam] = useState(null);

  // Toggle panel open/close
  const togglePanel = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  // Open panel
  const openPanel = useCallback(() => {
    setIsOpen(true);
  }, []);

  // Close panel
  const closePanel = useCallback(() => {
    setIsOpen(false);
    setEditingParam(null);
  }, []);

  // Focus navigation
  const navigateFocus = useCallback((direction) => {
    if (direction === 'up') {
      setFocusZone(prev => {
        if (prev === 'carousel') return 'hotrow';
        if (prev === 'hotrow') return 'advanced';
        return prev;
      });
    } else if (direction === 'down') {
      setFocusZone(prev => {
        if (prev === 'advanced') return 'hotrow';
        if (prev === 'hotrow') return 'carousel';
        return prev;
      });
    }
  }, []);

  const value = {
    // State
    isOpen,
    focusedSessionId,
    focusZone,
    editingParam,
    // Actions
    togglePanel,
    openPanel,
    closePanel,
    setFocusedSessionId,
    setFocusZone,
    setEditingParam,
    navigateFocus,
  };

  return (
    <TrainingPanelContext.Provider value={value}>
      {children}
    </TrainingPanelContext.Provider>
  );
}

export function useTrainingPanel() {
  const context = useContext(TrainingPanelContext);
  if (!context) {
    throw new Error('useTrainingPanel must be used within TrainingPanelProvider');
  }
  return context;
}
```

**Step 2: Verify file created**

Run: `ls -la src/client/contexts/TrainingPanelContext.jsx`
Expected: File exists with correct permissions

**Step 3: Commit**

```bash
git add src/client/contexts/TrainingPanelContext.jsx
git commit -m "feat(training): add TrainingPanelContext for panel state management"
```

---

### Task 2: Add TrainingPanelProvider to App

**Files:**
- Modify: `src/client/components/App.jsx`

**Step 1: Import TrainingPanelProvider**

Add import at top of file (around line 8):

```jsx
import { TrainingPanelProvider } from '../contexts/TrainingPanelContext';
```

**Step 2: Wrap AppContent with TrainingPanelProvider**

Find `AppWithGenesis` component (around line 308-320) and add provider:

```jsx
export function AppWithGenesis() {
  return (
    <SettingsProvider>
      <GenesisProvider>
        <TrainingPanelProvider>
          <AppContent />
        </TrainingPanelProvider>
      </GenesisProvider>
    </SettingsProvider>
  );
}
```

**Step 3: Verify app still loads**

Run: `npm run build && npm start`
Expected: App builds without errors, loads in browser

**Step 4: Commit**

```bash
git add src/client/components/App.jsx
git commit -m "feat(training): integrate TrainingPanelProvider into App"
```

---

### Task 3: Create TrainingPanel Component Shell

**Files:**
- Create: `src/client/components/TrainingPanel.jsx`
- Create: `src/client/styles/TrainingPanel.css`

**Step 1: Create CSS file**

```css
/* TrainingPanel.css - Main training panel overlay */

.training-panel {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 60px; /* Leave room for trust-strip */
  background: rgba(15, 15, 20, 0.95);
  z-index: 100;
  display: flex;
  flex-direction: column;
  padding: 16px;
  opacity: 0;
  pointer-events: none;
  transform: translateY(20px);
  transition: opacity 0.2s ease, transform 0.2s ease;
}

.training-panel--open {
  opacity: 1;
  pointer-events: auto;
  transform: translateY(0);
}

.training-panel__header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 16px;
}

.training-panel__title {
  font-size: 18px;
  font-weight: 600;
  color: #fff;
  margin: 0;
}

.training-panel__pip {
  position: absolute;
  top: 16px;
  left: 16px;
  width: 22%;
  aspect-ratio: 16 / 9;
  background: rgba(0, 0, 0, 0.6);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  overflow: hidden;
  display: flex;
  align-items: center;
  justify-content: center;
  color: rgba(255, 255, 255, 0.4);
  font-size: 12px;
}

.training-panel__content {
  flex: 1;
  display: flex;
  flex-direction: column;
  margin-left: calc(22% + 32px); /* Offset for PiP */
}

.training-panel__carousel {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  min-height: 200px;
}

.training-panel__hotrow {
  display: flex;
  gap: 12px;
  padding: 16px;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 8px;
  margin-top: 16px;
}

.training-panel__footer {
  display: flex;
  justify-content: center;
  gap: 24px;
  padding: 12px 0;
  font-size: 12px;
  color: rgba(255, 255, 255, 0.5);
  border-top: 1px solid rgba(255, 255, 255, 0.1);
  margin-top: 16px;
}

.training-panel__footer-hint {
  display: flex;
  align-items: center;
  gap: 6px;
}

.training-panel__footer-key {
  background: rgba(255, 255, 255, 0.1);
  padding: 2px 6px;
  border-radius: 3px;
  font-family: monospace;
}
```

**Step 2: Create component file**

```jsx
import React from 'react';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import '../styles/TrainingPanel.css';

export default function TrainingPanel() {
  const { isOpen, focusZone } = useTrainingPanel();

  // Footer hints based on focus zone
  const getFooterHints = () => {
    switch (focusZone) {
      case 'carousel':
        return [
          { key: 'A', action: 'Select' },
          { key: 'Y', action: 'Peek' },
          { key: 'L2', action: 'Clone' },
        ];
      case 'hotrow':
        return [
          { key: 'A', action: 'Edit' },
          { key: 'Y', action: 'Reset' },
          { key: 'L1', action: 'Fine' },
        ];
      default:
        return [];
    }
  };

  const footerHints = getFooterHints();

  return (
    <div className={`training-panel ${isOpen ? 'training-panel--open' : ''}`}>
      {/* PiP Viewer */}
      <div className="training-panel__pip">
        Episode Preview
      </div>

      {/* Main Content */}
      <div className="training-panel__content">
        {/* Carousel Area */}
        <div className="training-panel__carousel">
          <p style={{ color: 'rgba(255,255,255,0.4)' }}>
            Session Carousel (Coming Soon)
          </p>
        </div>

        {/* Hot Row */}
        <div className="training-panel__hotrow">
          <p style={{ color: 'rgba(255,255,255,0.4)', margin: 0 }}>
            Hot Row: 6 Essential Knobs (Coming Soon)
          </p>
        </div>
      </div>

      {/* Footer with context-aware hints */}
      <div className="training-panel__footer">
        <span style={{ marginRight: 'auto', color: 'rgba(255,255,255,0.3)' }}>
          Focus: {focusZone.charAt(0).toUpperCase() + focusZone.slice(1)}
        </span>
        {footerHints.map(({ key, action }) => (
          <div key={key} className="training-panel__footer-hint">
            <span className="training-panel__footer-key">{key}</span>
            <span>{action}</span>
          </div>
        ))}
      </div>
    </div>
  );
}
```

**Step 3: Verify files created**

Run: `ls -la src/client/components/TrainingPanel.jsx src/client/styles/TrainingPanel.css`
Expected: Both files exist

**Step 4: Commit**

```bash
git add src/client/components/TrainingPanel.jsx src/client/styles/TrainingPanel.css
git commit -m "feat(training): create TrainingPanel component shell with PiP and footer"
```

---

### Task 4: Add TrainingPanel to App and Wire Toggle

**Files:**
- Modify: `src/client/components/App.jsx`

**Step 1: Import TrainingPanel**

Add import (around line 9):

```jsx
import TrainingPanel from './TrainingPanel';
```

**Step 2: Import useTrainingPanel hook**

Update the TrainingPanelContext import:

```jsx
import { TrainingPanelProvider, useTrainingPanel } from '../contexts/TrainingPanelContext';
```

**Step 3: Add TrainingPanel to AppContent render**

Find the return statement in `AppContent` (around line 200) and add TrainingPanel before the closing fragment:

```jsx
{/* Training Panel Overlay */}
<TrainingPanel />
```

**Step 4: Verify app builds and panel exists (hidden)**

Run: `npm run build`
Expected: Build succeeds

**Step 5: Commit**

```bash
git add src/client/components/App.jsx
git commit -m "feat(training): render TrainingPanel in App"
```

---

### Task 5: Add Train Button to TrustStrip

**Files:**
- Modify: `src/client/components/TrustStrip.jsx`
- Modify: `src/client/styles/TrustStrip.css`

**Step 1: Import useTrainingPanel in TrustStrip**

Add import at top of TrustStrip.jsx:

```jsx
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
```

**Step 2: Use the hook in TrustStrip component**

Inside the TrustStrip function, add:

```jsx
const { isOpen: trainingPanelOpen, togglePanel: toggleTrainingPanel } = useTrainingPanel();
```

**Step 3: Add Train button to TrustStrip**

Find the mode buttons section (around line 434-463) and add the Train button after the mode buttons div but before the E-STOP button:

```jsx
{/* Train Button */}
{genesisConnected && (
  <button
    className={`trust-strip__train-btn ${trainingPanelOpen ? 'trust-strip__train-btn--active' : ''}`}
    onClick={toggleTrainingPanel}
    title="Toggle Training Panel"
  >
    TRAIN
  </button>
)}
```

**Step 4: Add CSS for Train button**

Add to TrustStrip.css (at end of file):

```css
/* Train Button */
.trust-strip__train-btn {
  padding: 6px 16px;
  font-size: 12px;
  font-weight: 600;
  border: 1px solid rgba(139, 92, 246, 0.5);
  border-radius: 4px;
  background: rgba(139, 92, 246, 0.1);
  color: #a78bfa;
  cursor: pointer;
  transition: all 0.15s ease;
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.trust-strip__train-btn:hover {
  background: rgba(139, 92, 246, 0.2);
  border-color: rgba(139, 92, 246, 0.7);
}

.trust-strip__train-btn--active {
  background: rgba(139, 92, 246, 0.3);
  border-color: #a78bfa;
  box-shadow: 0 0 12px rgba(139, 92, 246, 0.3);
}
```

**Step 5: Verify button appears and toggles panel**

Run: `npm run build && npm start`
Expected: Train button appears in trust-strip when Genesis connected, clicking toggles panel visibility

**Step 6: Commit**

```bash
git add src/client/components/TrustStrip.jsx src/client/styles/TrustStrip.css
git commit -m "feat(training): add Train button to TrustStrip that toggles TrainingPanel"
```

---

### Task 6: Create HotRowKnob Component

**Files:**
- Create: `src/client/components/HotRowKnob.jsx`
- Create: `src/client/styles/HotRowKnob.css`

**Step 1: Create CSS file**

```css
/* HotRowKnob.css - Individual knob in Hot Row */

.hotrow-knob {
  display: flex;
  flex-direction: column;
  align-items: center;
  padding: 12px 16px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  min-width: 100px;
  cursor: pointer;
  transition: all 0.15s ease;
}

.hotrow-knob:hover {
  background: rgba(255, 255, 255, 0.05);
  border-color: rgba(255, 255, 255, 0.2);
}

.hotrow-knob--focused {
  border-color: rgba(139, 92, 246, 0.6);
  background: rgba(139, 92, 246, 0.1);
}

.hotrow-knob--editing {
  border-color: #a78bfa;
  background: rgba(139, 92, 246, 0.15);
  box-shadow: 0 0 12px rgba(139, 92, 246, 0.2);
}

.hotrow-knob--overridden::after {
  content: '';
  position: absolute;
  top: 6px;
  right: 6px;
  width: 6px;
  height: 6px;
  background: #f59e0b;
  border-radius: 50%;
}

.hotrow-knob {
  position: relative;
}

.hotrow-knob__label {
  font-size: 10px;
  text-transform: uppercase;
  letter-spacing: 0.5px;
  color: rgba(255, 255, 255, 0.5);
  margin-bottom: 6px;
}

.hotrow-knob__value {
  font-size: 16px;
  font-weight: 600;
  color: #fff;
  font-family: 'SF Mono', 'Consolas', monospace;
}

.hotrow-knob__subfield {
  font-size: 9px;
  color: rgba(255, 255, 255, 0.4);
  margin-top: 4px;
}
```

**Step 2: Create component file**

```jsx
import React from 'react';
import '../styles/HotRowKnob.css';

export default function HotRowKnob({
  label,
  value,
  subfield,
  focused = false,
  editing = false,
  overridden = false,
  onChange,
  onToggleSubfield,
  onClick,
}) {
  const formatValue = (val) => {
    if (typeof val === 'number') {
      return val.toFixed(2);
    }
    return val;
  };

  return (
    <div
      className={`hotrow-knob ${focused ? 'hotrow-knob--focused' : ''} ${editing ? 'hotrow-knob--editing' : ''} ${overridden ? 'hotrow-knob--overridden' : ''}`}
      onClick={onClick}
    >
      <span className="hotrow-knob__label">{label}</span>
      <span className="hotrow-knob__value">{formatValue(value)}</span>
      {subfield && (
        <span className="hotrow-knob__subfield">{subfield}</span>
      )}
    </div>
  );
}
```

**Step 3: Verify files created**

Run: `ls -la src/client/components/HotRowKnob.jsx src/client/styles/HotRowKnob.css`
Expected: Both files exist

**Step 4: Commit**

```bash
git add src/client/components/HotRowKnob.jsx src/client/styles/HotRowKnob.css
git commit -m "feat(training): create HotRowKnob component for parameter controls"
```

---

### Task 7: Create HotRow Component with 2 Initial Knobs

**Files:**
- Create: `src/client/components/HotRow.jsx`
- Create: `src/client/styles/HotRow.css`

**Step 1: Create CSS file**

```css
/* HotRow.css - 6 essential knobs strip */

.hotrow {
  display: flex;
  gap: 12px;
  padding: 16px;
  background: rgba(255, 255, 255, 0.02);
  border: 1px solid rgba(255, 255, 255, 0.08);
  border-radius: 8px;
}

.hotrow--focused {
  border-color: rgba(139, 92, 246, 0.3);
}
```

**Step 2: Create component file with 2 initial knobs**

```jsx
import React, { useState } from 'react';
import HotRowKnob from './HotRowKnob';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import '../styles/HotRow.css';

// Initial knob definitions (will expand to 6)
const KNOB_DEFINITIONS = [
  {
    id: 'decimation',
    label: 'Decimation',
    defaultValue: 4,
    min: 1,
    max: 20,
    step: 1,
    type: 'int',
  },
  {
    id: 'cmd_range_vx',
    label: 'Cmd Range',
    defaultValue: 1.0,
    min: 0.1,
    max: 3.0,
    step: 0.1,
    type: 'float',
    subfields: ['vx', 'vy', 'yaw'],
  },
];

export default function HotRow() {
  const { focusZone, editingParam, setEditingParam } = useTrainingPanel();
  const [focusedKnobIndex, setFocusedKnobIndex] = useState(0);
  const [values, setValues] = useState(() => {
    const initial = {};
    KNOB_DEFINITIONS.forEach(k => {
      initial[k.id] = k.defaultValue;
    });
    return initial;
  });
  const [subfieldIndex, setSubfieldIndex] = useState(0);

  const isFocused = focusZone === 'hotrow';

  const handleKnobClick = (index) => {
    setFocusedKnobIndex(index);
    const knob = KNOB_DEFINITIONS[index];
    if (editingParam === knob.id) {
      // Already editing, confirm
      setEditingParam(null);
    } else {
      // Start editing
      setEditingParam(knob.id);
    }
  };

  return (
    <div className={`hotrow ${isFocused ? 'hotrow--focused' : ''}`}>
      {KNOB_DEFINITIONS.map((knob, index) => (
        <HotRowKnob
          key={knob.id}
          label={knob.label}
          value={values[knob.id]}
          subfield={knob.subfields ? knob.subfields[subfieldIndex] : null}
          focused={isFocused && focusedKnobIndex === index}
          editing={editingParam === knob.id}
          overridden={values[knob.id] !== knob.defaultValue}
          onClick={() => handleKnobClick(index)}
        />
      ))}
    </div>
  );
}
```

**Step 3: Verify files created**

Run: `ls -la src/client/components/HotRow.jsx src/client/styles/HotRow.css`
Expected: Both files exist

**Step 4: Commit**

```bash
git add src/client/components/HotRow.jsx src/client/styles/HotRow.css
git commit -m "feat(training): create HotRow component with decimation and cmd range knobs"
```

---

### Task 8: Integrate HotRow into TrainingPanel

**Files:**
- Modify: `src/client/components/TrainingPanel.jsx`

**Step 1: Import HotRow**

Add import at top:

```jsx
import HotRow from './HotRow';
```

**Step 2: Replace placeholder with HotRow**

Replace the hotrow placeholder div:

```jsx
{/* Hot Row */}
<div className="training-panel__hotrow">
  <p style={{ color: 'rgba(255,255,255,0.4)', margin: 0 }}>
    Hot Row: 6 Essential Knobs (Coming Soon)
  </p>
</div>
```

With:

```jsx
{/* Hot Row */}
<HotRow />
```

**Step 3: Verify HotRow renders in panel**

Run: `npm run build && npm start`
Expected: Opening Training Panel shows HotRow with 2 knobs (Decimation, Cmd Range)

**Step 4: Commit**

```bash
git add src/client/components/TrainingPanel.jsx
git commit -m "feat(training): integrate HotRow into TrainingPanel"
```

---

### Task 9: Create SessionCard Component Shell

**Files:**
- Create: `src/client/components/SessionCard.jsx`
- Create: `src/client/styles/SessionCard.css`

**Step 1: Create CSS file**

```css
/* SessionCard.css - Individual session card in carousel */

.session-card {
  display: flex;
  flex-direction: column;
  width: 280px;
  height: 320px;
  background: rgba(255, 255, 255, 0.03);
  border: 1px solid rgba(255, 255, 255, 0.1);
  border-radius: 12px;
  overflow: hidden;
  cursor: pointer;
  transition: all 0.2s ease;
  flex-shrink: 0;
}

.session-card:hover {
  background: rgba(255, 255, 255, 0.05);
  border-color: rgba(255, 255, 255, 0.2);
  transform: translateY(-2px);
}

.session-card--focused {
  border-color: rgba(139, 92, 246, 0.6);
  background: rgba(139, 92, 246, 0.08);
  box-shadow: 0 0 20px rgba(139, 92, 246, 0.15);
}

.session-card--new {
  border-style: dashed;
  align-items: center;
  justify-content: center;
}

.session-card__preview {
  height: 160px;
  background: rgba(0, 0, 0, 0.3);
  display: flex;
  align-items: center;
  justify-content: center;
  color: rgba(255, 255, 255, 0.3);
  font-size: 12px;
}

.session-card__info {
  padding: 12px;
  flex: 1;
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.session-card__header {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
}

.session-card__name {
  font-size: 14px;
  font-weight: 600;
  color: #fff;
  margin: 0;
}

.session-card__robot {
  font-size: 11px;
  color: rgba(255, 255, 255, 0.5);
}

.session-card__badges {
  display: flex;
  gap: 6px;
  flex-wrap: wrap;
}

.session-card__badge {
  font-size: 9px;
  padding: 2px 6px;
  border-radius: 3px;
  text-transform: uppercase;
  letter-spacing: 0.3px;
}

.session-card__badge--type {
  background: rgba(59, 130, 246, 0.2);
  color: #60a5fa;
}

.session-card__badge--preset {
  background: rgba(16, 185, 129, 0.2);
  color: #34d399;
}

.session-card__badge--status {
  background: rgba(245, 158, 11, 0.2);
  color: #fbbf24;
}

.session-card__status-row {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-top: auto;
}

.session-card__time {
  font-size: 11px;
  color: rgba(255, 255, 255, 0.4);
}

.session-card__new-icon {
  font-size: 48px;
  color: rgba(255, 255, 255, 0.2);
  margin-bottom: 12px;
}

.session-card__new-text {
  font-size: 14px;
  color: rgba(255, 255, 255, 0.4);
}
```

**Step 2: Create component file**

```jsx
import React from 'react';
import '../styles/SessionCard.css';

export default function SessionCard({
  session = null,
  isNew = false,
  focused = false,
  onClick,
}) {
  if (isNew) {
    return (
      <div
        className={`session-card session-card--new ${focused ? 'session-card--focused' : ''}`}
        onClick={onClick}
      >
        <span className="session-card__new-icon">+</span>
        <span className="session-card__new-text">New Session</span>
      </div>
    );
  }

  if (!session) return null;

  const { name, robot, type, preset, status, lastRun } = session;

  return (
    <div
      className={`session-card ${focused ? 'session-card--focused' : ''}`}
      onClick={onClick}
    >
      {/* 3D URDF Preview Area */}
      <div className="session-card__preview">
        3D URDF Preview
      </div>

      {/* Session Info */}
      <div className="session-card__info">
        <div className="session-card__header">
          <div>
            <h3 className="session-card__name">{name}</h3>
            <span className="session-card__robot">{robot}</span>
          </div>
        </div>

        <div className="session-card__badges">
          <span className="session-card__badge session-card__badge--type">
            {type}
          </span>
          <span className="session-card__badge session-card__badge--preset">
            {preset}
          </span>
        </div>

        <div className="session-card__status-row">
          <span className="session-card__badge session-card__badge--status">
            {status}
          </span>
          <span className="session-card__time">{lastRun}</span>
        </div>
      </div>
    </div>
  );
}
```

**Step 3: Verify files created**

Run: `ls -la src/client/components/SessionCard.jsx src/client/styles/SessionCard.css`
Expected: Both files exist

**Step 4: Commit**

```bash
git add src/client/components/SessionCard.jsx src/client/styles/SessionCard.css
git commit -m "feat(training): create SessionCard component for carousel"
```

---

### Task 10: Create SessionCarousel Component

**Files:**
- Create: `src/client/components/SessionCarousel.jsx`
- Create: `src/client/styles/SessionCarousel.css`

**Step 1: Create CSS file**

```css
/* SessionCarousel.css - Horizontal carousel of session cards */

.session-carousel {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 24px;
  padding: 24px;
  overflow-x: auto;
  scroll-behavior: smooth;
  scrollbar-width: none;
}

.session-carousel::-webkit-scrollbar {
  display: none;
}

.session-carousel--focused {
  /* Visual indicator that carousel is focused */
}
```

**Step 2: Create component file with mock data**

```jsx
import React, { useState } from 'react';
import SessionCard from './SessionCard';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import '../styles/SessionCarousel.css';

// Mock sessions for MVP
const MOCK_SESSIONS = [
  {
    id: '1',
    name: 'Go2 Locomotion',
    robot: 'Go2',
    type: 'RL Training',
    preset: 'Locomotion',
    status: 'Running',
    lastRun: '2m ago',
  },
  {
    id: '2',
    name: 'Go2 Sit/Stand',
    robot: 'Go2',
    type: 'RL Training',
    preset: 'Sit/Stand',
    status: 'Paused',
    lastRun: '1h ago',
  },
];

export default function SessionCarousel() {
  const { focusZone, focusedSessionId, setFocusedSessionId } = useTrainingPanel();
  const [focusedIndex, setFocusedIndex] = useState(0);

  const isFocused = focusZone === 'carousel';
  const allItems = [...MOCK_SESSIONS, { id: 'new', isNew: true }];

  const handleCardClick = (index, session) => {
    setFocusedIndex(index);
    setFocusedSessionId(session.id);
  };

  return (
    <div className={`session-carousel ${isFocused ? 'session-carousel--focused' : ''}`}>
      {allItems.map((session, index) => (
        <SessionCard
          key={session.id}
          session={session.isNew ? null : session}
          isNew={session.isNew}
          focused={isFocused && focusedIndex === index}
          onClick={() => handleCardClick(index, session)}
        />
      ))}
    </div>
  );
}
```

**Step 3: Verify files created**

Run: `ls -la src/client/components/SessionCarousel.jsx src/client/styles/SessionCarousel.css`
Expected: Both files exist

**Step 4: Commit**

```bash
git add src/client/components/SessionCarousel.jsx src/client/styles/SessionCarousel.css
git commit -m "feat(training): create SessionCarousel component with mock sessions"
```

---

### Task 11: Integrate SessionCarousel into TrainingPanel

**Files:**
- Modify: `src/client/components/TrainingPanel.jsx`

**Step 1: Import SessionCarousel**

Add import at top:

```jsx
import SessionCarousel from './SessionCarousel';
```

**Step 2: Replace carousel placeholder**

Replace:

```jsx
{/* Carousel Area */}
<div className="training-panel__carousel">
  <p style={{ color: 'rgba(255,255,255,0.4)' }}>
    Session Carousel (Coming Soon)
  </p>
</div>
```

With:

```jsx
{/* Carousel Area */}
<div className="training-panel__carousel">
  <SessionCarousel />
</div>
```

**Step 3: Verify carousel renders**

Run: `npm run build && npm start`
Expected: Training Panel shows carousel with 2 mock sessions and "+ New Session" card

**Step 4: Commit**

```bash
git add src/client/components/TrainingPanel.jsx
git commit -m "feat(training): integrate SessionCarousel into TrainingPanel"
```

---

### Task 12: Add Keyboard Navigation for Training Panel

**Files:**
- Modify: `src/client/components/TrainingPanel.jsx`

**Step 1: Add useEffect for keyboard handling**

Add imports at top:

```jsx
import React, { useEffect, useCallback } from 'react';
```

**Step 2: Add keyboard event handler**

Inside TrainingPanel component, before the return statement:

```jsx
// Keyboard navigation
const handleKeyDown = useCallback((e) => {
  if (!isOpen) return;

  switch (e.key) {
    case 'ArrowUp':
      e.preventDefault();
      // Navigate focus up
      if (focusZone === 'carousel') {
        // Move to hotrow
        // Will be handled by context
      }
      break;
    case 'ArrowDown':
      e.preventDefault();
      // Navigate focus down
      break;
    case 'Escape':
      e.preventDefault();
      // Close panel handled by Train button
      break;
    default:
      break;
  }
}, [isOpen, focusZone]);

useEffect(() => {
  window.addEventListener('keydown', handleKeyDown);
  return () => window.removeEventListener('keydown', handleKeyDown);
}, [handleKeyDown]);
```

**Step 3: Verify keyboard events don't break anything**

Run: `npm run build && npm start`
Expected: Panel still works, arrow keys captured when panel open

**Step 4: Commit**

```bash
git add src/client/components/TrainingPanel.jsx
git commit -m "feat(training): add keyboard navigation scaffolding to TrainingPanel"
```

---

## Phase 1 Complete Checkpoint

At this point you have:
- TrainingPanelContext for state management
- TrainingPanel overlay (75% screen, PiP placeholder, footer hints)
- Train button in TrustStrip that toggles panel
- HotRow with 2 knobs (Decimation, Cmd Range)
- SessionCarousel with mock sessions and "+ New Session" card
- Basic keyboard event scaffolding

**Verification:**
1. `npm run build` succeeds
2. App loads without errors
3. Train button appears when Genesis connected
4. Clicking Train toggles the Training Panel
5. Panel shows carousel with mock sessions
6. Panel shows HotRow with 2 knobs
7. Footer shows context-aware hints

---

## Phase 2: Session State & Backend Integration

Connect to backend, add SessionContext, wire up real session data.

---

### Task 13: Create SessionContext

**Files:**
- Create: `src/client/contexts/SessionContext.jsx`

**Step 1: Create context with session state management**

```jsx
import React, { createContext, useContext, useState, useCallback, useEffect } from 'react';
import { useGenesis } from './GenesisContext';

const SessionContext = createContext(null);

// Default session structure
const createDefaultSession = (id, robot = 'Go2') => ({
  id,
  name: `${robot} Training`,
  robot,
  type: 'RL Training', // 'RL Training' | 'Env Capture'
  phase: 'training', // 'teleop' | 'training' | 'slam'
  preset: 'Locomotion', // 'Sit/Stand' | 'Locomotion' | 'Transfer'
  status: 'idle', // 'idle' | 'running' | 'paused' | 'done'
  environment: 'Flat Ground',
  overrides: {},
  createdAt: new Date().toISOString(),
  lastRun: null,
});

export function SessionProvider({ children }) {
  const { socket, genesisConnected } = useGenesis();
  const [sessions, setSessions] = useState([]);
  const [activeSessionId, setActiveSessionId] = useState(null);

  // Get active session
  const activeSession = sessions.find(s => s.id === activeSessionId) || null;

  // Create new session
  const createSession = useCallback((robot, type, preset, name) => {
    const id = `session_${Date.now()}`;
    const session = {
      ...createDefaultSession(id, robot),
      type,
      preset,
      name: name || `${robot} ${preset}`,
    };
    setSessions(prev => [...prev, session]);
    setActiveSessionId(id);

    // Notify backend
    if (socket && genesisConnected) {
      socket.emit('genesis_create_session', { session });
    }

    return session;
  }, [socket, genesisConnected]);

  // Clone session
  const cloneSession = useCallback((sessionId) => {
    const source = sessions.find(s => s.id === sessionId);
    if (!source) return null;

    const id = `session_${Date.now()}`;
    const now = new Date();
    const timestamp = `${(now.getMonth() + 1).toString().padStart(2, '0')}-${now.getDate().toString().padStart(2, '0')} ${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`;

    const cloned = {
      ...source,
      id,
      name: `${source.name} (clone) ${timestamp}`,
      status: 'idle',
      createdAt: now.toISOString(),
      lastRun: null,
    };

    setSessions(prev => [...prev, cloned]);
    setActiveSessionId(id);

    if (socket && genesisConnected) {
      socket.emit('genesis_clone_session', { sourceId: sessionId, session: cloned });
    }

    return cloned;
  }, [sessions, socket, genesisConnected]);

  // Delete session
  const deleteSession = useCallback((sessionId) => {
    setSessions(prev => prev.filter(s => s.id !== sessionId));
    if (activeSessionId === sessionId) {
      setActiveSessionId(null);
    }

    if (socket && genesisConnected) {
      socket.emit('genesis_delete_session', { sessionId });
    }
  }, [activeSessionId, socket, genesisConnected]);

  // Update session param
  const updateSessionParam = useCallback((sessionId, paramPath, value) => {
    setSessions(prev => prev.map(s => {
      if (s.id !== sessionId) return s;
      return {
        ...s,
        overrides: {
          ...s.overrides,
          [paramPath]: value,
        },
      };
    }));

    if (socket && genesisConnected) {
      socket.emit('genesis_set_param', { sessionId, paramPath, value });
    }
  }, [socket, genesisConnected]);

  const value = {
    sessions,
    activeSession,
    activeSessionId,
    setActiveSessionId,
    createSession,
    cloneSession,
    deleteSession,
    updateSessionParam,
  };

  return (
    <SessionContext.Provider value={value}>
      {children}
    </SessionContext.Provider>
  );
}

export function useSession() {
  const context = useContext(SessionContext);
  if (!context) {
    throw new Error('useSession must be used within SessionProvider');
  }
  return context;
}
```

**Step 2: Verify file created**

Run: `ls -la src/client/contexts/SessionContext.jsx`
Expected: File exists

**Step 3: Commit**

```bash
git add src/client/contexts/SessionContext.jsx
git commit -m "feat(training): create SessionContext for session state management"
```

---

### Task 14: Integrate SessionProvider into App

**Files:**
- Modify: `src/client/components/App.jsx`

**Step 1: Import SessionProvider**

Add import:

```jsx
import { SessionProvider } from '../contexts/SessionContext';
```

**Step 2: Add SessionProvider to component tree**

Update AppWithGenesis:

```jsx
export function AppWithGenesis() {
  return (
    <SettingsProvider>
      <GenesisProvider>
        <SessionProvider>
          <TrainingPanelProvider>
            <AppContent />
          </TrainingPanelProvider>
        </SessionProvider>
      </GenesisProvider>
    </SettingsProvider>
  );
}
```

**Step 3: Verify app builds**

Run: `npm run build`
Expected: Build succeeds

**Step 4: Commit**

```bash
git add src/client/components/App.jsx
git commit -m "feat(training): integrate SessionProvider into App"
```

---

### Task 15: Wire SessionCarousel to SessionContext

**Files:**
- Modify: `src/client/components/SessionCarousel.jsx`

**Step 1: Import useSession**

Add import:

```jsx
import { useSession } from '../contexts/SessionContext';
```

**Step 2: Replace mock data with real session state**

Replace the MOCK_SESSIONS constant and update the component:

```jsx
import React, { useState, useEffect } from 'react';
import SessionCard from './SessionCard';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import { useSession } from '../contexts/SessionContext';
import '../styles/SessionCarousel.css';

export default function SessionCarousel() {
  const { focusZone, focusedSessionId, setFocusedSessionId } = useTrainingPanel();
  const { sessions, createSession } = useSession();
  const [focusedIndex, setFocusedIndex] = useState(0);

  const isFocused = focusZone === 'carousel';

  // Convert sessions to card format
  const sessionCards = sessions.map(s => ({
    id: s.id,
    name: s.name,
    robot: s.robot,
    type: s.type,
    preset: s.preset,
    status: s.status,
    lastRun: s.lastRun ? formatTimeAgo(s.lastRun) : 'Never',
  }));

  const allItems = [...sessionCards, { id: 'new', isNew: true }];

  // Keep focusedIndex in bounds
  useEffect(() => {
    if (focusedIndex >= allItems.length) {
      setFocusedIndex(Math.max(0, allItems.length - 1));
    }
  }, [allItems.length, focusedIndex]);

  const handleCardClick = (index, item) => {
    setFocusedIndex(index);
    if (item.isNew) {
      // Create a new session with defaults
      const newSession = createSession('Go2', 'RL Training', 'Locomotion');
      setFocusedSessionId(newSession.id);
    } else {
      setFocusedSessionId(item.id);
    }
  };

  return (
    <div className={`session-carousel ${isFocused ? 'session-carousel--focused' : ''}`}>
      {allItems.map((item, index) => (
        <SessionCard
          key={item.id}
          session={item.isNew ? null : item}
          isNew={item.isNew}
          focused={isFocused && focusedIndex === index}
          onClick={() => handleCardClick(index, item)}
        />
      ))}
    </div>
  );
}

// Helper function
function formatTimeAgo(dateString) {
  const date = new Date(dateString);
  const now = new Date();
  const diffMs = now - date;
  const diffMins = Math.floor(diffMs / 60000);

  if (diffMins < 1) return 'Just now';
  if (diffMins < 60) return `${diffMins}m ago`;
  const diffHours = Math.floor(diffMins / 60);
  if (diffHours < 24) return `${diffHours}h ago`;
  const diffDays = Math.floor(diffHours / 24);
  return `${diffDays}d ago`;
}
```

**Step 3: Verify carousel uses real session state**

Run: `npm run build && npm start`
Expected: Carousel starts empty (no mock data), clicking "+ New Session" creates a real session

**Step 4: Commit**

```bash
git add src/client/components/SessionCarousel.jsx
git commit -m "feat(training): wire SessionCarousel to SessionContext"
```

---

## Phase 2 Continues...

This plan continues with:
- Task 16-20: Backend socket event handlers in bridge_server.py
- Task 21-25: Preset config files and loading
- Task 26-30: Complete HotRow with all 6 knobs
- Task 31-35: Advanced drawer with categories
- Task 36-40: Phase-specific controls (Teleop, Training, SLAM)
- Task 41-45: BlendEvalPanel (right edge)
- Task 46-50: Environment Library
- Task 51-55: Gamepad navigation

**The plan is structured for incremental delivery.** After Phase 1 (Tasks 1-12), you have a working MVP. Each subsequent phase adds functionality without breaking what exists.

---

## Verification Gates

After each phase, verify:

**Phase 1 (Tasks 1-12):**
- [ ] `npm run build` succeeds
- [ ] Train button visible in TrustStrip
- [ ] Panel toggles on click
- [ ] Carousel shows sessions
- [ ] HotRow shows 2 knobs
- [ ] Footer shows focus hints

**Phase 2 (Tasks 13-15):**
- [ ] Sessions persist in state
- [ ] Create session works
- [ ] Clone session works
- [ ] Sessions survive panel close/open

---

## Notes for Implementer

1. **CSS follows existing patterns** — Check `EdgePanel.css`, `TrustStrip.css` for reference
2. **Socket events follow naming** — All Genesis events prefixed with `genesis_`
3. **Context pattern matches GenesisContext** — Same structure, same hook pattern
4. **No external dependencies** — Uses existing Three.js, Socket.io, React
5. **Steam Deck gamepad** — Will be added in Phase 5 after core UI works
