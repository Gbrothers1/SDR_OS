# Genesis-Style Viewer Camera Controls Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace the passive `SimpleSimViewer` (JPEG-only, no controls) with the interactive `SimViewer` that has Genesis-style orbit/pan/zoom camera controls, and add matching keyboard shortcuts from the Genesis native viewer.

**Architecture:** The existing `SimViewer.jsx` already implements spherical-coordinate orbit/pan/zoom via mouse, sending `genesis_camera` events through Socket.io to the bridge. The plan is to: (1) swap `SimpleSimViewer` for `SimViewer` in App.jsx, (2) add Genesis-style keyboard shortcuts (reset view, toggle wireframe, etc.), (3) add touch support for Steam Deck, and (4) add a keyboard shortcut help overlay.

**Tech Stack:** React 18, Socket.io, CSS (glassmorphism design system already in place)

---

### Task 1: Swap SimpleSimViewer for SimViewer in App.jsx

**Files:**
- Modify: `src/client/components/App.jsx:9,418`

**Step 1: Update import**

In `App.jsx`, replace the `SimpleSimViewer` import with `SimViewer`:

```javascript
// Change line 9 from:
import SimpleSimViewer from './SimpleSimViewer';
// To:
import SimViewer from './SimViewer';
```

**Step 2: Update JSX rendering**

In `App.jsx`, replace the component usage around line 418:

```javascript
// Change from:
<SimpleSimViewer />
// To:
<SimViewer />
```

**Step 3: Verify it builds**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds with no errors

**Step 4: Commit**

```bash
git add src/client/components/App.jsx
git commit -m "feat: swap SimpleSimViewer for interactive SimViewer in Genesis mode"
```

---

### Task 2: Add keyboard shortcuts to SimViewer

Genesis native viewer supports keyboard shortcuts for camera reset (Z), animation toggle (A), and others. We'll add the most useful ones for a web viewer context.

**Files:**
- Modify: `src/client/components/SimViewer.jsx`

**Step 1: Add keyboard shortcut state and handler**

Add these state/ref declarations after the existing `cameraState` ref (around line 42):

```javascript
// Default camera state for reset
const defaultCameraState = useRef({
  radius: 3.0,
  theta: Math.PI / 4,
  phi: Math.PI / 4,
  target: { x: 0, y: 0, z: 0.5 },
  up: { x: 0, y: 0, z: 1 }
});

const [showShortcuts, setShowShortcuts] = useState(false);
const [autoRotate, setAutoRotate] = useState(false);
const autoRotateRef = useRef(false);
const animFrameRef = useRef(null);
```

**Step 2: Add resetCamera function**

Add after the `updateCamera` function:

```javascript
// Reset camera to default view (Genesis 'Z' key)
const resetCamera = useCallback(() => {
  const defaults = defaultCameraState.current;
  cameraState.current.radius = defaults.radius;
  cameraState.current.theta = defaults.theta;
  cameraState.current.phi = defaults.phi;
  cameraState.current.target = { ...defaults.target };
  updateCamera();
}, [updateCamera]);
```

**Step 3: Add auto-rotate logic**

Add after `resetCamera`:

```javascript
// Auto-rotate animation (Genesis 'A' key)
useEffect(() => {
  autoRotateRef.current = autoRotate;
  if (!autoRotate) {
    if (animFrameRef.current) {
      cancelAnimationFrame(animFrameRef.current);
      animFrameRef.current = null;
    }
    return;
  }

  const rotateStep = () => {
    if (!autoRotateRef.current) return;
    cameraState.current.theta += 0.005;
    updateCamera();
    animFrameRef.current = requestAnimationFrame(rotateStep);
  };
  animFrameRef.current = requestAnimationFrame(rotateStep);

  return () => {
    if (animFrameRef.current) {
      cancelAnimationFrame(animFrameRef.current);
    }
  };
}, [autoRotate, updateCamera]);
```

**Step 4: Add keyboard event handler**

Add after the auto-rotate effect:

```javascript
// Keyboard shortcuts (matching Genesis native viewer)
const handleKeyDown = useCallback((e) => {
  // Don't capture keys when typing in inputs
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA' || e.target.tagName === 'SELECT') return;

  switch (e.key.toLowerCase()) {
    case 'z':
      // Reset view (Genesis: Z)
      resetCamera();
      break;
    case 'a':
      // Toggle auto-rotate (Genesis: A)
      setAutoRotate(prev => !prev);
      break;
    case 'f':
      // Toggle fullscreen
      if (containerRef.current) {
        if (document.fullscreenElement) {
          document.exitFullscreen();
        } else {
          containerRef.current.requestFullscreen();
        }
      }
      break;
    case '?':
      // Toggle shortcut help
      setShowShortcuts(prev => !prev);
      break;
    case 'escape':
      setShowShortcuts(false);
      break;
    default:
      break;
  }
}, [resetCamera]);
```

**Step 5: Register keyboard listener**

Add inside the existing `useEffect` that handles mouse listeners (the one starting around line 152), alongside the existing event listeners:

```javascript
// Add keyboard listener
window.addEventListener('keydown', handleKeyDown);

// In cleanup:
window.removeEventListener('keydown', handleKeyDown);
```

Make sure to add `handleKeyDown` to the dependency array of that `useEffect`.

**Step 6: Add shortcut help overlay to JSX**

Add inside the return JSX, after the camera-hint div (before the closing `</div>`):

```jsx
{/* Keyboard shortcut overlay */}
{showShortcuts && (
  <div className="sim-shortcuts-overlay">
    <div className="sim-shortcuts-panel">
      <div className="sim-shortcuts-header">
        <span>Keyboard Shortcuts</span>
        <button className="sim-shortcuts-close" onClick={() => setShowShortcuts(false)}>x</button>
      </div>
      <div className="sim-shortcuts-list">
        <div className="sim-shortcut-item"><kbd>Z</kbd><span>Reset camera view</span></div>
        <div className="sim-shortcut-item"><kbd>A</kbd><span>Toggle auto-rotate</span></div>
        <div className="sim-shortcut-item"><kbd>F</kbd><span>Toggle fullscreen</span></div>
        <div className="sim-shortcut-item"><kbd>?</kbd><span>Show/hide shortcuts</span></div>
        <div className="sim-shortcut-item"><kbd>Esc</kbd><span>Close this panel</span></div>
      </div>
      <div className="sim-shortcuts-mouse">
        <div className="sim-shortcut-item"><span>Left drag</span><span>Orbit</span></div>
        <div className="sim-shortcut-item"><span>Shift+drag / Middle</span><span>Pan</span></div>
        <div className="sim-shortcut-item"><span>Ctrl+drag / Right</span><span>Zoom</span></div>
        <div className="sim-shortcut-item"><span>Scroll wheel</span><span>Zoom</span></div>
      </div>
    </div>
  </div>
)}
```

**Step 7: Update camera hint to mention keyboard shortcuts**

Replace the existing camera-hint JSX:

```jsx
{hasVideo && (
  <div className="sim-camera-hint">
    <span>Drag: orbit</span>
    <span>Shift+drag: pan</span>
    <span>Scroll: zoom</span>
    <span>Press ? for shortcuts</span>
  </div>
)}
```

**Step 8: Verify it builds**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 9: Commit**

```bash
git add src/client/components/SimViewer.jsx
git commit -m "feat: add Genesis-style keyboard shortcuts to SimViewer (reset, auto-rotate, fullscreen)"
```

---

### Task 3: Add CSS for keyboard shortcut overlay

**Files:**
- Modify: `src/client/styles/SimViewer.css`

**Step 1: Add shortcut overlay styles**

Append to `SimViewer.css`:

```css
/* Keyboard shortcuts overlay */
.sim-shortcuts-overlay {
  position: absolute;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  background-color: rgba(0, 0, 0, 0.5);
  z-index: 100;
  pointer-events: auto;
}

.sim-shortcuts-panel {
  background-color: var(--color-glass-bg);
  border: 1px solid var(--color-glass-border);
  border-radius: var(--radius-md);
  backdrop-filter: blur(20px);
  -webkit-backdrop-filter: blur(20px);
  padding: var(--space-5);
  min-width: 280px;
  max-width: 360px;
}

.sim-shortcuts-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: var(--space-4);
  font-size: var(--font-size-base);
  font-weight: bold;
  color: var(--color-text-primary);
}

.sim-shortcuts-close {
  background: none;
  border: 1px solid var(--color-glass-border);
  color: var(--color-text-muted);
  cursor: pointer;
  font-size: var(--font-size-sm);
  padding: 2px 8px;
  border-radius: var(--radius-sm);
}

.sim-shortcuts-close:hover {
  color: var(--color-text-primary);
  border-color: var(--color-text-muted);
}

.sim-shortcuts-list {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
  margin-bottom: var(--space-4);
  padding-bottom: var(--space-4);
  border-bottom: 1px solid var(--color-glass-border);
}

.sim-shortcuts-mouse {
  display: flex;
  flex-direction: column;
  gap: var(--space-2);
}

.sim-shortcut-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
  font-size: var(--font-size-sm);
  color: var(--color-text-secondary);
}

.sim-shortcut-item kbd {
  background-color: rgba(255, 255, 255, 0.08);
  border: 1px solid var(--color-glass-border);
  border-radius: 4px;
  padding: 2px 8px;
  font-family: var(--font-mono);
  font-size: var(--font-size-xs);
  color: var(--color-text-primary);
  min-width: 24px;
  text-align: center;
}
```

**Step 2: Verify it builds**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 3: Commit**

```bash
git add src/client/styles/SimViewer.css
git commit -m "feat: add glassmorphism CSS for keyboard shortcut overlay"
```

---

### Task 4: Add touch support for Steam Deck

The Steam Deck has a touchscreen. Genesis mouse controls map to touch gestures: single-finger drag = orbit, two-finger drag = pan, pinch = zoom.

**Files:**
- Modify: `src/client/components/SimViewer.jsx`

**Step 1: Add touch state refs**

Add after the existing mouse state refs (around line 31):

```javascript
const touchStartRef = useRef([]);
const initialPinchDistance = useRef(null);
const initialPinchRadius = useRef(null);
```

**Step 2: Add touch event handlers**

Add after the existing `handleContextMenu` callback:

```javascript
// Touch support for Steam Deck
const handleTouchStart = useCallback((e) => {
  const touches = Array.from(e.touches);
  touchStartRef.current = touches.map(t => ({ x: t.clientX, y: t.clientY }));

  if (touches.length === 1) {
    setIsDragging(true);
    setDragMode('orbit');
    lastMousePos.current = { x: touches[0].clientX, y: touches[0].clientY };
  } else if (touches.length === 2) {
    setIsDragging(true);
    // Calculate initial pinch distance for zoom
    const dx = touches[1].clientX - touches[0].clientX;
    const dy = touches[1].clientY - touches[0].clientY;
    initialPinchDistance.current = Math.hypot(dx, dy);
    initialPinchRadius.current = cameraState.current.radius;
    // Two-finger starts as pan
    setDragMode('pan');
    const midX = (touches[0].clientX + touches[1].clientX) / 2;
    const midY = (touches[0].clientY + touches[1].clientY) / 2;
    lastMousePos.current = { x: midX, y: midY };
  }
}, []);

const handleTouchMove = useCallback((e) => {
  e.preventDefault();
  const touches = Array.from(e.touches);

  if (touches.length === 2 && initialPinchDistance.current !== null) {
    // Pinch zoom
    const dx = touches[1].clientX - touches[0].clientX;
    const dy = touches[1].clientY - touches[0].clientY;
    const currentDist = Math.hypot(dx, dy);
    const scale = initialPinchDistance.current / currentDist;
    cameraState.current.radius = Math.max(0.5, Math.min(50,
      initialPinchRadius.current * scale
    ));
    updateCamera();

    // Also pan with midpoint movement
    const midX = (touches[0].clientX + touches[1].clientX) / 2;
    const midY = (touches[0].clientY + touches[1].clientY) / 2;
    const deltaX = midX - lastMousePos.current.x;
    const deltaY = midY - lastMousePos.current.y;
    lastMousePos.current = { x: midX, y: midY };

    const { theta, phi, radius } = cameraState.current;
    const panSpeed = radius * 0.005 * 0.5;
    const rightX = Math.sin(theta + Math.PI / 2);
    const rightY = Math.cos(theta + Math.PI / 2);
    const upX = Math.cos(phi) * Math.cos(theta);
    const upY = Math.cos(phi) * Math.sin(theta);
    const upZ = Math.sin(phi);
    cameraState.current.target.x -= (deltaX * rightX + deltaY * upX) * panSpeed;
    cameraState.current.target.y -= (deltaX * rightY + deltaY * upY) * panSpeed;
    cameraState.current.target.z += deltaY * upZ * panSpeed;
    updateCamera();
  } else if (touches.length === 1 && isDragging) {
    // Single finger orbit
    const deltaX = touches[0].clientX - lastMousePos.current.x;
    const deltaY = touches[0].clientY - lastMousePos.current.y;
    lastMousePos.current = { x: touches[0].clientX, y: touches[0].clientY };

    cameraState.current.theta -= deltaX * 0.005 * 2;
    cameraState.current.phi -= deltaY * 0.005 * 2;
    cameraState.current.phi = Math.max(0.1, Math.min(Math.PI - 0.1, cameraState.current.phi));
    updateCamera();
  }
}, [isDragging, updateCamera]);

const handleTouchEnd = useCallback((e) => {
  if (e.touches.length === 0) {
    setIsDragging(false);
    setDragMode(null);
    initialPinchDistance.current = null;
    initialPinchRadius.current = null;
  } else if (e.touches.length === 1) {
    // Went from 2 fingers to 1 - switch to orbit
    setDragMode('orbit');
    initialPinchDistance.current = null;
    initialPinchRadius.current = null;
    lastMousePos.current = { x: e.touches[0].clientX, y: e.touches[0].clientY };
  }
}, []);
```

**Step 3: Register touch listeners**

In the same `useEffect` that registers mouse listeners, add:

```javascript
container.addEventListener('touchstart', handleTouchStart, { passive: false });
container.addEventListener('touchmove', handleTouchMove, { passive: false });
container.addEventListener('touchend', handleTouchEnd);

// In cleanup:
container.removeEventListener('touchstart', handleTouchStart);
container.removeEventListener('touchmove', handleTouchMove);
container.removeEventListener('touchend', handleTouchEnd);
```

Add `handleTouchStart`, `handleTouchMove`, `handleTouchEnd` to the dependency array.

**Step 4: Verify it builds**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 5: Commit**

```bash
git add src/client/components/SimViewer.jsx
git commit -m "feat: add touch gesture support for Steam Deck (orbit, pinch-zoom, two-finger pan)"
```

---

### Task 5: Add object-fit toggle (cover vs contain)

The current `SimViewer` uses `object-fit: cover` which crops the frame. Genesis native viewer shows the full frame. Add a toggle.

**Files:**
- Modify: `src/client/components/SimViewer.jsx`
- Modify: `src/client/styles/SimViewer.css`

**Step 1: Add contain mode state**

Add a state variable:

```javascript
const [containMode, setContainMode] = useState(false);
```

**Step 2: Add 'C' keyboard shortcut**

In the `handleKeyDown` switch statement, add:

```javascript
case 'c':
  // Toggle contain/cover mode
  setContainMode(prev => !prev);
  break;
```

**Step 3: Apply class to container**

Update the container div:

```jsx
<div className={`sim-viewer${containMode ? ' contain-mode' : ''}`} ref={containerRef}>
```

**Step 4: Add to shortcut list and overlay**

Add to the shortcuts panel:

```jsx
<div className="sim-shortcut-item"><kbd>C</kbd><span>Toggle frame fit mode</span></div>
```

**Step 5: Verify it builds**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 6: Commit**

```bash
git add src/client/components/SimViewer.jsx src/client/styles/SimViewer.css
git commit -m "feat: add frame fit toggle (cover/contain) with 'C' keyboard shortcut"
```

---

### Task 6: Persist camera state in GenesisContext

When switching modes or panels, the camera resets. Store last-known camera state in GenesisContext so it persists.

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx`
- Modify: `src/client/components/SimViewer.jsx`

**Step 1: Add camera state to GenesisContext**

In `GenesisContext.jsx`, add after the existing state declarations:

```javascript
// Persistent camera state
const [cameraPosition, setCameraPosition] = useState(null);
```

Expose it in the value object:

```javascript
cameraPosition,
setCameraPosition,
```

**Step 2: Sync SimViewer camera state with context**

In `SimViewer.jsx`, import `setCameraPosition` and `cameraPosition` from `useGenesis()`.

When `updateCamera` fires, also update the context:

```javascript
const updateCamera = useCallback(() => {
  if (!socket) return;

  const pos = getCartesianPosition();
  const { target } = cameraState.current;

  socket.emit('genesis_camera', {
    position: [pos.x, pos.y, pos.z],
    lookat: [target.x, target.y, target.z]
  });

  // Persist to context
  if (setCameraPosition) {
    setCameraPosition({
      radius: cameraState.current.radius,
      theta: cameraState.current.theta,
      phi: cameraState.current.phi,
      target: { ...cameraState.current.target }
    });
  }
}, [socket, getCartesianPosition, setCameraPosition]);
```

On mount, restore from context if available:

```javascript
useEffect(() => {
  if (cameraPosition) {
    cameraState.current.radius = cameraPosition.radius;
    cameraState.current.theta = cameraPosition.theta;
    cameraState.current.phi = cameraPosition.phi;
    cameraState.current.target = { ...cameraPosition.target };
    updateCamera();
  }
}, []); // Only on mount
```

**Step 3: Verify it builds**

Run: `cd /home/ethan/dev/Genesis/SDR_OS && npm run build`
Expected: Build succeeds

**Step 4: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx src/client/components/SimViewer.jsx
git commit -m "feat: persist camera state in GenesisContext across panel switches"
```

---

## Summary of Changes

| Task | What | Files |
|------|------|-------|
| 1 | Swap SimpleSimViewer for SimViewer | App.jsx |
| 2 | Add keyboard shortcuts (Z/A/F/?) | SimViewer.jsx |
| 3 | CSS for shortcut overlay | SimViewer.css |
| 4 | Touch gestures for Steam Deck | SimViewer.jsx |
| 5 | Frame fit toggle (cover/contain) | SimViewer.jsx, SimViewer.css |
| 6 | Persist camera state in context | GenesisContext.jsx, SimViewer.jsx |

## Genesis Control Mapping Reference

| Genesis Native (Python/pyglet) | SDR_OS Web (SimViewer) |
|-------------------------------|----------------------|
| Left drag → Orbit (trackball rotate) | Left drag → Orbit (spherical) |
| Shift/Alt+Left drag → Pan | Shift+drag / Middle drag → Pan |
| Right drag → Zoom | Ctrl+drag / Right drag → Zoom |
| Scroll wheel → Zoom | Scroll wheel → Zoom |
| Z → Reset view | Z → Reset view |
| A → Toggle auto-rotate | A → Toggle auto-rotate |
| F11 → Fullscreen | F → Fullscreen |
| (no equivalent) | ? → Shortcut help |
| (no equivalent) | C → Toggle frame fit |
| (no equivalent) | Touch: 1-finger orbit, pinch zoom, 2-finger pan |
