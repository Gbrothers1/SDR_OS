# NATS Transport, Safety Stack & Service Connectivity — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Replace Socket.io sim relay with NATS backbone, add 3-layer safety stack (HOLD/ESTOP), add H.264+JPEG codec pipeline, fix defaults, align docs.

**Architecture:** Browser sends commands via 0x04 WS messages → transport-server publishes to NATS `command.genesis.*` → genesis-sim subscribes and executes. Sim publishes telemetry to NATS `telemetry.*` → transport-server relays to browser via 0x02 WS messages. Safety is enforced at 3 layers: frontend (cosmetic gate), transport (command gate + latch zero), sim (TTL decay + canonical state authority).

**Tech Stack:** Rust (transport-server, axum, async-nats), Python (genesis sim runner, nats-py, PyAV/NVENC), JavaScript/React (frontend, WebCodecs H.264), NATS messaging, Docker Compose.

**Design doc:** `docs/plans/2026-02-06-nats-safety-transport-design.md`

---

## Task 1: Protocol Constants & NATS Subject Schema

Lock down the wire protocol and subject names so all subsequent tasks build on a shared contract.

**Files:**
- Modify: `services/transport-server/src/transport/websocket.rs`
- Modify: `services/transport-server/src/shm/protocol.rs`
- Modify: `src/client/utils/H264Decoder.js`
- Create: `docs/nats-subjects.md`

**Step 1: Add 0x04 COMMAND and CODEC_JPEG constants to Rust transport**

In `services/transport-server/src/transport/websocket.rs`, after line 6, add:

```rust
pub const MSG_TYPE_SIGNALING: u8 = 0x03;
pub const MSG_TYPE_COMMAND: u8 = 0x04;
```

In `services/transport-server/src/shm/protocol.rs`, after line 21 (`CODEC_HEVC`), add:

```rust
pub const CODEC_JPEG: u16 = 3;
```

**Step 2: Add 0x04 COMMAND constant to frontend**

In `src/client/utils/H264Decoder.js`, update the MsgType object (line 161-165):

```javascript
export const MsgType = {
  VIDEO: 0x01,
  TELEMETRY: 0x02,
  SIGNALING: 0x03,
  COMMAND: 0x04,
};
```

**Step 3: Write NATS subject schema doc**

Create `docs/nats-subjects.md` with the full subject table, payload schemas, and direction arrows. Copy the tables from the design doc sections 3 and 4.

**Step 4: Commit**

```bash
git add services/transport-server/src/transport/websocket.rs \
      services/transport-server/src/shm/protocol.rs \
      src/client/utils/H264Decoder.js \
      docs/nats-subjects.md
git commit -m "feat: add 0x04 COMMAND protocol constant and NATS subject schema"
```

---

## Task 2: Transport-Server — Handle Inbound 0x04 Commands

Wire up the unused `_ws_receiver` in the transport-server to parse 0x04 messages and publish them to NATS.

**Files:**
- Modify: `services/transport-server/src/main.rs`
- Modify: `services/transport-server/src/nats/mod.rs`
- Create: `services/transport-server/src/nats/publisher.rs`

**Step 1: Create NATS publisher module**

Create `services/transport-server/src/nats/publisher.rs`:

```rust
use async_nats::Client;
use tracing::{info, warn};

/// Publishes a command to NATS with subject `command.genesis.{action}`.
pub async fn publish_command(
    nats: &Client,
    action: &str,
    payload: &[u8],
) -> Result<(), async_nats::PublishError> {
    let subject = format!("command.genesis.{}", action);
    nats.publish(subject.clone().into(), payload.into()).await?;
    info!(subject = %subject, bytes = payload.len(), "Published command to NATS");
    Ok(())
}
```

**Step 2: Register publisher module**

In `services/transport-server/src/nats/mod.rs`, add:

```rust
pub mod publisher;
```

**Step 3: Wire 0x04 handling into unified_send_loop**

In `services/transport-server/src/main.rs`:

1. Change `_ws_receiver` (line 137) to `ws_receiver`
2. Add `nats_client: Option<async_nats::Client>` parameter to `unified_send_loop()`
3. In the `tokio::select!` block (line 176), add a new branch:

```rust
msg = ws_receiver.next() => {
    match msg {
        Some(Ok(axum::extract::ws::Message::Binary(data))) => {
            if data.len() > 1 && data[0] == transport::websocket::MSG_TYPE_COMMAND {
                // Parse JSON after the type byte
                if let Ok(cmd) = serde_json::from_slice::<serde_json::Value>(&data[1..]) {
                    if let Some(action) = cmd.get("action").and_then(|a| a.as_str()) {
                        if let Some(ref nc) = nats_client {
                            let payload = serde_json::to_vec(&cmd).unwrap_or_default();
                            if let Err(e) = nats::publisher::publish_command(nc, action, &payload).await {
                                warn!("Failed to publish command: {}", e);
                            }
                        }
                    }
                }
            }
        }
        Some(Ok(axum::extract::ws::Message::Close(_))) | None => break,
        _ => {}
    }
}
```

4. Pass the NATS client through. In `main()`, after connecting to NATS for telemetry (around line 80), store the client. Pass it into `handle_ws_client()` → `unified_send_loop()`.

**Step 4: Build and verify compilation**

Run: `cargo build --manifest-path services/transport-server/Cargo.toml`
Expected: Compiles with no errors.

**Step 5: Commit**

```bash
git add services/transport-server/src/
git commit -m "feat(transport): handle inbound 0x04 COMMAND messages, publish to NATS"
```

---

## Task 3: Transport-Server — Video Gate + Latch Zero

Add Layer 2 safety: track SHM frame freshness, gate velocity commands when video is stale, publish one zero then latch.

**Files:**
- Modify: `services/transport-server/src/main.rs`
- Modify: `services/transport-server/src/config.rs`

**Step 1: Add gate config**

In `services/transport-server/src/config.rs`, add to the Config struct:

```rust
pub video_gate_hold_ms: u64,   // 1000ms — enter HOLD
pub video_gate_estop_ms: u64,  // 5000ms — escalate to ESTOP
```

Add to `from_env()`:

```rust
video_gate_hold_ms: std::env::var("SDR_VIDEO_GATE_HOLD_MS")
    .ok().and_then(|v| v.parse().ok()).unwrap_or(1000),
video_gate_estop_ms: std::env::var("SDR_VIDEO_GATE_ESTOP_MS")
    .ok().and_then(|v| v.parse().ok()).unwrap_or(5000),
```

**Step 2: Add gate state to AppState**

In `main.rs`, add shared gate state:

```rust
use std::sync::atomic::{AtomicBool, Ordering};
use tokio::sync::RwLock;

// Add to AppState or create new struct:
struct VideoGateState {
    gate_active: AtomicBool,
    estop_active: AtomicBool,
    last_frame_time: std::sync::Mutex<std::time::Instant>,
}
```

**Step 3: Implement gate logic in the 0x04 handler**

In the 0x04 command handler (from Task 2), before publishing to NATS:

```rust
// Check if this is a motion command
let is_motion = matches!(action, "set_cmd_vel" | "set_alpha" | "skill_command");
let is_always_allowed = matches!(action, "estop" | "estop_clear" | "pause" | "reset");

if is_motion && !is_always_allowed {
    let stale_ms = gate_state.last_frame_time.lock().unwrap()
        .elapsed().as_millis() as u64;

    if stale_ms > config.video_gate_estop_ms {
        // ESTOP escalation
        if !gate_state.estop_active.swap(true, Ordering::SeqCst) {
            // Publish estop command once
            let estop_payload = serde_json::json!({"reason": "video_timeout", "_safety": "estop"});
            nats::publisher::publish_command(nc, "estop", &serde_json::to_vec(&estop_payload).unwrap()).await.ok();
            // Publish telemetry
            let gate_msg = serde_json::json!({"gated": true, "mode": "ESTOP", "stale_ms": stale_ms});
            nc.publish("telemetry.safety.video_gate".into(), serde_json::to_vec(&gate_msg).unwrap().into()).await.ok();
        }
        continue; // Drop the command
    } else if stale_ms > config.video_gate_hold_ms {
        if !gate_state.gate_active.swap(true, Ordering::SeqCst) {
            // Latch: publish ONE zero velocity
            let zero_vel = serde_json::json!({"action": "set_cmd_vel", "data": {"linear_x": 0, "linear_y": 0, "angular_z": 0}, "_safety": "hold"});
            nats::publisher::publish_command(nc, "set_cmd_vel", &serde_json::to_vec(&zero_vel).unwrap()).await.ok();
            // Publish telemetry
            let gate_msg = serde_json::json!({"gated": true, "mode": "HOLD", "stale_ms": stale_ms});
            nc.publish("telemetry.safety.video_gate".into(), serde_json::to_vec(&gate_msg).unwrap().into()).await.ok();
        }
        continue; // Drop subsequent motion commands
    }
}
```

**Step 4: Update last_frame_time from SHM reader**

In `shm_read_loop()` (line 286-311), on successful frame read, update the shared `last_frame_time`:

```rust
ReadResult::Frame(frame) => {
    *gate_state.last_frame_time.lock().unwrap() = Instant::now();
    if gate_state.gate_active.swap(false, Ordering::SeqCst) {
        // Video recovered — clear gate
        if let Some(ref nc) = nats_client {
            let msg = serde_json::json!({"gated": false});
            nc.publish("telemetry.safety.video_gate".into(), serde_json::to_vec(&msg).unwrap().into()).await.ok();
        }
    }
    gate_state.estop_active.store(false, Ordering::SeqCst);
    // ... existing frame broadcast code
}
```

**Step 5: Build and verify**

Run: `cargo build --manifest-path services/transport-server/Cargo.toml`
Expected: Compiles.

**Step 6: Commit**

```bash
git add services/transport-server/src/
git commit -m "feat(transport): add Layer 2 video gate with latch-zero and ESTOP escalation"
```

---

## Task 4: Genesis Sim Runner — NATS Client + Command Subscriber

Replace Socket.io with NATS. Sim subscribes to `command.genesis.>` and publishes telemetry.

**Files:**
- Modify: `scripts/genesis_sim_runner.py`
- Modify: `pyproject.toml` (add nats-py dependency)

**Step 1: Add nats-py dependency**

In `pyproject.toml`, add to `[project.dependencies]`:

```toml
nats-py>=2.9.0
```

Install in container: `docker exec sdr_os-genesis-sim-1 pip install nats-py`

**Step 2: Replace Socket.io with NATS in GenesisSimRunner**

In `scripts/genesis_sim_runner.py`, replace the `connect_socketio()` method (lines 240-274) with:

```python
async def connect_nats(self):
    """Connect to NATS for command/telemetry."""
    import nats as nats_client

    nats_url = os.environ.get("SDR_NATS_URL", "nats://localhost:4222")
    self.nc = await nats_client.connect(nats_url)
    logger.info(f"Connected to NATS at {nats_url}")

    # Subscribe to all genesis commands
    self.cmd_sub = await self.nc.subscribe("command.genesis.>")
    asyncio.create_task(self._handle_commands())

async def _handle_commands(self):
    """Process incoming NATS commands."""
    async for msg in self.cmd_sub.messages:
        try:
            action = msg.subject.split(".")[-1]  # command.genesis.<action>
            data = json.loads(msg.data.decode()) if msg.data else {}
            cmd_data = data.get("data", data)
            cmd_seq = data.get("cmd_seq", 0)

            # Out-of-order protection for velocity commands
            if action == "set_cmd_vel":
                if cmd_seq <= self._last_cmd_seq:
                    continue
                self._last_cmd_seq = cmd_seq
                self._last_cmd_vel_time = time.monotonic()
                if self.env:
                    self.env.set_velocity_from_gamepad(cmd_data)
                continue

            if action == "pause":
                self.paused = cmd_data.get("paused", True)
            elif action == "reset":
                if self.env:
                    obs, _ = self.env.reset()
                    self.current_obs = obs
            elif action == "estop":
                self._safety_mode = "ESTOP"
                self._velocity = [0, 0, 0]
            elif action == "estop_clear":
                if not self._video_gate_active:
                    self._safety_mode = "ARMED"
            elif action == "load_policy":
                checkpoint_dir = cmd_data.get("checkpoint_dir", "")
                model_file = cmd_data.get("model_file")
                if checkpoint_dir and os.path.exists(checkpoint_dir):
                    obs_dim = self.current_obs.shape[-1] if self.current_obs is not None else 310
                    self.policy = load_policy(checkpoint_dir, model_file=model_file, obs_dim=obs_dim)

            # Publish ack
            ack = {"action": action, "cmd_seq": cmd_seq, "status": "ok"}
            await self.nc.publish("telemetry.command.ack", json.dumps(ack).encode())

        except Exception as e:
            logger.error(f"Command handler error: {e}")
```

**Step 3: Add safety state tracking**

Add to `__init__()`:

```python
self._safety_mode = "ARMED"  # ARMED | HOLD | ESTOP
self._safety_state_id = 0
self._last_cmd_seq = 0
self._last_cmd_vel_time = time.monotonic()
self._video_gate_active = False
self.paused = False
```

Subscribe to video gate in `connect_nats()`:

```python
self.gate_sub = await self.nc.subscribe("telemetry.safety.video_gate")
asyncio.create_task(self._handle_video_gate())

async def _handle_video_gate(self):
    async for msg in self.gate_sub.messages:
        try:
            data = json.loads(msg.data.decode())
            self._video_gate_active = data.get("gated", False)
            if self._video_gate_active and data.get("mode") == "ESTOP":
                self._safety_mode = "ESTOP"
        except Exception:
            pass
```

**Step 4: Replace metrics broadcast**

In the `run()` method, replace the Socket.io metrics emit block (lines 300-318) with NATS publishes:

```python
if now - last_metrics_time > 1.0 and self.nc and self.nc.is_connected:
    metrics = {
        "step": step_count,
        "fps": self.target_fps,
        "policy_loaded": self.policy is not None,
        "paused": self.paused,
    }
    if self.env:
        metrics["velocity_command"] = self.env.get_velocity_command()
        metrics["obs_breakdown"] = self.env.get_obs_breakdown()
        metrics["reward_breakdown"] = self.env.get_reward_breakdown()

    await self.nc.publish("telemetry.training.metrics", json.dumps(metrics).encode())

    if metrics.get("reward_breakdown"):
        await self.nc.publish("telemetry.reward.breakdown", json.dumps(metrics["reward_breakdown"]).encode())
    if metrics.get("obs_breakdown"):
        await self.nc.publish("telemetry.obs.breakdown", json.dumps(metrics["obs_breakdown"]).encode())

    last_metrics_time = now
```

**Step 5: Add TTL decay + safety state publisher**

Add a method called from the main loop:

```python
def _enforce_cmd_ttl(self):
    """Layer 3: TTL decay and ESTOP on command timeout."""
    if self._safety_mode == "ESTOP":
        return  # Already stopped

    elapsed = time.monotonic() - self._last_cmd_vel_time
    if elapsed > 2.0:
        self._safety_mode = "ESTOP"
        self._velocity = [0, 0, 0]
        logger.warning("Command timeout >2s — ESTOP")
    elif elapsed > 0.2:
        if self._safety_mode != "HOLD":
            self._safety_mode = "HOLD"
            logger.warning("Command TTL expired — HOLD, decaying velocity")
        # Exponential decay
        self._velocity = [v * 0.5 for v in self._velocity]
        if all(abs(v) < 0.01 for v in self._velocity):
            self._velocity = [0, 0, 0]
```

Publish canonical safety state at 2 Hz:

```python
if now - last_safety_time > 0.5 and self.nc and self.nc.is_connected:
    self._safety_state_id += 1
    state = {
        "state_id": self._safety_state_id,
        "mode": self._safety_mode,
        "reason": "ok" if self._safety_mode == "ARMED" else "cmd_timeout",
        "since_ms": int((time.monotonic() - self._last_cmd_vel_time) * 1000),
    }
    await self.nc.publish("telemetry.safety.state", json.dumps(state).encode())
    last_safety_time = now
```

**Step 6: Remove Socket.io imports and usage**

- Remove `import socketio` (was inside `connect_socketio`)
- Remove `connect_socketio()` method entirely
- Replace `sio = await self.connect_socketio()` with `await self.connect_nats()`
- Remove `sio.connected` checks
- Remove `sio.disconnect()` in the finally block
- Remove `sio.emit()` calls
- Add `import json` at top

**Step 7: Commit**

```bash
git add scripts/genesis_sim_runner.py pyproject.toml
git commit -m "feat(sim): replace Socket.io with NATS for commands and telemetry"
```

---

## Task 5: Frontend — Send Commands via 0x04 WS + Consume Safety State

Wire GenesisContext to send 0x04 commands over WS instead of Socket.io, and consume `telemetry.safety.state`.

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx`
- Modify: `src/client/utils/H264Decoder.js`

**Step 1: Add sendCommand helper for 0x04**

In `GenesisContext.jsx`, add a ref for the command sequence counter and a send function:

```javascript
const cmdSeqRef = useRef(0);

const sendWsCommand = useCallback((action, data = {}, ttlMs = null) => {
  const ws = bridgeWsRef.current;
  if (!ws || ws.readyState !== WebSocket.OPEN) return;

  cmdSeqRef.current += 1;
  const cmd = { action, cmd_seq: cmdSeqRef.current, data };
  if (ttlMs !== null) cmd.ttl_ms = ttlMs;

  const json = JSON.stringify(cmd);
  const jsonBytes = new TextEncoder().encode(json);
  const msg = new Uint8Array(1 + jsonBytes.length);
  msg[0] = MsgType.COMMAND;
  msg.set(jsonBytes, 1);
  ws.send(msg.buffer);
}, []);
```

**Step 2: Replace Socket.io action emitters with sendWsCommand**

Replace all `socket.emit('genesis_*')` calls in the actions section (lines 534-635):

```javascript
const loadRobot = useCallback((robotName) => {
  sendWsCommand('load_robot', { robot_name: robotName });
}, [sendWsCommand]);

const resetEnv = useCallback(() => {
  sendWsCommand('reset');
}, [sendWsCommand]);

const pauseSim = useCallback((paused) => {
  sendWsCommand('pause', { paused });
}, [sendWsCommand]);

const setMode = useCallback((mode) => {
  sendWsCommand('set_mode', { mode });
}, [sendWsCommand]);

const loadPolicy = useCallback((checkpointDir, modelFile) => {
  sendWsCommand('load_policy', { checkpoint_dir: checkpointDir, model_file: modelFile });
}, [sendWsCommand]);

const estop = useCallback(() => {
  sendWsCommand('estop', { reason: 'operator' });
}, [sendWsCommand]);

const estopClear = useCallback(() => {
  sendWsCommand('estop_clear');
}, [sendWsCommand]);

// Velocity commands get TTL
const sendVelocityCommand = useCallback((linearX, linearY, angularZ) => {
  sendWsCommand('set_cmd_vel', { linear_x: linearX, linear_y: linearY, angular_z: angularZ }, 200);
}, [sendWsCommand]);
```

**Step 3: Add safety state + video health tracking**

Add state variables:

```javascript
const [safetyState, setSafetyState] = useState({ state_id: 0, mode: 'ARMED', reason: 'ok', since_ms: 0 });
const [videoHealthy, setVideoHealthy] = useState(true);
const lastFrameTimeRef = useRef(Date.now());
const lastSafetyStateIdRef = useRef(0);
```

In `handleVideoFrame`, update lastFrameTime:

```javascript
lastFrameTimeRef.current = Date.now();
if (!videoHealthy) setVideoHealthy(true);
```

Add a 200ms interval to check video health (Layer 1):

```javascript
const videoHealthInterval = setInterval(() => {
  const age = Date.now() - lastFrameTimeRef.current;
  if (age > 500 && videoHealthy) {
    setVideoHealthy(false);
  } else if (age <= 500 && !videoHealthy) {
    setVideoHealthy(true);
  }
}, 200);
```

In `handleTelemetry`, add safety state routing:

```javascript
if (subject.includes('safety.state')) {
  const state = data;
  // Ignore out-of-order
  if (state.state_id > lastSafetyStateIdRef.current) {
    lastSafetyStateIdRef.current = state.state_id;
    setSafetyState(state);
  }
}
```

**Step 4: Remove Socket.io Genesis event listeners**

Remove the entire block at lines 411-514 that registers `socket.on('genesis_*')` listeners. Keep only controller-related Socket.io usage.

**Step 5: Expose new state in context value**

Add to the context value object:

```javascript
safetyState,
videoHealthy,
sendWsCommand,
sendVelocityCommand,
estopClear,
```

**Step 6: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx src/client/utils/H264Decoder.js
git commit -m "feat(frontend): send commands via 0x04 WS, consume safety state from NATS"
```

---

## Task 6: Frontend — Safety UI in TrustStrip + SimViewer

Show HOLD/ESTOP badges and VIDEO LOST overlay.

**Files:**
- Modify: `src/client/components/TrustStrip.jsx`
- Modify: `src/client/components/SimViewer.jsx`
- Modify: `src/client/styles/SimViewer.css`

**Step 1: Add safety badge to TrustStrip**

In `TrustStrip.jsx`, destructure from GenesisContext:

```javascript
const { safetyState, videoHealthy, estopClear } = useGenesis();
```

Add a safety badge next to the E-STOP button (around line 868):

```javascript
{safetyState.mode !== 'ARMED' && (
  <span className={`trust-strip__safety-badge trust-strip__safety-badge--${safetyState.mode.toLowerCase()}`}>
    {safetyState.mode}
  </span>
)}
```

Update E-STOP button to toggle (estop/estopClear):

```javascript
<button
  className={`trust-strip__estop ${safetyState.mode === 'ESTOP' ? 'trust-strip__estop--active' : ''}`}
  onClick={() => safetyState.mode === 'ESTOP' ? estopClear() : handleEstop()}
  title={safetyState.mode === 'ESTOP' ? 'Clear E-STOP (re-arm)' : 'Emergency stop'}
>
  {safetyState.mode === 'ESTOP' ? 'RE-ARM' : 'E-STOP'}
</button>
```

**Step 2: Add VIDEO LOST overlay to SimViewer**

In `SimViewer.jsx`, destructure `videoHealthy` and `safetyState`:

```javascript
const { videoHealthy, safetyState } = useGenesis();
```

Add overlay after the video/canvas/img elements (around line 420):

```javascript
{!videoHealthy && (
  <div className="sim-viewer__video-lost">
    <span className="sim-viewer__video-lost-text">VIDEO LOST — {safetyState.mode}</span>
  </div>
)}
```

**Step 3: Add CSS for safety badges and VIDEO LOST overlay**

In `src/client/styles/SimViewer.css`, add:

```css
.sim-viewer__video-lost {
  position: absolute;
  inset: 0;
  display: flex;
  align-items: center;
  justify-content: center;
  background: rgba(200, 0, 0, 0.3);
  z-index: 10;
  pointer-events: none;
}

.sim-viewer__video-lost-text {
  font-size: 2rem;
  font-weight: 700;
  color: #ff4444;
  text-shadow: 0 0 20px rgba(255, 0, 0, 0.5);
  animation: pulse-red 1s ease-in-out infinite;
}

@keyframes pulse-red {
  0%, 100% { opacity: 1; }
  50% { opacity: 0.5; }
}
```

**Step 4: Commit**

```bash
git add src/client/components/TrustStrip.jsx src/client/components/SimViewer.jsx src/client/styles/SimViewer.css
git commit -m "feat(ui): add HOLD/ESTOP safety badges and VIDEO LOST overlay"
```

---

## Task 7: Delete Socket.io Genesis Relay from server.js

Remove the 48 GENESIS_EVENTS, genesis_identify handler, and genesisBridgeConnected tracking.

**Files:**
- Modify: `server.js` (both worktree and main)

**Step 1: Remove GENESIS_EVENTS array and forEach broadcast**

Delete lines 73-148 (the `GENESIS_EVENTS` array) and lines 233-238 (the `forEach` broadcast loop).

**Step 2: Remove genesis_identify handler**

Delete lines 224-229 (the `genesis_identify` handler).

**Step 3: Remove genesisBridgeConnected tracking**

- Delete line 71: `let genesisBridgeConnected = false;`
- Remove from `/api/status` response (line 66): delete `genesisBridge: genesisBridgeConnected`
- Delete disconnect handler lines 246-250 (genesis bridge disconnect logic)

**Step 4: Sync worktree**

Copy `server.js` from worktree to main repo (or vice versa, whichever was edited).

**Step 5: Verify server starts**

Run: `node server.js` — confirm it starts without errors and `/api/status` returns `{"ok": true}`.

**Step 6: Commit**

```bash
git add server.js
git commit -m "refactor(server): remove Socket.io Genesis relay, keep only controller events"
```

---

## Task 8: JPEG Codec Path in Frontend

Handle JPEG frames (codec=3) via Blob → objectURL → `<img>` tag.

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx`
- Modify: `src/client/components/SimViewer.jsx`

**Step 1: Route JPEG frames in handleVideoFrame**

In `GenesisContext.jsx` `handleVideoFrame()`, after the stats update (around line 138), replace the H264 decode block:

```javascript
// Codec-aware frame routing
if (codec === CodecType.H264) {
  if (h264DecoderRef.current && streamBackend !== 'webrtc') {
    h264DecoderRef.current.decode(payload, isKeyframe);
  }
  if (streamCodec !== 'h264') setStreamCodec('h264');
} else if (codec === CodecType.JPEG) {
  // Create object URL from JPEG payload
  if (prevFrameUrlRef.current) {
    URL.revokeObjectURL(prevFrameUrlRef.current);
  }
  const blob = new Blob([payload], { type: 'image/jpeg' });
  const url = URL.createObjectURL(blob);
  prevFrameUrlRef.current = url;
  setCurrentFrame(url);
  if (streamCodec !== 'jpeg') setStreamCodec('jpeg');
}
```

Add ref for previous URL:

```javascript
const prevFrameUrlRef = useRef(null);
```

Import CodecType:

```javascript
import { detectH264Support, H264Decoder, MsgType, CodecType, parseVideoHeader } from '../utils/H264Decoder';
```

**Step 2: Update SimViewer render logic**

In `SimViewer.jsx`, update the conditional rendering to use `streamCodec`:

```javascript
{streamCodec === 'h264' && (
  <canvas ref={h264CanvasRef} className="sim-frame-image sim-viewer__canvas"
    style={{ objectFit: containMode ? 'contain' : 'cover' }} />
)}
{streamCodec === 'jpeg' && currentFrame && (
  <img src={currentFrame} alt="Simulation" className="sim-frame-image" draggable={false}
    style={{ objectFit: containMode ? 'contain' : 'cover' }} />
)}
{streamBackend === 'webrtc' && webrtcConnected && (
  <video ref={videoRef} className="sim-frame-image" autoPlay playsInline muted
    style={{ width: '100%', height: '100%', objectFit: 'contain' }} />
)}
```

**Step 3: Cleanup on unmount**

In the useEffect cleanup, revoke the last frame URL:

```javascript
return () => {
  if (prevFrameUrlRef.current) URL.revokeObjectURL(prevFrameUrlRef.current);
};
```

**Step 4: Build and verify**

Run: `npm run build`
Expected: Compiles without errors.

**Step 5: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx src/client/components/SimViewer.jsx
git commit -m "feat(frontend): add JPEG codec path with Blob → objectURL rendering"
```

---

## Task 9: H.264 NVENC Encoder in Sim Runner

Add hardware H.264 encoding with JPEG fallback.

**Files:**
- Modify: `scripts/genesis_sim_runner.py`
- Modify: `pyproject.toml`

**Step 1: Add PyAV dependency**

In `pyproject.toml`, add to `[project.optional-dependencies]` under `cuda`:

```toml
cuda = ["av>=14.0.0"]
```

Install in container: `docker exec sdr_os-genesis-sim-1 pip install av`

**Step 2: Create encoder abstraction in sim runner**

Add encoder classes before `GenesisSimRunner`:

```python
class JpegEncoder:
    """JPEG fallback encoder."""
    def __init__(self, quality=80):
        self.quality = quality
        self.codec = Codec.JPEG

    def encode(self, frame_rgb, frame_id):
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return None, False
        return buf.tobytes(), True  # (payload, is_keyframe=True for JPEG)

    def close(self):
        pass


class NvencEncoder:
    """H.264 NVENC hardware encoder producing Annex-B NAL units."""
    def __init__(self, width, height, fps=30, bitrate=3_000_000, gop=30):
        import av
        self.codec = Codec.H264
        self.gop = gop
        self.frame_count = 0

        self.container = av.open(
            "pipe:",
            mode="w",
            format="h264",
            options={
                "preset": "p4",
                "tune": "ull",
                "profile": "baseline",
                "b": str(bitrate),
                "g": str(gop),
                "bf": "0",
            },
        )
        self.stream = self.container.add_stream("h264_nvenc", rate=fps)
        self.stream.width = width
        self.stream.height = height
        self.stream.pix_fmt = "yuv420p"

    def encode(self, frame_rgb, frame_id):
        import av
        frame = av.VideoFrame.from_ndarray(frame_rgb, format="rgb24")
        frame.pts = frame_id
        packets = self.stream.encode(frame)
        if not packets:
            return None, False

        payload = b"".join(bytes(p) for p in packets)
        is_keyframe = self.frame_count % self.gop == 0
        self.frame_count += 1
        return payload, is_keyframe

    def close(self):
        try:
            self.container.close()
        except Exception:
            pass
```

**Step 3: Initialize encoder in GenesisSimRunner**

Add to `init_genesis()`, after env.build():

```python
width, height = self.camera_res
try:
    self.encoder = NvencEncoder(width, height, fps=self.target_fps)
    logger.info(f"NVENC H.264 encoder initialized ({width}x{height})")
except Exception as e:
    logger.warning(f"NVENC unavailable ({e}), falling back to JPEG")
    self.encoder = JpegEncoder(quality=self.jpeg_quality)
```

**Step 4: Update render_and_write to use encoder**

Replace the current `render_and_write()` method:

```python
def render_and_write(self):
    render_result = self.env.camera.render()
    img = render_result[0] if isinstance(render_result, tuple) else render_result

    if isinstance(img, torch.Tensor):
        frame_np = img.cpu().numpy()
    else:
        frame_np = np.asarray(img)
    if frame_np.ndim == 4:
        frame_np = frame_np[0]

    payload, is_keyframe = self.encoder.encode(frame_np, self.frame_id)
    if payload is None:
        return

    flags = FrameFlags.KEYFRAME if is_keyframe else FrameFlags.NONE
    self.shm_writer.write(
        payload=payload,
        frame_id=self.frame_id,
        flags=flags,
        codec=self.encoder.codec,
    )
    self.frame_id += 1
```

**Step 5: Commit**

```bash
git add scripts/genesis_sim_runner.py pyproject.toml
git commit -m "feat(sim): add NVENC H.264 encoder with JPEG fallback"
```

---

## Task 10: Fix Defaults, Config & Webpack

Clean up all stale settings, hardcoded IPs, and mismatched defaults.

**Files:**
- Modify: `src/client/contexts/SettingsContext.jsx`
- Modify: `src/client/components/Settings.jsx`
- Modify: `webpack.config.js`

**Step 1: Fix SettingsContext defaults**

In `src/client/contexts/SettingsContext.jsx` line 76, change:

```javascript
enabled: false,
```
to:
```javascript
enabled: true,
```

**Step 2: Fix Settings.jsx defaults**

In `src/client/components/Settings.jsx`, find the genesis defaults (around line 76):

Change `bridgeWsUrl: \`ws://${window.location.hostname}:9091\`` to `bridgeWsUrl: ''`

Remove `webrtcSignalingUrl: \`http://${window.location.hostname}:9092\`` entirely.

**Step 3: Fix webpack proxy**

In `webpack.config.js`, replace the proxy block (lines 53-56):

```javascript
proxy: [
  {
    context: ['/ros'],
    target: 'ws://localhost:9090',
    ws: true,
  },
],
```

**Step 4: Build and verify**

Run: `npm run build`
Expected: Compiles.

**Step 5: Commit**

```bash
git add src/client/contexts/SettingsContext.jsx src/client/components/Settings.jsx webpack.config.js
git commit -m "fix: align genesis defaults, remove stale 9091/9092 ports and hardcoded IPs"
```

---

## Task 11: Update Caddyfile for Production

Add all proxy routes and WebSocket timeout config.

**Files:**
- Modify: `configs/Caddyfile`

**Step 1: Update Caddyfile**

Replace the full Caddyfile with production-ready config:

```caddyfile
:80 {
    handle /health {
        respond "OK" 200
    }

    handle /stream/ws {
        reverse_proxy transport-server:8080 {
            flush_interval -1
            transport http {
                read_timeout 0
            }
        }
    }

    @socketio path /socket.io/*
    handle @socketio {
        reverse_proxy node:3000
    }

    handle /ros/* {
        reverse_proxy ros-bridge:9090
    }

    handle /api/* {
        reverse_proxy node:3000
    }

    handle {
        root * /srv/www
        try_files {path} /index.html
        file_server
    }
}
```

**Step 2: Commit**

```bash
git add configs/Caddyfile
git commit -m "feat(caddy): add production proxy routes with WS timeout config"
```

---

## Task 12: Update CLAUDE.md and Docs

Align documentation with the new architecture.

**Files:**
- Modify: `CLAUDE.md`
- Already created: `docs/nats-subjects.md` (Task 1)
- Already created: `docs/plans/2026-02-06-nats-safety-transport-design.md`

**Step 1: Update CLAUDE.md architecture section**

Add/update these sections in CLAUDE.md:

- **Binary WS Protocol** — 0x01 VIDEO, 0x02 TELEMETRY, 0x03 SIGNALING, 0x04 COMMAND
- **NATS Subjects** — link to `docs/nats-subjects.md`
- **Safety Stack** — ARMED → HOLD → ESTOP with thresholds
- **Transport Server** — expand existing section with gate logic
- **Genesis Sim Runner** — new section: NVENC encoder, NATS command subscriber, TTL decay
- **Docker Networking** — dev mode (host + port mapping) vs production (Caddy + backplane)
- **Socket.io** — clarify it's browser-only for gamepad sharing

**Step 2: Commit**

```bash
git add CLAUDE.md docs/
git commit -m "docs: update CLAUDE.md with NATS transport, safety stack, and binary WS protocol"
```

---

## Task 13: Smoke Tests

Add integration tests that validate the critical paths.

**Files:**
- Create: `tests/integration/test_ws_frame_delivery.py`
- Create: `tests/integration/test_nats_command_roundtrip.py`
- Create: `tests/integration/test_safety_video_gate.py`
- Create: `tests/integration/test_cmd_ttl_decay.py`
- Create: `tests/integration/test_safety_state_authority.py`

**Step 1: Write test_ws_frame_delivery**

```python
"""Smoke test: WS upgrade to /stream/ws, confirm 0x01 frame within 2s."""
import asyncio
import pytest
from websockets.asyncio.client import connect

@pytest.mark.asyncio
async def test_ws_frame_delivery():
    ws = await connect("ws://localhost:8080/stream/ws")
    try:
        msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
        assert len(msg) > 33, "Frame too short (need type + 32-byte header + payload)"
        assert msg[0] == 0x01, f"Expected VIDEO (0x01), got 0x{msg[0]:02x}"
    finally:
        await ws.close()
```

**Step 2: Write test_nats_command_roundtrip**

```python
"""Smoke test: publish command.genesis.pause, confirm telemetry.command.ack."""
import asyncio
import json
import pytest
import nats as nats_client

@pytest.mark.asyncio
async def test_nats_command_roundtrip():
    nc = await nats_client.connect("nats://localhost:4222")
    try:
        sub = await nc.subscribe("telemetry.command.ack")
        cmd = {"action": "pause", "cmd_seq": 1, "data": {"paused": True}}
        await nc.publish("command.genesis.pause", json.dumps(cmd).encode())

        msg = await asyncio.wait_for(sub.next_msg(), timeout=2.0)
        ack = json.loads(msg.data.decode())
        assert ack["action"] == "pause"
        assert ack["status"] == "ok"
    finally:
        await nc.close()
```

**Step 3: Write test_safety_video_gate**

```python
"""Smoke test: stop SHM writes, confirm telemetry.safety.video_gate fires."""
import asyncio
import json
import pytest
import nats as nats_client

@pytest.mark.asyncio
async def test_safety_video_gate():
    nc = await nats_client.connect("nats://localhost:4222")
    try:
        sub = await nc.subscribe("telemetry.safety.video_gate")
        # Send a velocity command to trigger gate check (video will be stale)
        cmd = {"action": "set_cmd_vel", "cmd_seq": 999, "data": {"linear_x": 1.0, "linear_y": 0, "angular_z": 0}}
        # Wait for gate to fire (transport checks on command receipt)
        await nc.publish("command.genesis.set_cmd_vel", json.dumps(cmd).encode())

        # The transport-server should publish video_gate within ~1s
        msg = await asyncio.wait_for(sub.next_msg(), timeout=3.0)
        gate = json.loads(msg.data.decode())
        assert gate["gated"] is True
    finally:
        await nc.close()
```

**Step 4: Write remaining tests**

Follow the same pattern for `test_cmd_ttl_decay` and `test_safety_state_authority`.

**Step 5: Commit**

```bash
git add tests/integration/
git commit -m "test: add smoke tests for WS delivery, NATS roundtrip, safety gate, TTL decay"
```

---

## Summary: Task Dependencies

```
Task 1 (protocol constants)
  ├── Task 2 (transport 0x04 handler) ──► Task 3 (video gate)
  ├── Task 4 (sim NATS client) ──────────► Task 9 (H.264 encoder)
  ├── Task 5 (frontend commands) ────────► Task 6 (safety UI)
  │                                        ├── Task 8 (JPEG codec path)
  ├── Task 7 (delete Socket.io relay)
  ├── Task 10 (fix defaults)
  ├── Task 11 (Caddyfile)
  └── Task 12 (docs) ──► Task 13 (smoke tests)
```

Tasks 2, 4, 5, 7, 10, 11 can be parallelized after Task 1.
