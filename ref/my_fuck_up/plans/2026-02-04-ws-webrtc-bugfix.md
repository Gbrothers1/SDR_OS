# WebSocket & WebRTC Bugfix Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Fix all WebSocket and WebRTC bugs found during audit — missing Socket.io event forwarding, broken WebRTC ICE/track handling, and misaligned defaults.

**Architecture:** Three independent fix areas: (1) server.js event forwarding list, (2) WebRTC client+server signaling, (3) default settings alignment across frontend and backend. Each area is a separate task that can be committed independently.

**Tech Stack:** React (GenesisContext.jsx), Node.js/Socket.io (server.js), Python/asyncio/aiortc (bridge_server.py), YAML config

---

## Task 1: Add missing events to server.js GENESIS_EVENTS

The `GENESIS_EVENTS` array in `server.js` controls which Socket.io events get forwarded between clients (browser <-> bridge). Nine events used in production are missing, which silently breaks robot loading, unloading, policy management, and init status.

**Files:**
- Modify: `server.js:26-57` (the GENESIS_EVENTS array)

**Step 1: Add the 9 missing events to the GENESIS_EVENTS array**

Add these events to the array in `server.js`, after the existing entries and before the closing bracket:

```javascript
'genesis_unload_robot',
'genesis_robot_loaded',
'genesis_robot_unloaded',
'genesis_robot_load_failed',
'genesis_memory_estimate',
'genesis_init_status',
'genesis_policy_list',
'genesis_policy_load_status',
'genesis_list_policies',
'genesis_get_memory_estimate',
'genesis_get_init_status',
```

Note: `genesis_get_memory_estimate`, `genesis_get_init_status`, and `genesis_list_policies` are request events from the frontend that the bridge listens for. They need forwarding too.

**Step 2: Remove `genesis_identify` from GENESIS_EVENTS**

`genesis_identify` has a dedicated handler on lines 128-134 that does server-side bookkeeping (sets `isGenesisBridge`, broadcasts `genesis_status`). The generic forwarding handler additionally broadcasts the raw `genesis_identify` event to browser clients that don't listen for it. Remove it from the array to avoid the duplicate handler.

**Step 3: Verify no regressions**

Run: `node -c server.js` (syntax check)
Expected: No output (clean parse)

**Step 4: Commit**

```bash
git add server.js
git commit -m "fix: add 11 missing events to Socket.io forwarding list

Robot load/unload, policy management, memory estimates, and init
status events were missing from GENESIS_EVENTS, causing them to be
silently dropped by server.js. Also remove genesis_identify from
the generic list since it has a dedicated handler."
```

---

## Task 2: Fix WebRTC — ontrack registration timing

The `pc.ontrack` and `pc.onconnectionstatechange` handlers in `GenesisContext.jsx` are registered AFTER the offer/answer exchange completes. With fast connections, the `track` event can fire during `setRemoteDescription()` before the handler exists, causing the video stream to silently never appear.

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx:158-215`

**Step 1: Move ontrack and onconnectionstatechange before the offer/answer exchange**

In the `connectWebRTC` function (inside the `useEffect` starting at line 139), restructure so the event handlers are registered immediately after creating the `RTCPeerConnection`, before `createOffer()`:

```javascript
const connectWebRTC = async () => {
  try {
    const pc = new RTCPeerConnection({ iceServers });
    webrtcPcRef.current = pc;
    const localCandidates = [];
    pc.onicecandidate = (e) => {
      if (e.candidate) localCandidates.push(e.candidate);
    };
    // Register track/state handlers BEFORE offer/answer exchange
    pc.ontrack = (e) => {
      if (cancelled) return;
      if (e.streams && e.streams[0]) {
        setMediaStream(e.streams[0]);
        setWebrtcConnected(true);
      }
    };
    pc.onconnectionstatechange = () => {
      if (cancelled) return;
      if (pc.connectionState === 'connected') {
        setWebrtcConnected(true);
      } else if (pc.connectionState === 'failed' || pc.connectionState === 'closed' || pc.connectionState === 'disconnected') {
        setWebrtcConnected(false);
        setMediaStream(null);
        webrtcReconnectTimerRef.current = setTimeout(connectWebRTC, 3000);
      }
    };
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    // ... rest of ICE gathering and signaling unchanged ...
```

Remove the duplicate `pc.ontrack` (old line 199-205) and `pc.onconnectionstatechange` (old line 206-215) that were after the exchange.

**Step 2: Verify build**

Run: `npx webpack --mode development` (or `npm run build`)
Expected: Compiles without errors

**Step 3: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx
git commit -m "fix: register WebRTC ontrack before offer/answer exchange

The ontrack handler was set after setRemoteDescription and
addIceCandidate completed. Fast connections could fire the track
event before the handler existed, silently dropping the video stream."
```

---

## Task 3: Fix WebRTC — server-side ICE candidate extraction

The bridge server creates an empty `candidates = []` list and returns it without populating it. For trickle ICE support, the server must extract gathered candidates from the peer connection and return them in the signaling response.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:1718-1731` (inside `_handle_webrtc_offer`)

**Step 1: Extract ICE candidates after gathering completes**

Replace the empty candidates logic (lines 1720-1731) with code that extracts candidates from the local description:

```python
answer = await pc.createAnswer()
await pc.setLocalDescription(answer)
# Wait for ICE gathering to complete
while pc.iceGatheringState != "complete":
    await asyncio.sleep(0.1)
# Extract gathered ICE candidates from local description
candidates = []
for line in pc.localDescription.sdp.splitlines():
    if line.startswith("a=candidate:"):
        candidates.append({
            "candidate": line[2:],  # strip "a=" prefix
            "sdpMid": "0",
            "sdpMLineIndex": 0,
        })
answer_sdp = pc.localDescription.sdp
answer_type = pc.localDescription.type
resp = web.json_response({
    "type": answer_type,
    "sdp": answer_sdp,
    "candidates": candidates,
})
resp.headers["Access-Control-Allow-Origin"] = "*"
return resp
```

**Step 2: Verify syntax**

Run: `python3 -c "import ast; ast.parse(open('genesis_bridge/bridge_server.py').read())"`
Expected: No output (clean parse)

**Step 3: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "fix: extract ICE candidates in WebRTC signaling response

The server returned an empty candidates array. For trickle ICE
support, parse gathered candidates from the local SDP and include
them in the signaling response."
```

---

## Task 4: Always start WebRTC signaling server when aiortc is available

The `webrtc_enabled` flag defaults to `False`, so the signaling server never starts. Users who toggle to WebRTC in the frontend settings get silent connection failures. Remove the flag and always start the signaling server when aiortc/aiohttp are installed.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:354-355` (BridgeConfig dataclass)
- Modify: `genesis_bridge/bridge_server.py:2490-2502` (run method, WebRTC startup)
- Modify: `genesis_bridge/bridge_server.py:2641` (CLI arg `--webrtc`)

**Step 1: Remove `webrtc_enabled` from BridgeConfig**

In the `BridgeConfig` dataclass (line 354-355), remove:
```python
    # WebRTC settings (optional)
    webrtc_enabled: bool = False
```

Keep `webrtc_port`:
```python
    # WebRTC settings
    webrtc_port: int = 9092
```

**Step 2: Update the `run()` method to start WebRTC unconditionally**

Replace lines 2490-2502:
```python
        # WebRTC: start signaling server when aiortc/aiohttp are available
        if _aiortc and _aiohttp is not None and GenesisWebRTCVideoTrack is not None:
            self._webrtc_new_frame_event = asyncio.Event()
            webrtc_app = self._create_webrtc_app()
            if webrtc_app is not None:
                runner = web.AppRunner(webrtc_app)
                await runner.setup()
                site = web.TCPSite(runner, self.config.ws_host, self.config.webrtc_port)
                await site.start()
                logger.info(f"WebRTC signaling server listening on http://{self.config.ws_host}:{self.config.webrtc_port}")
        else:
            if _aiortc is False:
                logger.info("aiortc not installed - WebRTC streaming unavailable (install with: pip install aiortc aiohttp)")
```

**Step 3: Remove `--webrtc` CLI arg and `webrtc_enabled` from cli_overrides**

In the `main()` function, remove:
```python
parser.add_argument("--webrtc", action="store_true", help="Enable WebRTC signaling server")
```
And remove from `cli_overrides`:
```python
"webrtc_enabled": args.webrtc,
```

**Step 4: Verify syntax**

Run: `python3 -c "import ast; ast.parse(open('genesis_bridge/bridge_server.py').read())"`
Expected: No output (clean parse)

**Step 5: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "fix: always start WebRTC signaling when aiortc is available

Remove the webrtc_enabled flag. The signaling server is zero-cost
when idle and must be running for frontend live-switching between
WebSocket and WebRTC streaming backends."
```

---

## Task 5: Align default settings — frontend controls all

Frontend is the source of truth for streaming defaults. Align backend code defaults and YAML config to match frontend values. The frontend pushes `genesis_settings` on connect, but the backend should start with matching values to avoid a quality/fps swing during the race window.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:362-364` (BridgeConfig defaults)
- Modify: `configs/genesis_bridge.yaml:4-6` (YAML overrides)
- Modify: `src/client/contexts/SettingsContext.jsx:84` (frontend streamFps)
- Modify: `src/client/components/Settings.jsx:82` (frontend streamFps duplicate)
- Modify: `src/client/contexts/GenesisContext.jsx:355` (getSetting default)
- Modify: `src/client/components/Settings.jsx:1163-1164` (slider default+max)

**Step 1: Update BridgeConfig code defaults**

In `genesis_bridge/bridge_server.py`, change lines 362-364:
```python
    camera_res: tuple = (1280, 720)
    jpeg_quality: int = 80
    target_fps: int = 60
```

**Step 2: Update YAML config to match**

In `configs/genesis_bridge.yaml`, change lines 4-6:
```yaml
  camera_res: [1280, 720]
  jpeg_quality: 80
  target_fps: 60
```

**Step 3: Update frontend streamFps default to 60**

In `src/client/contexts/SettingsContext.jsx` line 84, change:
```javascript
    streamFps: 60,
```

In `src/client/components/Settings.jsx` line 82, change:
```javascript
    streamFps: 60,
```

In `src/client/contexts/GenesisContext.jsx` line 355, change:
```javascript
  const genesisStreamFps = getSetting('genesis', 'streamFps', 60);
```

In `src/client/components/Settings.jsx` lines 1163-1164, update the default and slider max:
```javascript
                      value={getSetting('genesis', 'streamFps', 60)}
                      onChange={(e) => handleSliderChange('genesis', 'streamFps', e.target.value, 5, 120)}
```
Also update the display on line 1166:
```javascript
                    <span className="setting-value">{getSetting('genesis', 'streamFps', 60)} fps</span>
```

**Step 4: Verify build**

Run: `npx webpack --mode development`
Expected: Compiles without errors

Run: `python3 -c "import ast; ast.parse(open('genesis_bridge/bridge_server.py').read())"`
Expected: No output

**Step 5: Commit**

```bash
git add genesis_bridge/bridge_server.py configs/genesis_bridge.yaml \
  src/client/contexts/SettingsContext.jsx src/client/components/Settings.jsx \
  src/client/contexts/GenesisContext.jsx
git commit -m "fix: align streaming defaults across frontend and backend

Frontend is source of truth. Set all layers to:
- JPEG quality: 80
- Camera resolution: 1280x720
- Target FPS: 60
- Metrics rate: keep at 5 Hz (already aligned in code defaults)

Previously backend code defaulted to 70% quality / 960x540 / 60fps,
YAML overrode to 100% / 1280x720 / 120fps, and frontend pushed 80%
/ 1280x720 / 30fps on connect — causing a visible quality swing."
```

---

## Task 6: Clean up stale WebSocket clients on broadcast failure

In `bridge_server.py`, `broadcast_frame` catches send exceptions via `asyncio.gather(return_exceptions=True)` but doesn't remove failed clients from `ws_clients`. Dead clients accumulate send errors each frame until their `handle_websocket` loop eventually breaks.

**Files:**
- Modify: `genesis_bridge/bridge_server.py:1647-1676` (broadcast_frame method)

**Step 1: Remove failed clients after broadcast**

After the `asyncio.gather` call in `broadcast_frame`, close and remove clients that threw exceptions:

```python
    async def broadcast_frame(self, frame_bytes: Optional[bytes]):
        """Broadcast JPEG frame to all WebSocket clients."""
        if frame_bytes is None:
            logger.debug("broadcast_frame: Skipping None frame")
            self.frames_failed += 1
            return

        if not self.ws_clients:
            logger.debug("broadcast_frame: No WebSocket clients connected")
            return

        # Track frame statistics
        frame_size = len(frame_bytes)
        self.frames_sent += 1
        self.total_frame_bytes += frame_size

        # Snapshot clients to avoid mutation during iteration
        clients = list(self.ws_clients)

        # Send binary frame to all clients
        send_start = time.time()
        results = await asyncio.gather(
            *[client.send(frame_bytes) for client in clients],
            return_exceptions=True
        )
        self._timing_acc["broadcast"] += time.time() - send_start
        self._timing_acc["broadcast_count"] += 1

        # Remove failed clients
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                self.ws_clients.discard(client)
                self.frames_failed += 1
                try:
                    await client.close()
                except Exception:
                    pass
```

Apply the same pattern to `broadcast_metrics`:

```python
    async def broadcast_metrics(self, metrics: Dict):
        """Broadcast JSON metrics to all WebSocket clients."""
        if not self.ws_clients:
            return

        clients = list(self.ws_clients)
        message = json.dumps(metrics)
        results = await asyncio.gather(
            *[client.send(message) for client in clients],
            return_exceptions=True
        )
        for client, result in zip(clients, results):
            if isinstance(result, Exception):
                self.ws_clients.discard(client)
                try:
                    await client.close()
                except Exception:
                    pass
```

**Step 2: Verify syntax**

Run: `python3 -c "import ast; ast.parse(open('genesis_bridge/bridge_server.py').read())"`
Expected: No output

**Step 3: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "fix: remove stale WebSocket clients on broadcast failure

Previously dead clients stayed in ws_clients until their handler
loop broke, accumulating send errors on every frame. Now failed
clients are discarded and closed immediately after broadcast."
```

---

## Summary of all changes

| # | Bug | Files | Impact |
|---|-----|-------|--------|
| 1 | 9 missing Socket.io events + 1 stale duplicate | `server.js` | Robot load/unload, policy mgmt broken |
| 2 | WebRTC ontrack registered too late | `GenesisContext.jsx` | Video stream silently drops |
| 3 | WebRTC empty ICE candidates (no trickle ICE) | `bridge_server.py` | WebRTC unreliable on restricted networks |
| 4 | WebRTC signaling server disabled by default | `bridge_server.py` | Frontend WebRTC toggle broken |
| 5 | Streaming defaults misaligned across 3 layers | 5 files | Quality/FPS swing on connect |
| 6 | Stale WS clients not cleaned up | `bridge_server.py` | Accumulated send errors in logs |
