# Phase 3: WebRTC + End-to-End Video Pipeline

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement the plan generated from this design.

**Goal:** Wire the complete video pipeline from genesis-sim GPU render through the Rust transport server to the browser, with both WebSocket and WebRTC delivery paths plus DataChannel for gamepad control.

**Architecture:**

```
genesis-sim (CUDA + NVENC)
    |
    v
SHM ringbuffer (/dev/shm/sdr_os_ipc/frames)
    |  mmap read (Phase 2, done)
    v
transport-server (Rust/tokio/axum)
    |
    +-- /stream/ws  Binary WebSocket
    |     |-- 0x01: video (32-byte LE header + Annex-B NALs)
    |     |-- 0x02: telemetry (NATS relay, existing)
    |     +-- 0x03: signaling (JSON SDP/ICE)
    |
    +-- WebRTC PeerConnection (webrtc-rs)
          |-- Video track: H.264 RTP (NAL units from SHM)
          +-- DataChannels:
                |-- "control"  (unordered, maxRetransmits=0) - axes
                +-- "commands" (ordered, reliable) - estop/mode
```

**Tech Stack:** Rust 1.75+, webrtc-rs 0.12, tokio 1.x, axum 0.7 (existing), React 18, WebCodecs API, RTCPeerConnection.

---

## Goals

1. Browser renders live H.264 video from genesis-sim over WebSocket (baseline path)
2. Browser can upgrade to WebRTC for lower-latency video delivery
3. Gamepad control commands flow over DataChannel when WebRTC is active
4. Both paths coexist - switch rendering instantly without reconnecting
5. Late-joiners see video within one keyframe interval on either path

## Non-Goals

- HEVC/H.265 browser decode (no browser support worth targeting yet)
- Adaptive bitrate / quality negotiation (Phase 5+ concern)
- Multi-camera WebRTC (single video track only)
- Replacing Socket.io for non-sim paths (real robot mode stays on ROSLIB)
- TURN server / NAT traversal (transport server and browser are on the same LAN or localhost)
- Recording / playback (Phase 5 JetStream concern)

---

## Protocol

### Wire Format (WebSocket Binary Frames)

All messages are binary WebSocket frames. First byte discriminates:

#### 0x01 - Video Frame

```
[0x01][32-byte LE header][Annex-B payload]

Header (little-endian, matches SHM protocol):
  [0:8]   frame_id    u64   Application-level frame ID
  [8:16]  frame_seq   u64   Monotonic sequence (lap detection)
  [16:20] size        u32   Payload size in bytes
  [20:22] flags       u16   0x0001 = keyframe
  [22:24] codec       u16   1 = H.264, 2 = HEVC
  [24:28] crc32       i32   Payload CRC (signed, Python compat)
  [28:32] reserved    u32   Zero
```

Payload is raw Annex-B H.264 access unit (start-code-delimited NAL units). The browser parses the 32-byte header for `frame_id`, `flags`, and `codec`, then hands only the payload bytes to the WebCodecs `VideoDecoder`.

#### 0x02 - Telemetry

```
[0x02][u16 LE subject_len][subject bytes][payload]
```

Unchanged from Phase 2. NATS subject + arbitrary payload (typically JSON).

#### 0x03 - Signaling

```
[0x03][JSON payload]
```

No length prefix needed - WebSocket frames the message. JSON structure:

```json
{"type": "offer",  "session_id": "uuid", "sdp": "v=0\r\n..."}
{"type": "answer", "session_id": "uuid", "sdp": "v=0\r\n..."}
{"type": "ice",    "session_id": "uuid", "candidate": "...", "sdpMid": "0", "sdpMLineIndex": 0}
{"type": "close",  "session_id": "uuid"}
```

Every signaling message includes `session_id` (client-generated `crypto.randomUUID()` per negotiation attempt). Server rejects ICE candidates with mismatched `session_id`. New WebSocket connection = old `RtcSession` torn down.

### DataChannel Messages

#### "control" (unordered, maxRetransmits=0)

Gamepad axes spam. Binary, compact:

```
[u8 axis_count][f32 LE axis0][f32 LE axis1]...[u32 LE button_bitmask]
```

Rate-capped at 60 msg/sec server-side (one per animation frame). Max 256 bytes. Excess dropped silently. Published to NATS `control.gamepad` (plain pub, no JetStream).

#### "commands" (ordered, reliable)

Discrete events:

```json
{"type": "estop"}
{"type": "mode", "mode": "teleop"}
{"type": "button", "id": 0, "pressed": true}
```

Rate-capped at 10 msg/sec server-side. Max 1KB. Published to NATS `control.commands` (plain pub, no JetStream).

---

## Browser

### Connection Setup (GenesisContext.jsx)

Replace the ghost `ws://localhost:9091` genesis bridge connection with the real transport server:

```javascript
const wsProto = location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsUrl = settings.genesis.bridgeWsUrl
  || `${wsProto}//${location.host}/stream/ws`;
```

On WebSocket `message` event, route by first byte:

```javascript
const view = new DataView(buffer);
const type = view.getUint8(0);
switch (type) {
  case 0x01: handleVideoFrame(buffer);     break;
  case 0x02: handleTelemetry(buffer);      break;
  case 0x03: handleSignaling(buffer);      break;
}
```

### Video Frame Handling (0x01)

Parse the 32-byte LE header starting at offset 1:

```javascript
const flags    = view.getUint16(21, true);  // little-endian
const codec    = view.getUint16(23, true);
const frameId  = Number(view.getBigUint64(1, true));
const isKeyframe = (flags & 0x0001) !== 0;
const payload  = new Uint8Array(buffer, 33);  // skip type byte + 32-byte header
```

Feed `payload` (raw Annex-B) to `H264Decoder`. The decoder receives the bytes and the `isKeyframe` flag - nothing else.

**Keyframe gating:** On connect, set `awaitingKeyframe = true`. Drop all video frames until one arrives with `isKeyframe === true`. This is WS-path only; RTC path has its own gating server-side.

### H264Decoder.js Changes

- Remove `parseFrameHeader()` and the 12-byte BE header assumption
- `decode(payload, isKeyframe)` accepts raw Annex-B `Uint8Array` and a boolean
- Keep: WebCodecs `VideoDecoder` setup, codec detection (`avc1.42E01E` etc.), backpressure queue (max 3), canvas rendering, `getStats()`
- Add `reset()` method: flush internal queue, call `decoder.reset()`, set `awaitingKeyframe = true`
- Call `reset()` on: mode switch, stream discontinuity (frame_seq gap detected from header), WS reconnect

### WebRTC Upgrade Path

Triggered by user toggle in settings or auto-upgrade preference.

1. Create `RTCPeerConnection` with default `sdpSemantics: "unified-plan"` and STUN config from settings
2. Create DataChannels BEFORE creating offer:
   - `pc.createDataChannel("control", { ordered: false, maxRetransmits: 0 })`
   - `pc.createDataChannel("commands", { ordered: true })`
3. `pc.createOffer()` → `pc.setLocalDescription(offer)`
4. Wait for ICE gathering (or trickle)
5. Send `[0x03]{"type":"offer","sdp":"...","session_id":"..."}` over existing WS
6. On `0x03` answer: `pc.setRemoteDescription(answer)`
7. On `0x03` ice: `pc.addIceCandidate(candidate)`
8. `pc.ontrack`: `video.srcObject = new MediaStream([event.track])`
9. Wire `control` DC `onopen` → start sending gamepad axes
10. Wire `commands` DC `onopen` → start sending button events

**On `pc.connectionStateChange` → `failed`/`disconnected`:** close PeerConnection, null out `rtcSession`, keep WebSocket alive. Browser falls back to WS rendering automatically. User can re-trigger WebRTC upgrade.

### SimViewer.jsx Rendering

Two mutually exclusive render modes:

- **WS mode (default):** `<canvas>` element. WebCodecs decodes Annex-B, draws frames. Active when no WebRTC video track or user explicitly chose WS.
- **WebRTC mode:** `<video srcObject={mediaStream}>` element with HUD `<canvas>` overlaid (z-index above, pointer-events none). Active when `pc.connectionState === "connected"` and video track is live.

Both WS and RTC connections can be alive simultaneously. Only one renders. Switching is instant (show/hide elements). When switching modes, call `h264Decoder.reset()` to flush stale frames from the WebCodecs queue. Never feed the WebCodecs decoder while in RTC render mode.

### Stats Strip (new component)

Small overlay at bottom of SimViewer, always visible:

| Stat | Source |
|------|--------|
| FPS | Decode frame count / second (WS) or `video.requestVideoFrameCallback` (RTC) |
| frame_id | Last seen from 32-byte header (WS) or N/A (RTC) |
| Keyframe | Time since last keyframe seen |
| WS KB/s | Sum of binary message sizes / second |
| RTC KB/s + RTT | `pc.getStats()` → `inbound-rtp` bytesReceived, `candidate-pair` currentRoundTripTime |
| Mode | `WS` or `RTC` badge |

Rendered as a single-line strip with monospace font. Toggle visibility with a hotkey or settings flag.

---

## Server

### Session State (extended unified_send_loop)

```rust
struct ClientSession {
    // Phase 2 (existing)
    ws_sender: Arc<Mutex<SplitSink<WebSocket, Message>>>,
    video_rx: broadcast::Receiver<Arc<Frame>>,
    telemetry_rx: broadcast::Receiver<Arc<TelemetryMsg>>,
    ws_video_state: ClientState,  // AwaitingKeyframe / Streaming

    // Phase 3 (new)
    rtc_session: Option<RtcSession>,
}

struct RtcSession {
    session_id: String,
    peer_connection: Arc<RTCPeerConnection>,
    video_track: Arc<TrackLocalStaticRTP>,
    control_dc: Arc<DataChannel>,
    commands_dc: Arc<DataChannel>,
    nal_cache: NalCache,
    rtp_timestamp_base: Instant,
    rtc_video_state: ClientState,  // independent from ws_video_state
    cancel: CancellationToken,
}

struct NalCache {
    sps: Option<Vec<u8>>,  // NAL type 7
    pps: Option<Vec<u8>>,  // NAL type 8
}
```

### Unified Send Loop (revised)

The `tokio::select!` loop gains a third arm: reading from `ws_receiver`:

```rust
loop {
    tokio::select! {
        // Video frame from broadcast
        result = video_rx.recv() => {
            // WS path: encode + send if ws_video_state == Streaming
            // RTC path: packetize + write to track if rtc_video_state == Streaming
        }

        // Telemetry from NATS relay
        result = telemetry_rx.recv() => {
            // Always send on WS (not gated by video state)
        }

        // Incoming WS message from client
        result = ws_receiver.next() => {
            // 0x03 signaling → handle offer/ice/close
            // Anything else → ignore
        }
    }
}
```

**Critical:** IDR timeout on WS video stops sending `0x01` frames and resets `ws_video_state` to `AwaitingKeyframe`. It does NOT close the WebSocket. Telemetry (`0x02`) and signaling (`0x03`) continue flowing. The client's UI stays informed even when video hiccups.

### NAL Unit Processing

On each video frame from SHM:

1. **Split Annex-B into NAL units** by scanning for `0x00 0x00 0x00 0x01` start codes
2. **Identify NAL types** from first byte after start code: `nal_type = byte & 0x1F`
   - 7 = SPS, 8 = PPS, 5 = IDR slice, 1 = non-IDR slice
3. **On keyframe (IDR):**
   - Extract and cache SPS/PPS NAL units in `NalCache`
   - Update `rtc_video_state` to `Streaming` if was `AwaitingKeyframe`
   - Write to RTP as three sequential samples (same timestamp): `[SPS] [PPS] [IDR]`
4. **On delta frame:**
   - If `rtc_video_state == AwaitingKeyframe`: drop (don't write to track)
   - Otherwise: write NAL units as RTP sample
5. **On new RTC subscriber:** `rtc_video_state = AwaitingKeyframe`, wait for next keyframe from SHM before writing anything to the track

No STAP-A aggregation. Three sequential samples with the same timestamp is simpler and universally compatible. webrtc-rs handles FU-A fragmentation for NAL units exceeding MTU.

### RTP Timestamps

90kHz clock derived from wall time at frame read:

```rust
let elapsed_ns = (Instant::now() - session.rtp_timestamp_base).as_nanos();
let rtp_ts = (elapsed_ns * 90_000 / 1_000_000_000) as u32;
```

This gives the browser's jitter buffer a monotonic, paced clock. We use read-time (not SHM write-time) because the SHM header has no nanosecond timestamp field - `frame_id` is a counter, not a clock.

The `rtp_timestamp_base` is set to `Instant::now()` when the `RtcSession` is created, so timestamps start near zero and grow monotonically.

### DataChannel → NATS Relay

```rust
// "control" DC onmessage
control_dc.on_message(|msg| {
    // Rate limit: 60 msg/sec, max 256 bytes
    if rate_limiter.check() && msg.len() <= 256 {
        nats_client.publish("control.gamepad", msg).await;
    }
});

// "commands" DC onmessage
commands_dc.on_message(|msg| {
    // Rate limit: 10 msg/sec, max 1KB
    if rate_limiter.check() && msg.len() <= 1024 {
        nats_client.publish("control.commands", msg).await;
    }
});
```

Plain NATS publish only. No JetStream acknowledgment for either channel. JetStream is reserved for training data replay (Phase 5).

### Signaling Handler

```rust
fn handle_signaling(msg: &[u8], session: &mut ClientSession) {
    let json: SignalingMessage = serde_json::from_slice(&msg[1..])?;

    match json.msg_type.as_str() {
        "offer" => {
            // Tear down existing RtcSession if session_id differs
            // Create RTCPeerConnection with webrtc-rs
            // Add video track (H.264, 90kHz clock)
            // Set remote description (offer)
            // Create answer, set local description
            // Send 0x03 answer back on WS
            // Register on_ice_candidate → send 0x03 ice on WS
            // Register on_data_channel → wire control/commands handlers
            // Register on_connection_state_change → cleanup on failed
        }
        "ice" => {
            // Verify session_id matches
            // Add ICE candidate to PeerConnection
        }
        "close" => {
            // Verify session_id matches
            // Tear down RtcSession, cancel writer task
        }
    }
}
```

### PeerConnection Cleanup

On `pc.on_peer_connection_state_change`:
- `connected`: log, update metrics
- `disconnected` or `failed`: set `cancel` token → RTP writer task exits → `RtcSession` dropped → track and DCs cleaned up → WebSocket stays alive → `rtc_session = None`
- Client can send a new `0x03` offer to re-establish

---

## Failure Modes

| Failure | Detection | Response |
|---------|-----------|----------|
| SHM file missing | `Path::exists()` poll (existing) | Reader loop waits, logs. No frames broadcast. |
| SHM torn read | `seq1 != seq2` | Retry immediately (spin_loop). No client impact. |
| WS video IDR timeout (5s) | `since.elapsed() > timeout` | Stop sending `0x01` to that client, reset to AwaitingKeyframe. Telemetry + signaling continue. |
| RTC video IDR timeout (5s) | Same check on `rtc_video_state` | Stop writing to RTP track, reset to AwaitingKeyframe. WS path unaffected. |
| WS client lagged | `RecvError::Lagged(n)` | Reset `ws_video_state` to AwaitingKeyframe, drop until keyframe. |
| RTC PeerConnection failed | `on_connection_state_change` | Cancel RTP writer, drop `RtcSession`. WS stays alive. |
| ICE session_id mismatch | Compare on every `0x03` message | Reject message, log warning. |
| DataChannel rate exceeded | Token bucket (60/s or 10/s) | Drop excess silently. |
| DataChannel oversized msg | `msg.len() > max` | Drop, log warning. |
| NATS unavailable | `telemetry_relay` error (existing) | Auto-reconnect loop (5s). Telemetry paused, video unaffected. |
| WebCodecs decoder error | `decoder.onerror` callback | Call `decoder.reset()`, set `awaitingKeyframe = true`. |
| Browser mixed-content block | `wss://` auto-detection | `location.protocol === 'https:' ? 'wss:' : 'ws:'` |
| Mode switch stale frames | User toggles WS ↔ RTC | Call `h264Decoder.reset()` to flush queue. Don't feed decoder in RTC mode. |
| NAL cache miss (no SPS/PPS) | First frame is delta, no cache yet | RTC subscriber stays in AwaitingKeyframe until a keyframe arrives. |

---

## Testing Matrix

| Layer | Test | Method | Automated |
|-------|------|--------|-----------|
| NAL parser: split Annex-B | Known H.264 bitstream → expected NAL count + types | Rust unit test | Yes |
| NAL parser: identify SPS/PPS/IDR | Fixture with all NAL types | Rust unit test | Yes |
| SPS/PPS cache | Cache on keyframe, prepend on new subscriber | Rust unit test | Yes |
| RTP timestamps | Monotonic 90kHz from Instant | Rust unit test | Yes |
| Signaling parse | Valid/invalid JSON, session_id mismatch | Rust unit test | Yes |
| Rate limiter | 60/s cap, burst, steady state | Rust unit test | Yes |
| Signaling round-trip | WS client sends offer, gets answer | `tokio-tungstenite` integration test | Yes |
| DataChannel → NATS | DC message appears on NATS subject | Requires NATS + webrtc-rs test | CI with NATS container |
| Browser 0x01/02/03 routing | ArrayBuffer fixtures, dispatch | Jest unit test | Yes |
| Browser header parse | 32-byte LE header → correct values | Jest unit test | Yes |
| H264Decoder reset | Mode switch flushes, reconnect resets | Jest unit test | Yes |
| Stats strip | Renders FPS, frame_id, mode | Playwright snapshot | CI |
| WS end-to-end | SHM writer → transport → browser renders | Docker compose + headless Chrome | CI (GPU runner) |
| WebRTC end-to-end | SHM → transport → WebRTC → browser `<video>` | Docker compose + headless Chrome | CI (GPU runner) |

---

## Rollout Flags

All flags are environment variables (server) or settings keys (browser). Default = safe/off.

### Server (environment variables)

| Flag | Default | Effect |
|------|---------|--------|
| `SDR_WEBRTC_ENABLED` | `false` | Enable WebRTC session handling. When false, `0x03` offer messages are ignored. WS-only mode. |
| `SDR_DC_CONTROL_RATE` | `60` | Max control DC messages/sec per client |
| `SDR_DC_COMMANDS_RATE` | `10` | Max commands DC messages/sec per client |
| `SDR_RTP_MTU` | `1200` | RTP payload MTU for FU-A fragmentation |
| `SDR_ICE_STUN_URL` | (empty) | STUN server URL. Empty = no STUN (LAN-only, host candidates only). |

### Browser (SettingsContext)

| Key | Default | Effect |
|-----|---------|--------|
| `genesis.streamBackend` | `"websocket"` | `"websocket"` or `"webrtc"` - which path to render |
| `genesis.webrtcAutoUpgrade` | `false` | Automatically attempt WebRTC after WS connects |
| `genesis.showStats` | `false` | Show the stats strip overlay |
| `genesis.bridgeWsUrl` | `""` | Override WS URL (empty = auto-detect from page origin) |
| `genesis.webrtcStunUrls` | `["stun:stun.l.google.com:19302"]` | STUN servers for ICE |

### Phased Rollout

1. **Phase 3a (WS last-mile):** Ship browser changes + Caddy route. `SDR_WEBRTC_ENABLED=false`. Validates the full WS video pipeline end-to-end.
2. **Phase 3b (WebRTC):** Enable `SDR_WEBRTC_ENABLED=true`. Test WebRTC path alongside WS. Compare latency via stats strip.
3. **Phase 3c (DataChannel):** Wire gamepad over DC. Test with `genesis.streamBackend="webrtc"`. Socket.io control path remains as fallback.
