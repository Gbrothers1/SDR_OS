# Phase 2: Rust Transport Server - Design

**Date:** 2026-02-05
**Status:** Approved
**Author:** Claude + Ethan

## Purpose

The transport server is the bridge between the SHM ringbuffer (written by genesis-sim) and browser clients. It reads encoded H.264/HEVC frames from shared memory, fans them out over WebSocket to connected clients, and relays NATS telemetry on the same connection using a binary protocol. It's the only service that touches both the SHM data plane and the client-facing network.

**Scope:** WebSocket frame delivery + NATS telemetry relay. No WebRTC (deferred to Phase 4).

## Core Data Flow

```
SHM ringbuffer (/dev/shm/sdr_os_ipc/frames)
      │
      ▼  adaptive poll (mmap read)
┌─────────────────────┐
│  transport-server    │
│  (Rust / tokio)      │
│                      │
│  ┌──── SHM Reader ──────── reads seq + header + payload
│  │                   │
│  ├──── Fanout ──────────── broadcast channel, per-client drop
│  │                   │
│  ├──── NATS Sub ────────── telemetry.* → binary envelope
│  │                   │
│  └──── axum WS ─────────── /stream/ws endpoint
│                      │
└─────────────────────┘
      │
      ▼  WebSocket (all binary: type byte discriminates video vs telemetry)
   Browser clients
```

### Network Position

- Lives on the **backplane** network only
- Caddy reverse-proxies `/stream/*` to `transport-server:8080`
- Never directly exposed to the edge

---

## SHM Reader

The reader mmaps the same file that genesis-sim writes to (`SDR_SHM_PATH`, default `/dev/shm/sdr_os_ipc/frames`). It implements the same atomic commit protocol as the Python reader from Phase 1, but in Rust with proper memory ordering.

### Read Loop

1. Read `seq1` (8 bytes at offset 0) with **acquire ordering**
2. If `seq1 == last_seq`: no new frame, backoff
3. Read `write_len` (8 bytes at offset 8) - **validate**: `write_len > 0`, `write_len <= max_payload`, `16 + write_len <= mmap_len`; retry on any violation
4. Read `frame_data` (header + payload starting at offset 16)
5. Read `seq2` with **acquire ordering** - if `seq1 != seq2`, writer was mid-write, discard and retry
6. Parse 32-byte header with **explicit little-endian packing** (no reliance on struct layout)
7. If CRC enabled: verify CRC32 of payload; on mismatch discard and increment error counter

### Adaptive Backoff

Three states to avoid burning CPU when idle while staying responsive when frames flow:

- **Spinning** (frames flowing): `std::hint::spin_loop()` between reads, ~microsecond latency
- **Yielding** (no frame for >100μs): `tokio::task::yield_now()`, lets other tasks run
- **Sleeping** (no frame for >50ms): `tokio::time::sleep(1ms)`, minimal CPU

Transitions back to Spinning on first new frame. When no clients are connected, jumps directly to Sleeping and only peeks at the sequence number (no CRC, no frame allocation).

### Lap Detection

- Track `last_frame_seq` as plain `u64` (reader-local state, not shared)
- If `header.frame_seq` jumps past expected: writer lapped, log warning, resync
- On resync: set `awaiting_keyframe = true`, drop all non-IDR frames until keyframe arrives
- Keyframe/IDR detection uses explicit flag check (`flags & FLAG_KEYFRAME != 0`), deterministic

### Key Types

```rust
pub struct ShmReader {
    mmap: Mmap,
    last_seq: u64,         // Reader-local, not atomic
    last_frame_seq: u64,
    awaiting_keyframe: bool,
    state: PollState,
    crc_enabled: bool,
}

enum PollState {
    Spinning { since: Instant },
    Yielding { since: Instant },
    Sleeping,
}
```

### Safety Notes

- Writer's sequence publish must use **release ordering**; reader uses **acquire** on both seq reads
- Header parsing uses `u64::from_le_bytes()` etc., never `transmute` or `repr(C)` struct reads
- All length fields are bounds-checked before any slice indexing
- CRC32 is toggleable via `SDR_CRC_ENABLED` for trusted SHM paths where CPU matters

---

## Client Fanout

Each connected client gets a `tokio::sync::broadcast::Receiver<Arc<Frame>>`. The `Arc` avoids copying frame data per-client.

### Broadcast Flow

1. SHM reader produces a `Frame` (header + payload bytes)
2. Wraps in `Arc<Frame>`, sends to `broadcast::Sender`
3. Each client task receives from its `Receiver`
4. Client task serializes to WebSocket binary message with type prefix byte

### Backpressure (Per-Client Lag Detection)

- `broadcast` channel capacity: 64 frames (~1 second at 60fps)
- When a client's receiver returns `RecvError::Lagged(n)`:
  - Log the lag count, increment per-client metrics counter
  - Set client state to `AwaitingKeyframe`
  - Drop all incoming frames until the next keyframe/IDR
  - Publish IDR request to NATS (`control.request_idr`)
- **IDR coalescing**: rate-limit IDR requests globally (one per `SDR_IDR_COALESCE_MS`, default 500ms) so a single bad Wi-Fi client doesn't spam NATS
- **Hard timeout**: if no IDR arrives within 5 seconds of requesting one, disconnect the client (encoder may be stuck)

### Client Lifecycle

```
Connect → Upgrade to WS → Subscribe to broadcast
    → AwaitingKeyframe (skip until IDR)
    → Streaming (forward all frames)
    → Lagged? → back to AwaitingKeyframe
    → IDR timeout? → Disconnect
    → Disconnect → drop Receiver (automatic cleanup)
```

### No-Client Optimization

- Track connected client count with `AtomicUsize`
- When count drops to 0:
  - SHM reader switches to `Sleeping` state (1ms polls, peek seq only)
  - No CRC computation, no frame allocation
  - Avoids burning CPU for an empty room

---

## NATS Telemetry Relay

The transport server subscribes to NATS telemetry subjects and relays them to browser clients as binary WebSocket frames.

### Subscription

- Single `async-nats` subscriber on configurable subject filter (`SDR_TELEMETRY_SUBJECTS`, default `telemetry.>`)
- Subject filter acts as allowlist - only matched subjects are relayed
- Messages are opaque bytes - the relay doesn't parse or validate payloads

### Delivery

- Separate `broadcast::Sender<Arc<TelemetryMsg>>` from the video fanout
- Smaller channel capacity (32) - telemetry is lossy by nature
- On lag: silently drop old telemetry, no recovery protocol needed
- Per-subject drop counter in metrics for flood detection
- Cap per-message payload size (e.g., 64KB) - drop anything oversized

### Rate Limiting

- The ros-bridge already rate-limits per-topic on the NATS side (100-200Hz)
- Transport server does not add a second rate limit layer
- Metrics counter tracks messages/sec per subject for observability

---

## WebSocket Protocol

All WebSocket messages are **binary**. The first byte discriminates message type.

### Message Types

| First byte | Meaning | Layout |
|------------|---------|--------|
| `0x01` | Video frame | `[0x01][32-byte frame header][payload]` |
| `0x02` | Telemetry | `[0x02][u16 subject_len][subject bytes][payload]` |

### Browser Dispatch

```javascript
ws.onmessage = (e) => {
  const view = new DataView(e.data);
  const type = view.getUint8(0);
  if (type === 0x01) {
    // Video: parse 32-byte header at offset 1, payload at offset 33
    // Feed payload to VideoDecoder
  } else if (type === 0x02) {
    // Telemetry: read subject length, subject string, then payload
  }
};
```

### Size Caps

- Video frame max: `SDR_SHM_SIZE` (4MB default)
- Telemetry message max: 64KB
- Subject length max: 256 bytes
- Drop and log anything exceeding caps

---

## API Surface

| Route | Method | Purpose |
|-------|--------|---------|
| `/stream/ws` | GET (upgrade) | WebSocket - video + telemetry (binary) |
| `/health` | GET | `{"status":"ok","clients":N,"shm_seq":N,"last_frame_age_ms":N}` |
| `/metrics` | GET | Prometheus text format |

- `last_frame_age_ms` in `/health` distinguishes "stuck writer" from "no clients"
- `/health` doubles as Docker healthcheck target

---

## Configuration

All via environment variables with sensible defaults.

| Var | Default | Purpose |
|-----|---------|---------|
| `SDR_SHM_PATH` | `/dev/shm/sdr_os_ipc/frames` | SHM ringbuffer file |
| `SDR_SHM_SIZE` | `4194304` (4MB) | Buffer size in bytes |
| `SDR_NATS_URL` | `nats://nats:4222` | NATS server |
| `SDR_LISTEN_ADDR` | `0.0.0.0:8080` | HTTP listen address |
| `SDR_BROADCAST_CAPACITY` | `64` | Video broadcast channel size |
| `SDR_CRC_ENABLED` | `true` | Toggle CRC32 verification |
| `SDR_IDR_COALESCE_MS` | `500` | Min interval between IDR requests |
| `SDR_IDR_TIMEOUT_MS` | `5000` | Disconnect client if no IDR after request |
| `SDR_TELEMETRY_SUBJECTS` | `telemetry.>` | NATS subject filter (allowlist) |
| `SDR_TELEMETRY_MAX_SIZE` | `65536` | Max telemetry message size in bytes |

---

## Crate Structure

```
services/transport-server/
├── Cargo.toml
├── Dockerfile
├── src/
│   ├── main.rs              # Tokio entrypoint, config loading, server setup
│   ├── config.rs            # Env var parsing with defaults
│   ├── shm/
│   │   ├── mod.rs
│   │   ├── reader.rs        # Mmap consumer, adaptive backoff, lap detection
│   │   └── protocol.rs      # Frame header parsing (explicit LE, no transmute)
│   ├── fanout/
│   │   ├── mod.rs
│   │   ├── broadcast.rs     # tokio::sync::broadcast, Arc<Frame>
│   │   └── backpressure.rs  # Per-client lag detection, IDR coalescing
│   ├── transport/
│   │   ├── mod.rs
│   │   └── websocket.rs     # axum WS upgrade, binary frame dispatch
│   ├── nats/
│   │   ├── mod.rs
│   │   └── subscriber.rs    # Telemetry relay, subject filtering
│   ├── metrics.rs           # Prometheus counters/histograms
│   └── health.rs            # /health + /metrics endpoints
```

### Key Dependencies

```toml
[dependencies]
tokio = { version = "1", features = ["full"] }
axum = { version = "0.7", features = ["ws"] }
async-nats = "0.39"
memmap2 = "0.9"
crc32fast = "1.4"
prometheus = "0.13"
serde = { version = "1", features = ["derive"] }
serde_json = "1"
tracing = "0.1"
tracing-subscriber = "0.3"
```

---

## Docker Integration

### Compose Service

```yaml
transport-server:
  build:
    context: .
    dockerfile: services/transport-server/Dockerfile
  image: sdr_os-transport
  networks:
    - backplane
  expose:
    - "8080"
  volumes:
    - sdr_ipc:/dev/shm/sdr_os_ipc:ro
  environment:
    - SDR_SHM_PATH=/dev/shm/sdr_os_ipc/frames
    - SDR_NATS_URL=nats://nats:4222
  depends_on:
    nats:
      condition: service_healthy
  healthcheck:
    test: ["CMD", "wget", "--spider", "-q", "http://localhost:8080/health"]
    interval: 10s
    timeout: 3s
    retries: 3
```

- SHM mount is **read-only**
- Only on backplane network; Caddy handles ingress
- Healthcheck hits `/health`

### Dockerfile (Multi-Stage)

- **Stage 1 (builder):** `rust:1.84-slim` → `cargo build --release`
- **Stage 2 (runtime):** `debian:bookworm-slim` → copy binary, install `libssl3` + `wget`
- Final image: ~30-50MB

---

## Testing Strategy

No GPU required for any test layer.

### Unit Tests

**1) SHM protocol parsing**
- Header packing with fixed little-endian byte order
- CRC32 correctness (when enabled)
- Bounds checks (write_len, header size, ring size)
- Backoff state machine transitions (Spinning → Yielding → Sleeping)
- `cargo test` with in-memory buffers / tempfiles

**2) Fanout behavior**
- `broadcast` lag detection (`RecvError::Lagged(n)`)
- Client state transitions: Streaming → AwaitingKeyframe → Streaming
- IDR request coalescing (min interval enforcement)
- WS frame discrimination (first byte type tag)
- `cargo test` with mock broadcast channels

### Integration Tests

**3) Cross-language SHM compatibility (critical)**

The guardrail that proves the Rust SHM reader and Python SHM writer agree on the contract.

- Python test writes frames into a tempfile using `genesis_bridge.shm.ShmWriter`
- Rust binary reads the same file
- Asserts: frames recognized, CRC passes, sequence increments, corrupted frames dropped
- Validates: header layout, endianness, CRC behavior, lap semantics

**4) NATS relay**
- Spin up `nats-server` in test harness
- Publish telemetry messages to various subjects
- Connect WS client, assert received frames match binary envelope format
- Verify subject filtering (allowlist) and oversized message drops

**5) Full pipeline sanity**
- `docker compose --profile sim up`
- WS test client connects, asserts "some frames arrived" within timeout
- Validates whole backplane comes up end-to-end

### Benchmarking

**6) Fanout throughput**
- N mock WS clients, pump frames at target FPS
- Measure: p50/p95/p99 delivery latency, drop rates per client, CPU cost per N clients

### Why the Cross-Language Test Matters

The SHM interface is the fast path, which also means it's the silent failure path. The cross-language integration test proves the Rust reader and Python writer actually agree on the contract. Without it, you're just doing interpretive dance with bytes.

---

## Future (Not Phase 2)

- **Unix DGRAM socket hint** (Phase 3?): Optional notification from genesis-sim when a frame is ready. Optimization for the adaptive backoff loop - hint, not dependency.
- **WebRTC** (Phase 4): DataChannel for low-latency control, media track for H.264 RTP.
- **Telemetry format upgrade**: If all telemetry is confirmed JSON, consider raw JSON relay without binary envelope (but verify UTF-8 safety first).
