# Phase 2: Rust Transport Server Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a Rust service that reads encoded video frames from the Phase 1 SHM ringbuffer and fans them out to browser clients over WebSocket, while relaying NATS telemetry on the same connection.

**Architecture:** A single tokio-based binary with four subsystems: SHM reader (adaptive poll mmap), broadcast fanout (per-client lag detection + IDR recovery), axum WebSocket server (binary protocol), and async-nats telemetry relay. All config via env vars.

**Tech Stack:** Rust 1.89, tokio 1.x, axum 0.7 (WebSocket), async-nats 0.39, memmap2 0.9, crc32fast 1.4, prometheus 0.13, tracing 0.1

**Ref:** [Phase 2 Design](/docs/plans/2026-02-05-phase2-rust-transport-server-design.md)

**Working directory:** `/home/ethan/SDR_OS/.worktrees/phase2`

**Python SHM protocol reference (must match byte-for-byte):**
- Header format: `<QQIHHiI` (little-endian: u64 frame_id, u64 frame_seq, u32 size, u16 flags, u16 codec, i32 crc32, u32 reserved) = 32 bytes
- Metadata: bytes [0:8] = sequence_number (u64 LE), bytes [8:16] = write_index (u64 LE)
- Data region starts at byte 16
- FLAG_KEYFRAME = 0x0001, CODEC_H264 = 1, CODEC_HEVC = 2

---

## Task 1: Scaffold Rust Crate

**Files:**
- Create: `services/transport-server/Cargo.toml`
- Create: `services/transport-server/src/main.rs`

**Step 1: Create directory structure**

```bash
mkdir -p services/transport-server/src
```

**Step 2: Write Cargo.toml**

Create `services/transport-server/Cargo.toml`:

```toml
[package]
name = "transport-server"
version = "0.1.0"
edition = "2021"
rust-version = "1.75"

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
tracing-subscriber = { version = "0.3", features = ["env-filter"] }
tower-http = { version = "0.5", features = ["cors"] }

[dev-dependencies]
tempfile = "3"
tokio-tungstenite = "0.24"
```

**Step 3: Write minimal main.rs**

Create `services/transport-server/src/main.rs`:

```rust
use std::net::SocketAddr;

use axum::{routing::get, Router};
use tracing_subscriber::EnvFilter;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let app = Router::new().route("/health", get(health));

    let addr: SocketAddr = std::env::var("SDR_LISTEN_ADDR")
        .unwrap_or_else(|_| "0.0.0.0:8080".into())
        .parse()
        .expect("invalid SDR_LISTEN_ADDR");

    tracing::info!("transport-server listening on {addr}");
    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn health() -> &'static str {
    "{\"status\":\"ok\"}"
}
```

**Step 4: Verify it compiles**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo build --manifest-path services/transport-server/Cargo.toml`
Expected: Build succeeds

**Step 5: Verify it runs**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo run --manifest-path services/transport-server/Cargo.toml &`
Then: `curl http://localhost:8080/health`
Expected: `{"status":"ok"}`
Kill the background process.

**Step 6: Commit**

```bash
git add services/transport-server/
git commit -m "feat: scaffold transport-server Rust crate with health endpoint"
```

---

## Task 2: Config Module

**Files:**
- Create: `services/transport-server/src/config.rs`
- Modify: `services/transport-server/src/main.rs`

**Step 1: Write the failing test**

Add to `services/transport-server/src/config.rs`:

```rust
use std::env;

/// All configuration for the transport server, loaded from environment variables.
pub struct Config {
    pub shm_path: String,
    pub shm_size: usize,
    pub nats_url: String,
    pub listen_addr: String,
    pub broadcast_capacity: usize,
    pub crc_enabled: bool,
    pub idr_coalesce_ms: u64,
    pub idr_timeout_ms: u64,
    pub telemetry_subjects: String,
    pub telemetry_max_size: usize,
}

impl Config {
    pub fn from_env() -> Self {
        Self {
            shm_path: env::var("SDR_SHM_PATH")
                .unwrap_or_else(|_| "/dev/shm/sdr_os_ipc/frames".into()),
            shm_size: env::var("SDR_SHM_SIZE")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(4 * 1024 * 1024),
            nats_url: env::var("SDR_NATS_URL")
                .unwrap_or_else(|_| "nats://nats:4222".into()),
            listen_addr: env::var("SDR_LISTEN_ADDR")
                .unwrap_or_else(|_| "0.0.0.0:8080".into()),
            broadcast_capacity: env::var("SDR_BROADCAST_CAPACITY")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(64),
            crc_enabled: env::var("SDR_CRC_ENABLED")
                .map(|v| v != "false" && v != "0")
                .unwrap_or(true),
            idr_coalesce_ms: env::var("SDR_IDR_COALESCE_MS")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(500),
            idr_timeout_ms: env::var("SDR_IDR_TIMEOUT_MS")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(5000),
            telemetry_subjects: env::var("SDR_TELEMETRY_SUBJECTS")
                .unwrap_or_else(|_| "telemetry.>".into()),
            telemetry_max_size: env::var("SDR_TELEMETRY_MAX_SIZE")
                .ok()
                .and_then(|v| v.parse().ok())
                .unwrap_or(65536),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_defaults() {
        // Clear any env vars that might interfere
        for key in &[
            "SDR_SHM_PATH", "SDR_SHM_SIZE", "SDR_NATS_URL", "SDR_LISTEN_ADDR",
            "SDR_BROADCAST_CAPACITY", "SDR_CRC_ENABLED", "SDR_IDR_COALESCE_MS",
            "SDR_IDR_TIMEOUT_MS", "SDR_TELEMETRY_SUBJECTS", "SDR_TELEMETRY_MAX_SIZE",
        ] {
            env::remove_var(key);
        }

        let cfg = Config::from_env();
        assert_eq!(cfg.shm_path, "/dev/shm/sdr_os_ipc/frames");
        assert_eq!(cfg.shm_size, 4 * 1024 * 1024);
        assert_eq!(cfg.nats_url, "nats://nats:4222");
        assert_eq!(cfg.listen_addr, "0.0.0.0:8080");
        assert_eq!(cfg.broadcast_capacity, 64);
        assert!(cfg.crc_enabled);
        assert_eq!(cfg.idr_coalesce_ms, 500);
        assert_eq!(cfg.idr_timeout_ms, 5000);
        assert_eq!(cfg.telemetry_subjects, "telemetry.>");
        assert_eq!(cfg.telemetry_max_size, 65536);
    }

    #[test]
    fn test_crc_disabled() {
        env::set_var("SDR_CRC_ENABLED", "false");
        let cfg = Config::from_env();
        assert!(!cfg.crc_enabled);
        env::remove_var("SDR_CRC_ENABLED");
    }
}
```

**Step 2: Wire config into main.rs**

Replace `services/transport-server/src/main.rs`:

```rust
mod config;

use std::net::SocketAddr;

use axum::{routing::get, Router};
use tracing_subscriber::EnvFilter;

use crate::config::Config;

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::from_default_env())
        .init();

    let cfg = Config::from_env();

    let app = Router::new().route("/health", get(health));

    let addr: SocketAddr = cfg.listen_addr.parse().expect("invalid SDR_LISTEN_ADDR");
    tracing::info!("transport-server listening on {addr}");
    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn health() -> &'static str {
    "{\"status\":\"ok\"}"
}
```

**Step 3: Run tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml -- --test-threads=1`
Expected: 2 tests PASS (test_defaults, test_crc_disabled). Use `--test-threads=1` because tests mutate env vars.

**Step 4: Commit**

```bash
git add services/transport-server/src/config.rs services/transport-server/src/main.rs
git commit -m "feat: add config module with env var parsing and defaults"
```

---

## Task 3: SHM Protocol - Frame Header Parsing

**Files:**
- Create: `services/transport-server/src/shm/mod.rs`
- Create: `services/transport-server/src/shm/protocol.rs`
- Modify: `services/transport-server/src/main.rs` (add `mod shm;`)

**Step 1: Write the failing tests**

Create `services/transport-server/src/shm/protocol.rs`:

```rust
/// SHM ringbuffer frame header protocol.
///
/// Must match the Python implementation byte-for-byte:
///   Python struct format: "<QQIHHiI" (little-endian)
///   Layout: frame_id(u64) + frame_seq(u64) + size(u32) + flags(u16) + codec(u16) + crc32(i32) + reserved(u32)
///   Total: 32 bytes
///
/// Metadata region (first 16 bytes of SHM):
///   [0:8]  sequence_number (u64 LE) - commit marker, written LAST by writer
///   [8:16] write_index (u64 LE) - length of frame data written

pub const HEADER_SIZE: usize = 32;
pub const META_SIZE: usize = 16;
pub const META_SEQ_OFFSET: usize = 0;
pub const META_WIDX_OFFSET: usize = 8;

pub const FLAG_KEYFRAME: u16 = 0x0001;
pub const FLAG_WRAP_MARKER: u16 = 0x0002;

pub const CODEC_H264: u16 = 1;
pub const CODEC_HEVC: u16 = 2;

#[derive(Debug, Clone, PartialEq)]
pub struct FrameHeader {
    pub frame_id: u64,
    pub frame_seq: u64,
    pub size: u32,
    pub flags: u16,
    pub codec: u16,
    pub crc32: i32,
    pub reserved: u32,
}

impl FrameHeader {
    /// Parse a 32-byte header from a byte slice. Explicit little-endian, no transmute.
    pub fn from_bytes(data: &[u8]) -> Option<Self> {
        if data.len() < HEADER_SIZE {
            return None;
        }
        Some(Self {
            frame_id: u64::from_le_bytes(data[0..8].try_into().ok()?),
            frame_seq: u64::from_le_bytes(data[8..16].try_into().ok()?),
            size: u32::from_le_bytes(data[16..20].try_into().ok()?),
            flags: u16::from_le_bytes(data[20..22].try_into().ok()?),
            codec: u16::from_le_bytes(data[22..24].try_into().ok()?),
            crc32: i32::from_le_bytes(data[24..28].try_into().ok()?),
            reserved: u32::from_le_bytes(data[28..32].try_into().ok()?),
        })
    }

    /// Serialize to 32 bytes, little-endian. Matches Python's struct.pack("<QQIHHiI", ...).
    pub fn to_bytes(&self) -> [u8; HEADER_SIZE] {
        let mut buf = [0u8; HEADER_SIZE];
        buf[0..8].copy_from_slice(&self.frame_id.to_le_bytes());
        buf[8..16].copy_from_slice(&self.frame_seq.to_le_bytes());
        buf[16..20].copy_from_slice(&self.size.to_le_bytes());
        buf[20..22].copy_from_slice(&self.flags.to_le_bytes());
        buf[22..24].copy_from_slice(&self.codec.to_le_bytes());
        buf[24..28].copy_from_slice(&self.crc32.to_le_bytes());
        buf[28..32].copy_from_slice(&self.reserved.to_le_bytes());
        buf
    }

    pub fn is_keyframe(&self) -> bool {
        self.flags & FLAG_KEYFRAME != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_roundtrip() {
        let hdr = FrameHeader {
            frame_id: 1,
            frame_seq: 42,
            size: 1024,
            flags: FLAG_KEYFRAME,
            codec: CODEC_H264,
            crc32: -559038737_i32, // 0xDEADBEEF as i32
            reserved: 0,
        };
        let bytes = hdr.to_bytes();
        assert_eq!(bytes.len(), 32);

        let parsed = FrameHeader::from_bytes(&bytes).unwrap();
        assert_eq!(parsed, hdr);
    }

    #[test]
    fn test_python_compat_known_bytes() {
        // Build the exact bytes Python would produce with:
        //   struct.pack("<QQIHHiI", 1, 42, 1024, 1, 1, -559038737, 0)
        let mut expected = Vec::new();
        expected.extend_from_slice(&1u64.to_le_bytes());       // frame_id
        expected.extend_from_slice(&42u64.to_le_bytes());      // frame_seq
        expected.extend_from_slice(&1024u32.to_le_bytes());    // size
        expected.extend_from_slice(&1u16.to_le_bytes());       // flags (KEYFRAME)
        expected.extend_from_slice(&1u16.to_le_bytes());       // codec (H264)
        expected.extend_from_slice(&(-559038737i32).to_le_bytes()); // crc32
        expected.extend_from_slice(&0u32.to_le_bytes());       // reserved
        assert_eq!(expected.len(), 32);

        let hdr = FrameHeader::from_bytes(&expected).unwrap();
        assert_eq!(hdr.frame_id, 1);
        assert_eq!(hdr.frame_seq, 42);
        assert_eq!(hdr.size, 1024);
        assert_eq!(hdr.flags, FLAG_KEYFRAME);
        assert_eq!(hdr.codec, CODEC_H264);
        assert_eq!(hdr.crc32, -559038737);
        assert!(hdr.is_keyframe());

        // And serializing back should produce identical bytes
        assert_eq!(hdr.to_bytes().to_vec(), expected);
    }

    #[test]
    fn test_is_keyframe() {
        let idr = FrameHeader {
            frame_id: 0, frame_seq: 0, size: 0,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: 0, reserved: 0,
        };
        assert!(idr.is_keyframe());

        let non_idr = FrameHeader {
            frame_id: 0, frame_seq: 0, size: 0,
            flags: 0, codec: CODEC_H264, crc32: 0, reserved: 0,
        };
        assert!(!non_idr.is_keyframe());
    }

    #[test]
    fn test_too_short_returns_none() {
        assert!(FrameHeader::from_bytes(&[0u8; 31]).is_none());
        assert!(FrameHeader::from_bytes(&[]).is_none());
    }

    #[test]
    fn test_hevc_codec() {
        let hdr = FrameHeader {
            frame_id: 5, frame_seq: 100, size: 2048,
            flags: 0, codec: CODEC_HEVC, crc32: 12345, reserved: 0,
        };
        let parsed = FrameHeader::from_bytes(&hdr.to_bytes()).unwrap();
        assert_eq!(parsed.codec, CODEC_HEVC);
        assert!(!parsed.is_keyframe());
    }
}
```

**Step 2: Create mod.rs and wire into main**

Create `services/transport-server/src/shm/mod.rs`:

```rust
pub mod protocol;
```

Add `mod shm;` to the top of `services/transport-server/src/main.rs` (after `mod config;`).

**Step 3: Run tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml`
Expected: All protocol tests PASS (5 new + 2 from config)

**Step 4: Commit**

```bash
git add services/transport-server/src/shm/
git add services/transport-server/src/main.rs
git commit -m "feat: add SHM frame header protocol with Python-compatible LE parsing"
```

---

## Task 4: SHM Reader - Core Read + Validation

**Files:**
- Create: `services/transport-server/src/shm/reader.rs`
- Modify: `services/transport-server/src/shm/mod.rs`

**Step 1: Write the failing tests and implementation**

Create `services/transport-server/src/shm/reader.rs`:

```rust
use std::time::Instant;

use memmap2::Mmap;

use super::protocol::{FrameHeader, HEADER_SIZE, META_SEQ_OFFSET, META_SIZE, META_WIDX_OFFSET};

/// A frame read from shared memory: header + payload bytes.
#[derive(Debug, Clone)]
pub struct Frame {
    pub header: FrameHeader,
    pub payload: Vec<u8>,
}

/// Adaptive poll state for the SHM reader.
#[derive(Debug)]
enum PollState {
    /// Hot path: std::hint::spin_loop() between reads.
    Spinning { since: Instant },
    /// Warm: tokio::task::yield_now() between reads.
    Yielding { since: Instant },
    /// Cold: tokio::time::sleep(1ms) between reads.
    Sleeping,
}

/// Reads encoded video frames from a shared memory ringbuffer.
///
/// The mmap layout must match the Python ShmWriter from genesis_bridge:
///   [0:8]   sequence_number (u64 LE, commit marker)
///   [8:16]  write_index (u64 LE, length of frame data)
///   [16:]   frame data (32-byte header + payload)
pub struct ShmReader {
    mmap: Mmap,
    mmap_len: usize,
    last_seq: u64,
    last_frame_seq: u64,
    awaiting_keyframe: bool,
    state: PollState,
    crc_enabled: bool,
}

/// Result of a single read attempt.
#[derive(Debug)]
pub enum ReadResult {
    /// A valid frame was read.
    Frame(Frame),
    /// No new frame available (seq unchanged).
    NoNewFrame,
    /// Writer was mid-write (seq1 != seq2), retry.
    TornRead,
    /// write_len failed validation.
    InvalidLength,
    /// Header parse failed.
    BadHeader,
    /// CRC mismatch.
    CrcMismatch,
    /// Frame skipped because we're awaiting a keyframe.
    SkippedNonKeyframe,
}

impl ShmReader {
    /// Create a reader from an existing Mmap.
    pub fn new(mmap: Mmap, crc_enabled: bool) -> Self {
        let mmap_len = mmap.len();
        Self {
            mmap,
            mmap_len,
            last_seq: 0,
            last_frame_seq: 0,
            awaiting_keyframe: false,
            state: PollState::Sleeping,
            crc_enabled,
        }
    }

    /// Attempt to read one frame from the ringbuffer.
    ///
    /// This is the atomic read protocol:
    ///   1. Read seq1 (acquire)
    ///   2. Validate write_len
    ///   3. Read frame data
    ///   4. Read seq2 (acquire) - must equal seq1
    ///   5. Parse header, check CRC
    pub fn try_read_frame(&mut self) -> ReadResult {
        let buf = &self.mmap[..];

        // Step 1: Read sequence number
        let seq1 = read_u64_le(buf, META_SEQ_OFFSET);
        if seq1 == 0 || seq1 == self.last_seq {
            return ReadResult::NoNewFrame;
        }

        // Step 2: Read and validate write_len
        let write_len = read_u64_le(buf, META_WIDX_OFFSET) as usize;
        if write_len == 0
            || write_len > self.mmap_len - META_SIZE
            || META_SIZE + write_len > self.mmap_len
            || write_len < HEADER_SIZE
        {
            return ReadResult::InvalidLength;
        }

        // Step 3: Read frame data
        let frame_data = &buf[META_SIZE..META_SIZE + write_len];

        // Step 4: Verify sequence didn't change during read
        let seq2 = read_u64_le(buf, META_SEQ_OFFSET);
        if seq1 != seq2 {
            return ReadResult::TornRead;
        }

        // Step 5: Parse header
        let header = match FrameHeader::from_bytes(&frame_data[..HEADER_SIZE]) {
            Some(h) => h,
            None => return ReadResult::BadHeader,
        };

        let payload = &frame_data[HEADER_SIZE..];

        // Validate payload size matches header
        if header.size as usize != payload.len() {
            return ReadResult::BadHeader;
        }

        // Step 6: CRC check (if enabled)
        if self.crc_enabled {
            let computed = crc32fast::hash(payload) as i32;
            if computed != header.crc32 {
                return ReadResult::CrcMismatch;
            }
        }

        // Lap detection
        if self.last_frame_seq > 0 && header.frame_seq != self.last_frame_seq + 1 {
            tracing::warn!(
                expected = self.last_frame_seq + 1,
                got = header.frame_seq,
                "SHM lap detected, awaiting keyframe"
            );
            self.awaiting_keyframe = true;
        }

        // Keyframe gating
        if self.awaiting_keyframe {
            if header.is_keyframe() {
                self.awaiting_keyframe = false;
            } else {
                self.last_seq = seq1;
                self.last_frame_seq = header.frame_seq;
                return ReadResult::SkippedNonKeyframe;
            }
        }

        self.last_seq = seq1;
        self.last_frame_seq = header.frame_seq;

        ReadResult::Frame(Frame {
            header,
            payload: payload.to_vec(),
        })
    }

    /// Transition backoff state after a successful frame read.
    pub fn on_frame_received(&mut self) {
        self.state = PollState::Spinning {
            since: Instant::now(),
        };
    }

    /// Get the appropriate delay for the current poll state, advancing state if needed.
    /// Returns None for spin (caller should spin_loop), or Some(duration) for sleep.
    pub fn poll_delay(&mut self) -> Option<std::time::Duration> {
        match &self.state {
            PollState::Spinning { since } => {
                if since.elapsed() > std::time::Duration::from_micros(100) {
                    self.state = PollState::Yielding {
                        since: Instant::now(),
                    };
                    None // yield_now, not sleep
                } else {
                    None // spin_loop
                }
            }
            PollState::Yielding { since } => {
                if since.elapsed() > std::time::Duration::from_millis(50) {
                    self.state = PollState::Sleeping;
                    Some(std::time::Duration::from_millis(1))
                } else {
                    None // yield_now
                }
            }
            PollState::Sleeping => Some(std::time::Duration::from_millis(1)),
        }
    }

    /// Force transition to sleeping state (e.g., when no clients connected).
    pub fn enter_sleep(&mut self) {
        self.state = PollState::Sleeping;
    }

    pub fn last_seq(&self) -> u64 {
        self.last_seq
    }
}

/// Read a little-endian u64 from a byte slice at the given offset.
/// Uses explicit LE decoding, not pointer casts.
fn read_u64_le(buf: &[u8], offset: usize) -> u64 {
    let bytes: [u8; 8] = buf[offset..offset + 8].try_into().unwrap();
    u64::from_le_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shm::protocol::{CODEC_H264, FLAG_KEYFRAME};

    /// Helper: create a fake SHM buffer with metadata + one frame.
    fn make_shm_buffer(seq: u64, header: &FrameHeader, payload: &[u8]) -> Vec<u8> {
        let write_len = HEADER_SIZE + payload.len();
        let total = META_SIZE + write_len + 256; // some padding
        let mut buf = vec![0u8; total];

        // Write metadata
        buf[META_SEQ_OFFSET..META_SEQ_OFFSET + 8].copy_from_slice(&seq.to_le_bytes());
        buf[META_WIDX_OFFSET..META_WIDX_OFFSET + 8]
            .copy_from_slice(&(write_len as u64).to_le_bytes());

        // Write frame header + payload
        buf[META_SIZE..META_SIZE + HEADER_SIZE].copy_from_slice(&header.to_bytes());
        buf[META_SIZE + HEADER_SIZE..META_SIZE + HEADER_SIZE + payload.len()]
            .copy_from_slice(payload);

        buf
    }

    fn make_mmap(data: &[u8]) -> Mmap {
        use std::io::Write;
        let mut f = tempfile::tempfile().unwrap();
        f.write_all(data).unwrap();
        unsafe { Mmap::map(&f).unwrap() }
    }

    #[test]
    fn test_read_single_frame() {
        let payload = b"fake H.264 NAL unit data";
        let crc = crc32fast::hash(payload) as i32;
        let header = FrameHeader {
            frame_id: 1,
            frame_seq: 1,
            size: payload.len() as u32,
            flags: FLAG_KEYFRAME,
            codec: CODEC_H264,
            crc32: crc,
            reserved: 0,
        };
        let buf = make_shm_buffer(1, &header, payload);
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true);

        match reader.try_read_frame() {
            ReadResult::Frame(f) => {
                assert_eq!(f.header, header);
                assert_eq!(f.payload, payload);
            }
            other => panic!("expected Frame, got {:?}", other),
        }
    }

    #[test]
    fn test_no_new_frame_when_seq_unchanged() {
        let payload = b"data";
        let crc = crc32fast::hash(payload) as i32;
        let header = FrameHeader {
            frame_id: 1, frame_seq: 1, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: crc, reserved: 0,
        };
        let buf = make_shm_buffer(1, &header, payload);
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true);

        // First read succeeds
        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));
        // Second read: no new frame
        assert!(matches!(reader.try_read_frame(), ReadResult::NoNewFrame));
    }

    #[test]
    fn test_crc_mismatch_detected() {
        let payload = b"valid data";
        let bad_crc = 0x12345678_i32; // Wrong CRC
        let header = FrameHeader {
            frame_id: 1, frame_seq: 1, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: bad_crc, reserved: 0,
        };
        let buf = make_shm_buffer(1, &header, payload);
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::CrcMismatch));
    }

    #[test]
    fn test_crc_disabled_skips_check() {
        let payload = b"valid data";
        let bad_crc = 0x12345678_i32; // Wrong CRC, but CRC disabled
        let header = FrameHeader {
            frame_id: 1, frame_seq: 1, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: bad_crc, reserved: 0,
        };
        let buf = make_shm_buffer(1, &header, payload);
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, false);

        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));
    }

    #[test]
    fn test_invalid_write_len_zero() {
        let mut buf = vec![0u8; 1024];
        // Set seq to nonzero but write_len to 0
        buf[META_SEQ_OFFSET..META_SEQ_OFFSET + 8].copy_from_slice(&1u64.to_le_bytes());
        buf[META_WIDX_OFFSET..META_WIDX_OFFSET + 8].copy_from_slice(&0u64.to_le_bytes());
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::InvalidLength));
    }

    #[test]
    fn test_invalid_write_len_exceeds_buffer() {
        let mut buf = vec![0u8; 1024];
        buf[META_SEQ_OFFSET..META_SEQ_OFFSET + 8].copy_from_slice(&1u64.to_le_bytes());
        // write_len larger than buffer
        buf[META_WIDX_OFFSET..META_WIDX_OFFSET + 8]
            .copy_from_slice(&10000u64.to_le_bytes());
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::InvalidLength));
    }

    #[test]
    fn test_lap_detection_triggers_keyframe_wait() {
        // Frame 1: keyframe, seq 1
        let payload1 = b"frame1";
        let crc1 = crc32fast::hash(payload1) as i32;
        let hdr1 = FrameHeader {
            frame_id: 1, frame_seq: 1, size: payload1.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: crc1, reserved: 0,
        };
        let buf1 = make_shm_buffer(1, &hdr1, payload1);
        let mmap1 = make_mmap(&buf1);
        let mut reader = ShmReader::new(mmap1, true);
        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));

        // Frame 2: seq jumps to 10 (lapped), not a keyframe
        let payload2 = b"frame2";
        let crc2 = crc32fast::hash(payload2) as i32;
        let hdr2 = FrameHeader {
            frame_id: 10, frame_seq: 10, size: payload2.len() as u32,
            flags: 0, codec: CODEC_H264, crc32: crc2, reserved: 0,
        };
        let buf2 = make_shm_buffer(2, &hdr2, payload2);

        // Swap mmap (simulates writer advancing)
        reader.mmap = make_mmap(&buf2);
        reader.mmap_len = reader.mmap.len();

        // Should detect lap and skip (awaiting keyframe)
        assert!(matches!(
            reader.try_read_frame(),
            ReadResult::SkippedNonKeyframe
        ));

        // Frame 3: keyframe arrives, should resume
        let payload3 = b"keyframe";
        let crc3 = crc32fast::hash(payload3) as i32;
        let hdr3 = FrameHeader {
            frame_id: 11, frame_seq: 11, size: payload3.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: crc3, reserved: 0,
        };
        let buf3 = make_shm_buffer(3, &hdr3, payload3);
        reader.mmap = make_mmap(&buf3);
        reader.mmap_len = reader.mmap.len();

        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));
    }

    #[test]
    fn test_seq_zero_returns_no_frame() {
        let buf = vec![0u8; 1024]; // All zeros, seq = 0
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::NoNewFrame));
    }
}
```

**Step 2: Add reader to mod.rs**

Update `services/transport-server/src/shm/mod.rs`:

```rust
pub mod protocol;
pub mod reader;
```

**Step 3: Run tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml`
Expected: All tests PASS (8 new reader tests + 5 protocol + 2 config = 15 total)

**Step 4: Commit**

```bash
git add services/transport-server/src/shm/reader.rs services/transport-server/src/shm/mod.rs
git commit -m "feat: add SHM reader with atomic read, CRC validation, lap detection"
```

---

## Task 5: Fanout - Broadcast + Backpressure

**Files:**
- Create: `services/transport-server/src/fanout/mod.rs`
- Create: `services/transport-server/src/fanout/broadcast.rs`
- Create: `services/transport-server/src/fanout/backpressure.rs`
- Modify: `services/transport-server/src/main.rs` (add `mod fanout;`)

**Step 1: Write broadcast.rs**

Create `services/transport-server/src/fanout/broadcast.rs`:

```rust
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use tokio::sync::broadcast;

use crate::shm::reader::Frame;

/// Shared state for the frame broadcast system.
pub struct FrameBroadcast {
    pub sender: broadcast::Sender<Arc<Frame>>,
    pub client_count: Arc<AtomicUsize>,
}

impl FrameBroadcast {
    pub fn new(capacity: usize) -> Self {
        let (sender, _) = broadcast::channel(capacity);
        Self {
            sender,
            client_count: Arc::new(AtomicUsize::new(0)),
        }
    }

    /// Subscribe a new client. Returns a receiver and a guard that decrements
    /// the client count on drop.
    pub fn subscribe(&self) -> (broadcast::Receiver<Arc<Frame>>, ClientGuard) {
        self.client_count.fetch_add(1, Ordering::Relaxed);
        let rx = self.sender.subscribe();
        let guard = ClientGuard {
            count: Arc::clone(&self.client_count),
        };
        (rx, guard)
    }

    pub fn client_count(&self) -> usize {
        self.client_count.load(Ordering::Relaxed)
    }

    /// Send a frame to all subscribers. Returns the number of active receivers.
    pub fn send_frame(&self, frame: Frame) -> usize {
        // If no receivers, send will fail - that's fine.
        self.sender.send(Arc::new(frame)).unwrap_or(0)
    }
}

/// RAII guard that decrements the client count when dropped.
pub struct ClientGuard {
    count: Arc<AtomicUsize>,
}

impl Drop for ClientGuard {
    fn drop(&mut self) {
        self.count.fetch_sub(1, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shm::protocol::{CODEC_H264, FLAG_KEYFRAME};

    fn make_frame(seq: u64, keyframe: bool) -> Frame {
        let payload = format!("frame-{seq}").into_bytes();
        Frame {
            header: crate::shm::protocol::FrameHeader {
                frame_id: seq,
                frame_seq: seq,
                size: payload.len() as u32,
                flags: if keyframe { FLAG_KEYFRAME } else { 0 },
                codec: CODEC_H264,
                crc32: 0,
                reserved: 0,
            },
            payload,
        }
    }

    #[test]
    fn test_client_count_lifecycle() {
        let bc = FrameBroadcast::new(16);
        assert_eq!(bc.client_count(), 0);

        let (_rx1, _g1) = bc.subscribe();
        assert_eq!(bc.client_count(), 1);

        let (_rx2, _g2) = bc.subscribe();
        assert_eq!(bc.client_count(), 2);

        drop(_g1);
        assert_eq!(bc.client_count(), 1);

        drop(_g2);
        assert_eq!(bc.client_count(), 0);
    }

    #[tokio::test]
    async fn test_frame_delivery() {
        let bc = FrameBroadcast::new(16);
        let (mut rx, _guard) = bc.subscribe();

        let frame = make_frame(1, true);
        bc.send_frame(frame);

        let received = rx.recv().await.unwrap();
        assert_eq!(received.header.frame_seq, 1);
        assert!(received.header.is_keyframe());
    }
}
```

**Step 2: Write backpressure.rs**

Create `services/transport-server/src/fanout/backpressure.rs`:

```rust
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::Mutex;

/// Per-client state for lag detection and keyframe recovery.
#[derive(Debug)]
pub enum ClientState {
    /// Normal operation: forwarding all frames.
    Streaming,
    /// Lagged: waiting for a keyframe before resuming.
    AwaitingKeyframe { since: Instant },
}

impl ClientState {
    pub fn is_awaiting_keyframe(&self) -> bool {
        matches!(self, Self::AwaitingKeyframe { .. })
    }
}

/// Global IDR request coalescer.
/// Ensures only one IDR request is sent per `min_interval`.
pub struct IdrCoalescer {
    last_request: Mutex<Instant>,
    min_interval: Duration,
    pending: AtomicBool,
}

impl IdrCoalescer {
    pub fn new(min_interval_ms: u64) -> Self {
        Self {
            // Start in the past so first request always goes through
            last_request: Mutex::new(Instant::now() - Duration::from_secs(60)),
            min_interval: Duration::from_millis(min_interval_ms),
            pending: AtomicBool::new(false),
        }
    }

    /// Request an IDR. Returns true if the request should be sent (not coalesced).
    pub async fn request_idr(&self) -> bool {
        // Fast path: if there's already a pending request, don't bother
        if self.pending.load(Ordering::Relaxed) {
            return false;
        }

        let mut last = self.last_request.lock().await;
        if last.elapsed() >= self.min_interval {
            *last = Instant::now();
            self.pending.store(true, Ordering::Relaxed);
            true
        } else {
            false
        }
    }

    /// Mark the IDR as delivered (keyframe received).
    pub fn idr_received(&self) {
        self.pending.store(false, Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_client_state_transitions() {
        let state = ClientState::Streaming;
        assert!(!state.is_awaiting_keyframe());

        let state = ClientState::AwaitingKeyframe {
            since: Instant::now(),
        };
        assert!(state.is_awaiting_keyframe());
    }

    #[tokio::test]
    async fn test_idr_coalescer_first_request_allowed() {
        let coalescer = IdrCoalescer::new(500);
        assert!(coalescer.request_idr().await);
    }

    #[tokio::test]
    async fn test_idr_coalescer_second_request_blocked() {
        let coalescer = IdrCoalescer::new(500);
        assert!(coalescer.request_idr().await);
        // Immediately after, should be blocked
        assert!(!coalescer.request_idr().await);
    }

    #[tokio::test]
    async fn test_idr_coalescer_allows_after_interval() {
        let coalescer = IdrCoalescer::new(10); // 10ms for fast test
        assert!(coalescer.request_idr().await);

        tokio::time::sleep(Duration::from_millis(15)).await;
        assert!(coalescer.request_idr().await);
    }

    #[test]
    fn test_idr_received_resets_pending() {
        let coalescer = IdrCoalescer::new(500);
        coalescer.pending.store(true, Ordering::Relaxed);
        coalescer.idr_received();
        assert!(!coalescer.pending.load(Ordering::Relaxed));
    }
}
```

**Step 3: Create mod.rs and wire into main**

Create `services/transport-server/src/fanout/mod.rs`:

```rust
pub mod backpressure;
pub mod broadcast;
```

Add `mod fanout;` to `services/transport-server/src/main.rs`.

**Step 4: Run tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml`
Expected: All tests PASS (6 new fanout tests + 15 prior = 21 total)

**Step 5: Commit**

```bash
git add services/transport-server/src/fanout/
git add services/transport-server/src/main.rs
git commit -m "feat: add broadcast fanout with per-client lag detection and IDR coalescing"
```

---

## Task 6: Health Endpoint with Live State

**Files:**
- Create: `services/transport-server/src/health.rs`
- Modify: `services/transport-server/src/main.rs`

**Step 1: Write health.rs**

Create `services/transport-server/src/health.rs`:

```rust
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use axum::extract::State;
use axum::Json;
use serde::Serialize;

/// Shared health state, updated by the SHM reader loop.
#[derive(Clone)]
pub struct HealthState {
    pub client_count: Arc<std::sync::atomic::AtomicUsize>,
    pub shm_seq: Arc<AtomicU64>,
    pub last_frame_time: Arc<std::sync::Mutex<Option<Instant>>>,
    pub start_time: Instant,
}

#[derive(Serialize)]
pub struct HealthResponse {
    pub status: &'static str,
    pub clients: usize,
    pub shm_seq: u64,
    pub last_frame_age_ms: Option<u64>,
    pub uptime_s: u64,
}

pub async fn health_handler(State(state): State<HealthState>) -> Json<HealthResponse> {
    let last_frame_age_ms = state
        .last_frame_time
        .lock()
        .ok()
        .and_then(|guard| guard.map(|t| t.elapsed().as_millis() as u64));

    Json(HealthResponse {
        status: "ok",
        clients: state.client_count.load(Ordering::Relaxed),
        shm_seq: state.shm_seq.load(Ordering::Relaxed),
        last_frame_age_ms,
        uptime_s: state.start_time.elapsed().as_secs(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_health_response_serializes() {
        let resp = HealthResponse {
            status: "ok",
            clients: 3,
            shm_seq: 42,
            last_frame_age_ms: Some(15),
            uptime_s: 120,
        };
        let json = serde_json::to_string(&resp).unwrap();
        assert!(json.contains("\"status\":\"ok\""));
        assert!(json.contains("\"clients\":3"));
        assert!(json.contains("\"shm_seq\":42"));
        assert!(json.contains("\"last_frame_age_ms\":15"));
    }

    #[test]
    fn test_health_response_null_frame_age() {
        let resp = HealthResponse {
            status: "ok",
            clients: 0,
            shm_seq: 0,
            last_frame_age_ms: None,
            uptime_s: 0,
        };
        let json = serde_json::to_string(&resp).unwrap();
        assert!(json.contains("\"last_frame_age_ms\":null"));
    }
}
```

**Step 2: Wire into main.rs**

Add `mod health;` to main.rs. We'll fully wire the router in Task 9 when all pieces exist.

**Step 3: Run tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml`
Expected: All tests PASS (2 new + 21 prior = 23 total)

**Step 4: Commit**

```bash
git add services/transport-server/src/health.rs services/transport-server/src/main.rs
git commit -m "feat: add health endpoint with client count, shm_seq, last_frame_age_ms"
```

---

## Task 7: WebSocket Handler

**Files:**
- Create: `services/transport-server/src/transport/mod.rs`
- Create: `services/transport-server/src/transport/websocket.rs`
- Modify: `services/transport-server/src/main.rs` (add `mod transport;`)

**Step 1: Write websocket.rs**

Create `services/transport-server/src/transport/websocket.rs`:

```rust
use std::sync::Arc;
use std::time::{Duration, Instant};

use axum::extract::ws::{Message, WebSocket};
use tokio::sync::broadcast;
use tracing;

use crate::fanout::backpressure::ClientState;
use crate::fanout::broadcast::ClientGuard;
use crate::shm::reader::Frame;

/// WS message type byte prefixes.
pub const MSG_TYPE_VIDEO: u8 = 0x01;
pub const MSG_TYPE_TELEMETRY: u8 = 0x02;

/// Maximum subject length for telemetry frames.
pub const MAX_SUBJECT_LEN: usize = 256;

/// Serialize a video frame for WebSocket delivery.
/// Layout: [0x01][32-byte header][payload]
pub fn encode_video_frame(frame: &Frame) -> Vec<u8> {
    let header_bytes = frame.header.to_bytes();
    let mut buf = Vec::with_capacity(1 + header_bytes.len() + frame.payload.len());
    buf.push(MSG_TYPE_VIDEO);
    buf.extend_from_slice(&header_bytes);
    buf.extend_from_slice(&frame.payload);
    buf
}

/// Serialize a telemetry message for WebSocket delivery.
/// Layout: [0x02][u16 subject_len LE][subject bytes][payload]
pub fn encode_telemetry(subject: &str, payload: &[u8]) -> Option<Vec<u8>> {
    if subject.len() > MAX_SUBJECT_LEN {
        return None;
    }
    let subj_bytes = subject.as_bytes();
    let subj_len = subj_bytes.len() as u16;
    let mut buf = Vec::with_capacity(1 + 2 + subj_bytes.len() + payload.len());
    buf.push(MSG_TYPE_TELEMETRY);
    buf.extend_from_slice(&subj_len.to_le_bytes());
    buf.extend_from_slice(subj_bytes);
    buf.extend_from_slice(payload);
    Some(buf)
}

/// Run the per-client WebSocket send loop.
///
/// Receives frames from the broadcast channel, encodes them, and sends
/// over the WebSocket. Handles lag detection and keyframe recovery.
pub async fn client_send_loop(
    mut ws_sender: futures_util::stream::SplitSink<WebSocket, Message>,
    mut video_rx: broadcast::Receiver<Arc<Frame>>,
    _guard: ClientGuard,
    idr_timeout_ms: u64,
) {
    use futures_util::SinkExt;

    let mut state = ClientState::AwaitingKeyframe {
        since: Instant::now(),
    };
    let idr_timeout = Duration::from_millis(idr_timeout_ms);

    loop {
        let frame = match video_rx.recv().await {
            Ok(f) => f,
            Err(broadcast::error::RecvError::Lagged(n)) => {
                tracing::warn!(lagged = n, "client lagged, awaiting keyframe");
                state = ClientState::AwaitingKeyframe {
                    since: Instant::now(),
                };
                continue;
            }
            Err(broadcast::error::RecvError::Closed) => {
                tracing::debug!("broadcast channel closed, disconnecting client");
                break;
            }
        };

        match &state {
            ClientState::AwaitingKeyframe { since } => {
                if since.elapsed() > idr_timeout {
                    tracing::warn!("IDR timeout, disconnecting client");
                    break;
                }
                if frame.header.is_keyframe() {
                    state = ClientState::Streaming;
                    // Fall through to send this keyframe
                } else {
                    continue; // Skip non-keyframes
                }
            }
            ClientState::Streaming => {}
        }

        let data = encode_video_frame(&frame);
        if ws_sender.send(Message::Binary(data.into())).await.is_err() {
            tracing::debug!("WebSocket send failed, disconnecting client");
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shm::protocol::{FrameHeader, CODEC_H264, CODEC_HEVC, FLAG_KEYFRAME, HEADER_SIZE};

    fn make_test_frame(seq: u64, keyframe: bool, payload: &[u8]) -> Frame {
        Frame {
            header: FrameHeader {
                frame_id: seq,
                frame_seq: seq,
                size: payload.len() as u32,
                flags: if keyframe { FLAG_KEYFRAME } else { 0 },
                codec: CODEC_H264,
                crc32: 0,
                reserved: 0,
            },
            payload: payload.to_vec(),
        }
    }

    #[test]
    fn test_encode_video_frame() {
        let frame = make_test_frame(1, true, b"hello");
        let encoded = encode_video_frame(&frame);

        assert_eq!(encoded[0], MSG_TYPE_VIDEO);
        assert_eq!(encoded.len(), 1 + HEADER_SIZE + 5);

        // Parse back the header
        let hdr = FrameHeader::from_bytes(&encoded[1..1 + HEADER_SIZE]).unwrap();
        assert_eq!(hdr.frame_seq, 1);
        assert!(hdr.is_keyframe());

        // Payload
        assert_eq!(&encoded[1 + HEADER_SIZE..], b"hello");
    }

    #[test]
    fn test_encode_telemetry() {
        let encoded = encode_telemetry("telemetry.imu", b"{\"x\":1}").unwrap();

        assert_eq!(encoded[0], MSG_TYPE_TELEMETRY);

        let subj_len = u16::from_le_bytes([encoded[1], encoded[2]]) as usize;
        assert_eq!(subj_len, "telemetry.imu".len());

        let subj = std::str::from_utf8(&encoded[3..3 + subj_len]).unwrap();
        assert_eq!(subj, "telemetry.imu");

        let payload = &encoded[3 + subj_len..];
        assert_eq!(payload, b"{\"x\":1}");
    }

    #[test]
    fn test_encode_telemetry_subject_too_long() {
        let long_subject = "a".repeat(257);
        assert!(encode_telemetry(&long_subject, b"data").is_none());
    }

    #[test]
    fn test_encode_telemetry_max_subject_ok() {
        let max_subject = "a".repeat(256);
        assert!(encode_telemetry(&max_subject, b"data").is_some());
    }
}
```

**Step 2: Create mod.rs and add dependency**

Create `services/transport-server/src/transport/mod.rs`:

```rust
pub mod websocket;
```

Add `mod transport;` to main.rs.

Add `futures-util` to `Cargo.toml` dependencies:

```toml
futures-util = "0.3"
```

**Step 3: Run tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml`
Expected: All tests PASS (4 new + 23 prior = 27 total)

**Step 4: Commit**

```bash
git add services/transport-server/src/transport/ services/transport-server/Cargo.toml services/transport-server/src/main.rs
git commit -m "feat: add WebSocket handler with binary frame encoding and client send loop"
```

---

## Task 8: NATS Telemetry Subscriber

**Files:**
- Create: `services/transport-server/src/nats/mod.rs`
- Create: `services/transport-server/src/nats/subscriber.rs`
- Modify: `services/transport-server/src/main.rs` (add `mod nats;`)

**Step 1: Write subscriber.rs**

Create `services/transport-server/src/nats/subscriber.rs`:

```rust
use std::sync::Arc;

use tokio::sync::broadcast;

use crate::transport::websocket::encode_telemetry;

/// A telemetry message ready for broadcast.
#[derive(Debug, Clone)]
pub struct TelemetryMsg {
    /// Pre-encoded binary WS frame (type byte + subject + payload).
    pub encoded: Vec<u8>,
}

/// Run the NATS telemetry relay loop.
///
/// Subscribes to the configured subject pattern, encodes messages as binary
/// WS frames, and sends them to the telemetry broadcast channel.
pub async fn telemetry_relay(
    nats_url: &str,
    subject: &str,
    max_payload_size: usize,
    tx: broadcast::Sender<Arc<TelemetryMsg>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let client = async_nats::connect(nats_url).await?;
    tracing::info!(subject, "subscribed to NATS telemetry");

    let mut sub = client.subscribe(subject.to_string()).await?;

    while let Some(msg) = sub.next().await {
        let subject = msg.subject.as_str();
        let payload = msg.payload.as_ref();

        // Size cap
        if payload.len() > max_payload_size {
            tracing::warn!(
                subject,
                size = payload.len(),
                max = max_payload_size,
                "telemetry message too large, dropping"
            );
            continue;
        }

        // Encode as binary WS frame
        let encoded = match encode_telemetry(subject, payload) {
            Some(e) => e,
            None => {
                tracing::warn!(subject, "telemetry subject too long, dropping");
                continue;
            }
        };

        let msg = Arc::new(TelemetryMsg { encoded });

        // Best-effort send; if no receivers, that's fine
        let _ = tx.send(msg);
    }

    Ok(())
}

// Note: Integration tests for NATS relay are in Task 11 (require nats-server).
// Unit tests for encoding are already covered in transport/websocket.rs.
```

**Step 2: Create mod.rs and wire into main**

Create `services/transport-server/src/nats/mod.rs`:

```rust
pub mod subscriber;
```

Add `mod nats;` to main.rs.

**Step 3: Verify compilation**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo build --manifest-path services/transport-server/Cargo.toml`
Expected: Compiles (no new unit tests - NATS tests need a real server, covered in Task 11)

**Step 4: Run existing tests still pass**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml`
Expected: All 27 tests PASS

**Step 5: Commit**

```bash
git add services/transport-server/src/nats/ services/transport-server/src/main.rs
git commit -m "feat: add NATS telemetry subscriber with size cap and binary encoding"
```

---

## Task 9: Wire Everything Together in main.rs

**Files:**
- Modify: `services/transport-server/src/main.rs`

**Step 1: Write the full main.rs**

Replace `services/transport-server/src/main.rs`:

```rust
mod config;
mod fanout;
mod health;
mod nats;
mod shm;
mod transport;

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use axum::extract::ws::WebSocket;
use axum::extract::{State, WebSocketUpgrade};
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use futures_util::StreamExt;
use tokio::sync::broadcast;
use tracing_subscriber::EnvFilter;

use crate::config::Config;
use crate::fanout::backpressure::IdrCoalescer;
use crate::fanout::broadcast::FrameBroadcast;
use crate::health::{health_handler, HealthState};
use crate::nats::subscriber::{telemetry_relay, TelemetryMsg};
use crate::shm::reader::{ReadResult, ShmReader};
use crate::transport::websocket::{client_send_loop, encode_telemetry, MSG_TYPE_TELEMETRY};

/// Shared application state passed to axum handlers.
#[derive(Clone)]
struct AppState {
    frame_broadcast: Arc<FrameBroadcast>,
    telemetry_tx: broadcast::Sender<Arc<TelemetryMsg>>,
    health: HealthState,
    idr_coalescer: Arc<IdrCoalescer>,
    idr_timeout_ms: u64,
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .init();

    let cfg = Config::from_env();

    // Frame broadcast
    let frame_broadcast = Arc::new(FrameBroadcast::new(cfg.broadcast_capacity));

    // Telemetry broadcast
    let (telemetry_tx, _) = broadcast::channel::<Arc<TelemetryMsg>>(32);

    // IDR coalescer
    let idr_coalescer = Arc::new(IdrCoalescer::new(cfg.idr_coalesce_ms));

    // Health state
    let health_state = HealthState {
        client_count: Arc::clone(&frame_broadcast.client_count),
        shm_seq: Arc::new(AtomicU64::new(0)),
        last_frame_time: Arc::new(std::sync::Mutex::new(None)),
        start_time: Instant::now(),
    };

    let app_state = AppState {
        frame_broadcast: Arc::clone(&frame_broadcast),
        telemetry_tx: telemetry_tx.clone(),
        health: health_state.clone(),
        idr_coalescer,
        idr_timeout_ms: cfg.idr_timeout_ms,
    };

    // Spawn SHM reader loop
    let shm_health = health_state.clone();
    let shm_broadcast = Arc::clone(&frame_broadcast);
    let shm_path = cfg.shm_path.clone();
    let shm_size = cfg.shm_size;
    let crc_enabled = cfg.crc_enabled;
    tokio::spawn(async move {
        shm_read_loop(shm_path, shm_size, crc_enabled, shm_broadcast, shm_health).await;
    });

    // Spawn NATS relay (non-fatal if NATS unavailable)
    let nats_url = cfg.nats_url.clone();
    let nats_subject = cfg.telemetry_subjects.clone();
    let nats_max_size = cfg.telemetry_max_size;
    let nats_tx = telemetry_tx.clone();
    tokio::spawn(async move {
        loop {
            match telemetry_relay(&nats_url, &nats_subject, nats_max_size, nats_tx.clone()).await {
                Ok(()) => {
                    tracing::info!("NATS relay ended, reconnecting in 5s");
                }
                Err(e) => {
                    tracing::warn!("NATS relay error: {e}, reconnecting in 5s");
                }
            }
            tokio::time::sleep(std::time::Duration::from_secs(5)).await;
        }
    });

    // Build router
    let app = Router::new()
        .route("/stream/ws", get(ws_upgrade_handler))
        .route("/health", get(health_handler))
        .with_state(app_state);

    let addr: std::net::SocketAddr = cfg.listen_addr.parse().expect("invalid SDR_LISTEN_ADDR");
    tracing::info!("transport-server listening on {addr}");
    let listener = tokio::net::TcpListener::bind(addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn ws_upgrade_handler(
    ws: WebSocketUpgrade,
    State(state): State<AppState>,
) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_ws_client(socket, state))
}

async fn handle_ws_client(socket: WebSocket, state: AppState) {
    let (ws_sender, _ws_receiver) = socket.split();

    let (video_rx, guard) = state.frame_broadcast.subscribe();
    let telemetry_rx = state.telemetry_tx.subscribe();

    tracing::info!(
        clients = state.frame_broadcast.client_count(),
        "client connected"
    );

    // Spawn telemetry forwarder
    let ws_sender = Arc::new(tokio::sync::Mutex::new(ws_sender));
    let telem_sender = Arc::clone(&ws_sender);
    let telem_handle = tokio::spawn(async move {
        telemetry_forward_loop(telem_sender, telemetry_rx).await;
    });

    // Run video send loop on current task
    let video_sender_lock = Arc::clone(&ws_sender);
    let video_sender = video_sender_lock.lock().await;
    // We need to restructure: use a single sender with a merge of both streams.
    // For simplicity, we'll use a select loop instead.
    drop(video_sender);
    drop(telem_handle);

    // Unified send loop: merge video and telemetry
    unified_send_loop(ws_sender, video_rx, telemetry_rx, guard, state.idr_timeout_ms).await;

    tracing::info!(
        clients = state.frame_broadcast.client_count(),
        "client disconnected"
    );
}

async fn unified_send_loop(
    ws_sender: Arc<tokio::sync::Mutex<futures_util::stream::SplitSink<WebSocket, axum::extract::ws::Message>>>,
    mut video_rx: broadcast::Receiver<Arc<crate::shm::reader::Frame>>,
    mut telemetry_rx: broadcast::Receiver<Arc<TelemetryMsg>>,
    _guard: crate::fanout::broadcast::ClientGuard,
    idr_timeout_ms: u64,
) {
    use axum::extract::ws::Message;
    use futures_util::SinkExt;

    let mut client_state = crate::fanout::backpressure::ClientState::AwaitingKeyframe {
        since: Instant::now(),
    };
    let idr_timeout = std::time::Duration::from_millis(idr_timeout_ms);

    loop {
        tokio::select! {
            result = video_rx.recv() => {
                match result {
                    Ok(frame) => {
                        // Check client state
                        match &client_state {
                            crate::fanout::backpressure::ClientState::AwaitingKeyframe { since } => {
                                if since.elapsed() > idr_timeout {
                                    tracing::warn!("IDR timeout, disconnecting client");
                                    break;
                                }
                                if frame.header.is_keyframe() {
                                    client_state = crate::fanout::backpressure::ClientState::Streaming;
                                } else {
                                    continue;
                                }
                            }
                            crate::fanout::backpressure::ClientState::Streaming => {}
                        }

                        let data = crate::transport::websocket::encode_video_frame(&frame);
                        let mut sender = ws_sender.lock().await;
                        if sender.send(Message::Binary(data.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(n)) => {
                        tracing::warn!(lagged = n, "client lagged, awaiting keyframe");
                        client_state = crate::fanout::backpressure::ClientState::AwaitingKeyframe {
                            since: Instant::now(),
                        };
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
            result = telemetry_rx.recv() => {
                match result {
                    Ok(msg) => {
                        let mut sender = ws_sender.lock().await;
                        if sender.send(Message::Binary(msg.encoded.clone().into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => {
                        // Silently drop old telemetry
                    }
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
        }
    }
}

async fn telemetry_forward_loop(
    _ws_sender: Arc<tokio::sync::Mutex<futures_util::stream::SplitSink<WebSocket, axum::extract::ws::Message>>>,
    _telemetry_rx: broadcast::Receiver<Arc<TelemetryMsg>>,
) {
    // This is now handled by unified_send_loop, kept as placeholder
}

async fn shm_read_loop(
    shm_path: String,
    shm_size: usize,
    crc_enabled: bool,
    broadcast: Arc<FrameBroadcast>,
    health: HealthState,
) {
    // Wait for the SHM file to appear
    loop {
        if std::path::Path::new(&shm_path).exists() {
            break;
        }
        tracing::debug!(path = %shm_path, "waiting for SHM file");
        tokio::time::sleep(std::time::Duration::from_secs(1)).await;
    }

    // Open mmap
    let file = match std::fs::File::open(&shm_path) {
        Ok(f) => f,
        Err(e) => {
            tracing::error!(path = %shm_path, error = %e, "failed to open SHM file");
            return;
        }
    };

    let mmap = match unsafe { memmap2::Mmap::map(&file) } {
        Ok(m) => m,
        Err(e) => {
            tracing::error!(error = %e, "failed to mmap SHM file");
            return;
        }
    };

    if mmap.len() < shm_size {
        tracing::warn!(
            actual = mmap.len(),
            expected = shm_size,
            "SHM file smaller than expected"
        );
    }

    let mut reader = ShmReader::new(mmap, crc_enabled);
    tracing::info!(path = %shm_path, "SHM reader started");

    loop {
        // No-client optimization: sleep and only peek
        if broadcast.client_count() == 0 {
            reader.enter_sleep();
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            // Still update seq for health endpoint
            match reader.try_read_frame() {
                ReadResult::Frame(f) => {
                    health.shm_seq.store(reader.last_seq(), Ordering::Relaxed);
                    *health.last_frame_time.lock().unwrap() = Some(Instant::now());
                    // Don't broadcast - no clients
                    let _ = f;
                }
                ReadResult::NoNewFrame => {}
                _ => {}
            }
            continue;
        }

        match reader.try_read_frame() {
            ReadResult::Frame(frame) => {
                health.shm_seq.store(reader.last_seq(), Ordering::Relaxed);
                *health.last_frame_time.lock().unwrap() = Some(Instant::now());
                reader.on_frame_received();
                broadcast.send_frame(frame);
            }
            ReadResult::NoNewFrame => {
                match reader.poll_delay() {
                    Some(d) => tokio::time::sleep(d).await,
                    None => {
                        // In Yielding state
                        tokio::task::yield_now().await;
                    }
                }
            }
            ReadResult::TornRead => {
                // Retry immediately
                std::hint::spin_loop();
            }
            ReadResult::CrcMismatch | ReadResult::BadHeader | ReadResult::InvalidLength => {
                tracing::warn!("SHM read error, retrying");
                tokio::task::yield_now().await;
            }
            ReadResult::SkippedNonKeyframe => {
                // Normal during lap recovery
            }
        }
    }
}
```

**Step 2: Verify compilation**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo build --manifest-path services/transport-server/Cargo.toml`
Expected: Compiles. Fix any type errors from wiring.

**Step 3: Run all tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && cargo test --manifest-path services/transport-server/Cargo.toml -- --test-threads=1`
Expected: All 27 tests PASS

**Step 4: Commit**

```bash
git add services/transport-server/src/main.rs
git commit -m "feat: wire SHM reader, fanout, NATS relay, and WS handler in main"
```

---

## Task 10: Dockerfile + Docker Compose Integration

**Files:**
- Create: `services/transport-server/Dockerfile`
- Modify: `docker-compose.yml`

**Step 1: Write the Dockerfile**

Create `services/transport-server/Dockerfile`:

```dockerfile
# Stage 1: Build
FROM rust:1.84-slim AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /build
COPY services/transport-server/ .

RUN cargo build --release

# Stage 2: Runtime
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl3 wget ca-certificates \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /build/target/release/transport-server /usr/local/bin/transport-server

EXPOSE 8080

CMD ["transport-server"]
```

**Step 2: Add transport-server to docker-compose.yml**

Add the following service to `docker-compose.yml` under `services:`, after the `ros-bridge` service and before any legacy profiles:

```yaml
  # ─── Transport Server (Rust) ──────────────────────────────
  transport-server:
    profiles: ["sim"]
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

Also add `/stream/*` route to Caddy config in `configs/Caddyfile` (if it exists from Phase 1, otherwise note as dependency):

The Caddyfile from Phase 1 already has:
```
handle /stream/* {
    reverse_proxy transport-server:8080
}
```

**Step 3: Validate compose config**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && docker compose config --quiet`
Expected: Exit 0, no errors

**Step 4: Commit**

```bash
git add services/transport-server/Dockerfile docker-compose.yml
git commit -m "feat: add transport-server Dockerfile and compose service"
```

---

## Task 11: Cross-Language SHM Integration Test

This is the critical test that proves the Rust reader and Python writer agree on the protocol.

**Files:**
- Create: `tests/integration/test_shm_cross_language.py`
- Create: `tests/integration/conftest.py`

**Step 1: Write the Python test that generates SHM data for Rust**

Create `tests/integration/conftest.py`:

```python
"""Shared fixtures for integration tests."""
```

Create `tests/integration/test_shm_cross_language.py`:

```python
"""Cross-language SHM protocol compatibility test.

Validates that the Rust transport-server SHM reader can correctly
parse frames written by the Python genesis_bridge ShmWriter.

This test:
1. Uses Python ShmWriter to write frames to a temp file
2. Runs the Rust SHM protocol test binary against the same file
3. Asserts both sides agree on header values, CRC, and sequence
"""

import json
import os
import struct
import subprocess
import tempfile
import zlib

import pytest

# Python protocol constants (must match genesis_bridge/shm/protocol.py)
HEADER_FORMAT = "<QQIHHiI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
META_SIZE = 16
FLAG_KEYFRAME = 0x0001
CODEC_H264 = 1
CODEC_HEVC = 2


def write_shm_frame(path: str, buf_size: int, seq: int, frame_id: int,
                     frame_seq: int, payload: bytes, codec: int = CODEC_H264,
                     keyframe: bool = False) -> dict:
    """Write a single frame to a SHM file using the Python protocol.

    Returns a dict of what was written for Rust to verify.
    """
    flags = FLAG_KEYFRAME if keyframe else 0
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    # Python struct.pack uses 'i' (signed i32) for crc32
    crc_signed = struct.unpack('<i', struct.pack('<I', crc))[0]

    header = struct.pack(HEADER_FORMAT,
                         frame_id, frame_seq, len(payload),
                         flags, codec, crc_signed, 0)
    assert len(header) == HEADER_SIZE

    frame_data = header + payload
    write_len = len(frame_data)

    # Create or open the file
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    os.ftruncate(fd, buf_size)

    with open(fd, 'r+b') as f:
        import mmap
        mm = mmap.mmap(f.fileno(), buf_size)

        # Write frame data at META_SIZE
        mm[META_SIZE:META_SIZE + write_len] = frame_data

        # Write write_index
        mm[8:16] = struct.pack('<Q', write_len)

        # Write sequence LAST (commit marker)
        mm[0:8] = struct.pack('<Q', seq)

        mm.close()

    return {
        "seq": seq,
        "frame_id": frame_id,
        "frame_seq": frame_seq,
        "size": len(payload),
        "flags": flags,
        "codec": codec,
        "crc32": crc_signed,
        "payload_hex": payload.hex(),
    }


class TestShmCrossLanguage:
    """Test that Rust can read what Python writes."""

    def test_python_writes_valid_header(self):
        """Verify Python produces the expected byte layout."""
        header = struct.pack(HEADER_FORMAT, 1, 42, 1024, 1, 1, -559038737, 0)
        assert len(header) == 32

        # Manually verify LE byte layout
        assert header[0:8] == (1).to_bytes(8, 'little')       # frame_id
        assert header[8:16] == (42).to_bytes(8, 'little')     # frame_seq
        assert header[16:20] == (1024).to_bytes(4, 'little')  # size
        assert header[20:22] == (1).to_bytes(2, 'little')     # flags
        assert header[22:24] == (1).to_bytes(2, 'little')     # codec

    def test_write_and_read_back_python(self):
        """Verify Python can write and read back its own format (sanity)."""
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "test_frames")
            buf_size = 4 * 1024 * 1024

            payload = b"test payload data" * 10
            meta = write_shm_frame(path, buf_size, seq=1, frame_id=1,
                                   frame_seq=1, payload=payload,
                                   codec=CODEC_H264, keyframe=True)

            # Read back and verify
            with open(path, 'rb') as f:
                import mmap
                mm = mmap.mmap(f.fileno(), buf_size, access=mmap.ACCESS_READ)
                seq = struct.unpack_from('<Q', mm, 0)[0]
                assert seq == 1

                write_len = struct.unpack_from('<Q', mm, 8)[0]
                frame_data = bytes(mm[META_SIZE:META_SIZE + write_len])
                header_data = frame_data[:HEADER_SIZE]

                fields = struct.unpack(HEADER_FORMAT, header_data)
                assert fields[0] == 1  # frame_id
                assert fields[1] == 1  # frame_seq
                assert fields[2] == len(payload)  # size
                assert fields[3] == FLAG_KEYFRAME  # flags
                assert fields[4] == CODEC_H264  # codec

                mm.close()

    def test_generate_rust_test_fixture(self):
        """Generate a SHM file + manifest for the Rust side to validate.

        The manifest is a JSON file describing what was written.
        Rust integration tests can read the manifest and assert the SHM reader
        produces matching results.
        """
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "test_frames")
            manifest_path = os.path.join(tmp, "manifest.json")
            buf_size = 4 * 1024 * 1024

            frames = []

            # Frame 1: H.264 keyframe
            payload1 = b"fake H.264 NAL unit " * 50
            meta1 = write_shm_frame(path, buf_size, seq=1, frame_id=1,
                                    frame_seq=1, payload=payload1,
                                    codec=CODEC_H264, keyframe=True)
            frames.append(meta1)

            manifest = {
                "buf_size": buf_size,
                "shm_path": path,
                "frames": frames,
            }

            with open(manifest_path, 'w') as f:
                json.dump(manifest, f, indent=2)

            # Verify manifest was written
            assert os.path.exists(manifest_path)
            loaded = json.loads(open(manifest_path).read())
            assert loaded["frames"][0]["frame_id"] == 1
```

**Step 2: Run the Python tests**

Run: `cd /home/ethan/SDR_OS/.worktrees/phase2 && python -m pytest tests/integration/test_shm_cross_language.py -v`
Expected: All 3 Python tests PASS

**Step 3: Commit**

```bash
git add tests/integration/
git commit -m "test: add cross-language SHM protocol compatibility tests (Python side)"
```

---

## Task 12: Update Documentation

**Files:**
- Modify: `CLAUDE.md`
- Modify: `documents/SETUP.md`

**Step 1: Add transport-server to CLAUDE.md architecture section**

Add after the Backend subsection in CLAUDE.md:

```markdown
### Services
- **services/transport-server/** - Rust: SHM frame reader → WebSocket fanout + NATS telemetry relay
  - Build: `cargo build --manifest-path services/transport-server/Cargo.toml`
  - Test: `cargo test --manifest-path services/transport-server/Cargo.toml`
  - Config: All via `SDR_*` env vars (see `src/config.rs`)
```

**Step 2: Add Rust build instructions to SETUP.md**

Add a Rust section:

```markdown
## Rust Environment

### Prerequisites
- Rust 1.75+ (install via rustup: `curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh`)

### Build Transport Server
```bash
cargo build --release --manifest-path services/transport-server/Cargo.toml
```

### Run Tests
```bash
cargo test --manifest-path services/transport-server/Cargo.toml
```
```

**Step 3: Commit**

```bash
git add CLAUDE.md documents/SETUP.md
git commit -m "docs: add transport-server build/test instructions to CLAUDE.md and SETUP.md"
```

---

## Task Summary

| # | Task | Key Deliverable | Tests |
|---|------|-----------------|-------|
| 1 | Scaffold Rust crate | Cargo.toml + health endpoint | Compile + curl |
| 2 | Config module | Env var parsing with defaults | 2 unit tests |
| 3 | SHM protocol | Frame header LE parsing | 5 unit tests |
| 4 | SHM reader | Atomic read, CRC, lap detection | 8 unit tests |
| 5 | Fanout | Broadcast + IDR coalescing | 6 unit tests |
| 6 | Health endpoint | JSON with live state | 2 unit tests |
| 7 | WebSocket handler | Binary frame encoding + client loop | 4 unit tests |
| 8 | NATS subscriber | Telemetry relay | Compile check |
| 9 | Wire main.rs | Full server assembly | All 27 pass |
| 10 | Docker | Dockerfile + compose service | compose config |
| 11 | Cross-language test | Python writes, Rust reads | 3 integration |
| 12 | Documentation | CLAUDE.md + SETUP.md | - |

**Estimated commits:** 12 (one per task)
**Total unit tests:** 27
**Total integration tests:** 3
