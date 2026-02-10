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
    keyframe_gating: bool,
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
    ///
    /// When `keyframe_gating` is false, sequence gaps do not trigger keyframe
    /// waiting — all frames pass through. This is appropriate when the encoder
    /// uses intra refresh (the decoder self-heals within one refresh period).
    pub fn new(mmap: Mmap, crc_enabled: bool, keyframe_gating: bool) -> Self {
        let mmap_len = mmap.len();
        Self {
            mmap,
            mmap_len,
            last_seq: 0,
            last_frame_seq: 0,
            awaiting_keyframe: false,
            state: PollState::Sleeping,
            crc_enabled,
            keyframe_gating,
        }
    }

    /// Attempt to read one frame from the ringbuffer.
    ///
    /// Atomic read protocol:
    ///   1. Read seq1 (acquire semantics via volatile/LE read)
    ///   2. Validate write_len
    ///   3. Read frame data
    ///   4. Read seq2 - must equal seq1
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
            || write_len < HEADER_SIZE
            || write_len > self.mmap_len.saturating_sub(META_SIZE)
            || META_SIZE.checked_add(write_len).map_or(true, |end| end > self.mmap_len)
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

        // Step 5: Parse header (explicit LE, no transmute)
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

        // Single-slot "latest wins" SHM means the reader can legitimately observe gaps in
        // frame_seq (it may miss intermediate frames). Treat forward gaps as dropped frames,
        // not an error. Only treat non-monotonic (<=) as a reset/corruption signal.
        if self.keyframe_gating && self.last_frame_seq > 0 {
            if header.frame_seq <= self.last_frame_seq {
                tracing::warn!(
                    last = self.last_frame_seq,
                    got = header.frame_seq,
                    "SHM non-monotonic frame_seq, awaiting keyframe"
                );
                self.awaiting_keyframe = true;
            } else if header.frame_seq > self.last_frame_seq + 1 {
                // We dropped one or more frames; wait for an IDR to guarantee decoder resync.
                self.awaiting_keyframe = true;
            }
        }

        // Keyframe gating: drop non-IDR frames while awaiting recovery
        if self.keyframe_gating && self.awaiting_keyframe {
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
    /// Returns None for spin/yield (caller decides), or Some(duration) for sleep.
    pub fn poll_delay(&mut self) -> Option<std::time::Duration> {
        match &self.state {
            PollState::Spinning { since } => {
                if since.elapsed() > std::time::Duration::from_micros(100) {
                    self.state = PollState::Yielding {
                        since: Instant::now(),
                    };
                }
                None // spin_loop or yield_now
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
        let total = META_SIZE + write_len + 256; // padding
        let mut buf = vec![0u8; total];

        // Metadata
        buf[META_SEQ_OFFSET..META_SEQ_OFFSET + 8].copy_from_slice(&seq.to_le_bytes());
        buf[META_WIDX_OFFSET..META_WIDX_OFFSET + 8]
            .copy_from_slice(&(write_len as u64).to_le_bytes());

        // Frame header + payload
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
            frame_id: 1, frame_seq: 1, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: crc, reserved: 0,
        };
        let buf = make_shm_buffer(1, &header, payload);
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true, true);

        match reader.try_read_frame() {
            ReadResult::Frame(f) => {
                assert_eq!(f.header, header);
                assert_eq!(f.payload, payload);
            }
            other => panic!("expected Frame, got {:?}", other),
        }
    }

    #[test]
    fn test_forward_gap_is_not_fatal_but_requires_keyframe() {
        // seq=1 frame_seq=1 keyframe -> accepted
        // seq=2 frame_seq=3 keyframe -> forward gap (dropped frame) should still be accepted
        let payload = b"fake frame";
        let crc = crc32fast::hash(payload) as i32;

        let header1 = FrameHeader {
            frame_id: 1, frame_seq: 1, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: crc, reserved: 0,
        };
        let buf1 = make_shm_buffer(1, &header1, payload);
        let mmap1 = make_mmap(&buf1);
        let mut reader = ShmReader::new(mmap1, true, true);
        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));

        let header2 = FrameHeader {
            frame_id: 2, frame_seq: 3, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: crc, reserved: 0,
        };
        let buf2 = make_shm_buffer(2, &header2, payload);
        // Swap the mmap underneath by constructing a fresh reader; simulate a jump.
        let mmap2 = make_mmap(&buf2);
        let mut reader2 = ShmReader::new(mmap2, true, true);
        reader2.last_frame_seq = reader.last_frame_seq;

        assert!(matches!(reader2.try_read_frame(), ReadResult::Frame(_)));
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
        let mut reader = ShmReader::new(mmap, true, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));
        assert!(matches!(reader.try_read_frame(), ReadResult::NoNewFrame));
    }

    #[test]
    fn test_crc_mismatch_detected() {
        let payload = b"valid data";
        let bad_crc = 0x12345678_i32;
        let header = FrameHeader {
            frame_id: 1, frame_seq: 1, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: bad_crc, reserved: 0,
        };
        let buf = make_shm_buffer(1, &header, payload);
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::CrcMismatch));
    }

    #[test]
    fn test_crc_disabled_skips_check() {
        let payload = b"valid data";
        let bad_crc = 0x12345678_i32;
        let header = FrameHeader {
            frame_id: 1, frame_seq: 1, size: payload.len() as u32,
            flags: FLAG_KEYFRAME, codec: CODEC_H264, crc32: bad_crc, reserved: 0,
        };
        let buf = make_shm_buffer(1, &header, payload);
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, false, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));
    }

    #[test]
    fn test_invalid_write_len_zero() {
        let mut buf = vec![0u8; 1024];
        buf[META_SEQ_OFFSET..META_SEQ_OFFSET + 8].copy_from_slice(&1u64.to_le_bytes());
        buf[META_WIDX_OFFSET..META_WIDX_OFFSET + 8].copy_from_slice(&0u64.to_le_bytes());
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::InvalidLength));
    }

    #[test]
    fn test_invalid_write_len_exceeds_buffer() {
        let mut buf = vec![0u8; 1024];
        buf[META_SEQ_OFFSET..META_SEQ_OFFSET + 8].copy_from_slice(&1u64.to_le_bytes());
        buf[META_WIDX_OFFSET..META_WIDX_OFFSET + 8].copy_from_slice(&10000u64.to_le_bytes());
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true, true);

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
        let mut reader = ShmReader::new(mmap1, true, true);
        assert!(matches!(reader.try_read_frame(), ReadResult::Frame(_)));

        // Frame 2: seq jumps to 10 (lapped), not a keyframe
        let payload2 = b"frame2";
        let crc2 = crc32fast::hash(payload2) as i32;
        let hdr2 = FrameHeader {
            frame_id: 10, frame_seq: 10, size: payload2.len() as u32,
            flags: 0, codec: CODEC_H264, crc32: crc2, reserved: 0,
        };
        let buf2 = make_shm_buffer(2, &hdr2, payload2);
        reader.mmap = make_mmap(&buf2);
        reader.mmap_len = reader.mmap.len();

        assert!(matches!(reader.try_read_frame(), ReadResult::SkippedNonKeyframe));

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
        let buf = vec![0u8; 1024];
        let mmap = make_mmap(&buf);
        let mut reader = ShmReader::new(mmap, true, true);

        assert!(matches!(reader.try_read_frame(), ReadResult::NoNewFrame));
    }
}
