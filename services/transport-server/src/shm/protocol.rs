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
