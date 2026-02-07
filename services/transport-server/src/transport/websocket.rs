use crate::shm::protocol::{FrameHeader, HEADER_SIZE};
use crate::shm::reader::Frame;

/// WS message type byte prefixes.
pub const MSG_TYPE_VIDEO: u8 = 0x01;
pub const MSG_TYPE_TELEMETRY: u8 = 0x02;
pub const MSG_TYPE_SIGNALING: u8 = 0x03;
pub const MSG_TYPE_COMMAND: u8 = 0x04;

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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::shm::protocol::{CODEC_H264, CODEC_HEVC, FLAG_KEYFRAME};

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

    #[test]
    fn test_encode_video_hevc() {
        let frame = Frame {
            header: FrameHeader {
                frame_id: 5, frame_seq: 5, size: 3,
                flags: 0, codec: CODEC_HEVC, crc32: 0, reserved: 0,
            },
            payload: b"abc".to_vec(),
        };
        let encoded = encode_video_frame(&frame);
        let hdr = FrameHeader::from_bytes(&encoded[1..1 + HEADER_SIZE]).unwrap();
        assert_eq!(hdr.codec, CODEC_HEVC);
        assert!(!hdr.is_keyframe());
    }
}
