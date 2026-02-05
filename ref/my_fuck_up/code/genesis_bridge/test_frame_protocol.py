"""Tests for binary frame protocol."""
import pytest
from genesis_bridge.frame_protocol import (
    FrameType, FrameFlags, pack_frame_header, unpack_frame_header,
    HEADER_SIZE, PROTOCOL_VERSION
)


class TestFrameProtocol:
    def test_header_size_is_12_bytes(self):
        assert HEADER_SIZE == 12

    def test_protocol_version_is_1(self):
        assert PROTOCOL_VERSION == 0x01

    def test_pack_jpeg_frame_header(self):
        header = pack_frame_header(
            frame_type=FrameType.JPEG,
            payload_len=1000,
            frame_id=42,
            flags=0
        )
        assert len(header) == 12
        assert header[0] == 0x01  # version
        assert header[1] == 0x00  # JPEG type

    def test_pack_h264_keyframe_header(self):
        header = pack_frame_header(
            frame_type=FrameType.H264,
            payload_len=5000,
            frame_id=100,
            flags=FrameFlags.KEYFRAME | FrameFlags.HAS_SPS_PPS
        )
        assert len(header) == 12
        assert header[0] == 0x01  # version
        assert header[1] == 0x01  # H264 type

    def test_pack_unpack_roundtrip(self):
        original = {
            'frame_type': FrameType.H264,
            'payload_len': 12345,
            'frame_id': 9999,
            'flags': FrameFlags.KEYFRAME | FrameFlags.ANNEXB
        }
        header = pack_frame_header(**original)
        unpacked = unpack_frame_header(header)

        assert unpacked['version'] == PROTOCOL_VERSION
        assert unpacked['frame_type'] == original['frame_type']
        assert unpacked['payload_len'] == original['payload_len']
        assert unpacked['frame_id'] == original['frame_id']
        assert unpacked['flags'] == original['flags']


class TestAnnexBToAVCC:
    def test_single_nal_conversion(self):
        from genesis_bridge.frame_protocol import annexb_to_avcc

        # Single NAL with 4-byte start code
        annexb = b'\x00\x00\x00\x01' + b'\x65' + b'\xAB' * 10  # IDR slice
        avcc = annexb_to_avcc(annexb)

        # Should have 4-byte length prefix instead of start code
        assert avcc[:4] == b'\x00\x00\x00\x0B'  # length = 11
        assert avcc[4:] == b'\x65' + b'\xAB' * 10

    def test_multiple_nals_conversion(self):
        from genesis_bridge.frame_protocol import annexb_to_avcc

        # SPS + PPS + IDR
        sps = b'\x00\x00\x00\x01\x67\x42\x00\x1E'  # 4-byte NAL
        pps = b'\x00\x00\x00\x01\x68\xCE\x3C\x80'  # 4-byte NAL
        idr = b'\x00\x00\x00\x01\x65' + b'\x00' * 20  # 21-byte NAL

        annexb = sps + pps + idr
        avcc = annexb_to_avcc(annexb)

        # Verify structure: [len][sps][len][pps][len][idr]
        assert len(avcc) == len(annexb)  # Same size (start codes = length prefixes)

    def test_three_byte_start_code(self):
        from genesis_bridge.frame_protocol import annexb_to_avcc

        # 3-byte start code (00 00 01)
        annexb = b'\x00\x00\x01\x65' + b'\xFF' * 5
        avcc = annexb_to_avcc(annexb)

        # Should handle 3-byte start codes too
        assert avcc[:4] == b'\x00\x00\x00\x06'  # length = 6

    def test_empty_input(self):
        from genesis_bridge.frame_protocol import annexb_to_avcc

        assert annexb_to_avcc(b'') == b''

    def test_mixed_start_codes(self):
        from genesis_bridge.frame_protocol import annexb_to_avcc

        # 4-byte start code followed by 3-byte start code
        annexb = b'\x00\x00\x00\x01\x67\xAA\xBB' + b'\x00\x00\x01\x68\xCC\xDD'
        avcc = annexb_to_avcc(annexb)

        # First NAL: 0x67 0xAA 0xBB (3 bytes)
        assert avcc[0:4] == b'\x00\x00\x00\x03'
        assert avcc[4:7] == b'\x67\xAA\xBB'

        # Second NAL: 0x68 0xCC 0xDD (3 bytes)
        assert avcc[7:11] == b'\x00\x00\x00\x03'
        assert avcc[11:14] == b'\x68\xCC\xDD'


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
