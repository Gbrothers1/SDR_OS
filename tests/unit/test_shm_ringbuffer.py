"""Tests for SHM ringbuffer protocol.

Covers: header pack/unpack, write/read round-trip, CRC validation,
lap detection, keyframe gating, and oversized payload rejection.
"""
import os
import struct
import tempfile
import zlib

import pytest

from sdr_os.ipc.shm_ringbuffer import (
    CONTROL_REGION_SIZE,
    DEFAULT_BUFFER_SIZE,
    FRAME_HEADER_SIZE,
    Codec,
    Frame,
    FrameFlags,
    FrameHeader,
    ShmRingReader,
    ShmRingWriter,
)


@pytest.fixture
def shm_path(tmp_path):
    """Create a temp path for a SHM file."""
    return str(tmp_path / "test_frames")


class TestFrameHeader:
    def test_pack_unpack_roundtrip(self):
        header = FrameHeader(
            frame_id=42,
            frame_seq=100,
            size=1024,
            flags=FrameFlags.KEYFRAME,
            codec=Codec.H264,
            crc32=0xDEADBEEF,
        )
        packed = header.pack()
        assert len(packed) == FRAME_HEADER_SIZE

        unpacked = FrameHeader.unpack(packed)
        assert unpacked.frame_id == 42
        assert unpacked.frame_seq == 100
        assert unpacked.size == 1024
        assert unpacked.flags == FrameFlags.KEYFRAME
        assert unpacked.codec == Codec.H264
        assert unpacked.crc32 == 0xDEADBEEF

    def test_unpack_too_short_raises(self):
        with pytest.raises(ValueError, match="Header too short"):
            FrameHeader.unpack(b"\x00" * 16)

    def test_flags_bitmask(self):
        header = FrameHeader(
            frame_id=0,
            frame_seq=1,
            size=0,
            flags=FrameFlags.KEYFRAME | FrameFlags.WRAP_MARKER,
            codec=Codec.HEVC,
            crc32=0,
        )
        packed = header.pack()
        unpacked = FrameHeader.unpack(packed)
        assert unpacked.flags & FrameFlags.KEYFRAME
        assert unpacked.flags & FrameFlags.WRAP_MARKER


class TestWriterReader:
    def test_basic_write_read(self, shm_path):
        payload = b"hello frame data" * 10
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            ok = writer.write(payload, frame_id=1, flags=FrameFlags.KEYFRAME)
            assert ok

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                frame = reader.read()
                assert frame is not None
                assert frame.payload == payload
                assert frame.header.frame_id == 1
                assert frame.header.frame_seq == 1
                assert frame.header.flags & FrameFlags.KEYFRAME
                assert frame.header.codec == Codec.H264

    def test_crc_validation(self, shm_path):
        payload = b"valid payload"
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            ok = writer.write(payload, frame_id=1, flags=FrameFlags.KEYFRAME)
            assert ok

            # Corrupt the payload in the mmap
            offset = CONTROL_REGION_SIZE + FRAME_HEADER_SIZE
            writer._mm[offset] = (writer._mm[offset] + 1) % 256

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                frame = reader.read()
                # CRC mismatch should return None
                assert frame is None

    def test_no_new_data_returns_none(self, shm_path):
        payload = b"data"
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            writer.write(payload, frame_id=1, flags=FrameFlags.KEYFRAME)

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                frame1 = reader.read()
                assert frame1 is not None

                # Second read with no new write returns None
                frame2 = reader.read()
                assert frame2 is None

    def test_multiple_writes_latest_wins(self, shm_path):
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            writer.write(b"frame1", frame_id=1, flags=FrameFlags.KEYFRAME)
            writer.write(b"frame2", frame_id=2, flags=FrameFlags.KEYFRAME)
            writer.write(b"frame3", frame_id=3, flags=FrameFlags.KEYFRAME)

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                frame = reader.read()
                assert frame is not None
                # Should get the latest frame
                assert frame.payload == b"frame3"
                assert frame.header.frame_id == 3

    def test_oversized_payload_rejected(self, shm_path):
        buf_size = 1024
        with ShmRingWriter(shm_path, buf_size) as writer:
            # Payload that exceeds data region
            big_payload = b"x" * (buf_size - CONTROL_REGION_SIZE)
            ok = writer.write(big_payload, frame_id=1)
            assert not ok

    def test_frame_seq_monotonic(self, shm_path):
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            writer.write(b"a", frame_id=1, flags=FrameFlags.KEYFRAME)

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                f1 = reader.read()
                assert f1.header.frame_seq == 1

            writer.write(b"b", frame_id=2, flags=FrameFlags.KEYFRAME)

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                f2 = reader.read()
                assert f2.header.frame_seq == 2
                assert f2.header.frame_seq > f1.header.frame_seq

    def test_codec_field(self, shm_path):
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            writer.write(b"hevc data", frame_id=1, flags=FrameFlags.KEYFRAME, codec=Codec.HEVC)

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                frame = reader.read()
                assert frame.header.codec == Codec.HEVC


class TestLapDetection:
    def test_sequential_frames_no_lap(self, shm_path):
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                writer.write(b"f1", frame_id=1, flags=FrameFlags.KEYFRAME)
                f1 = reader.read()
                assert f1 is not None
                assert reader.lapped_count == 0

                writer.write(b"f2", frame_id=2)
                f2 = reader.read()
                assert f2 is not None
                assert reader.lapped_count == 0

    def test_gap_triggers_lap_and_keyframe_wait(self, shm_path):
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                # Read frame 1
                writer.write(b"f1", frame_id=1, flags=FrameFlags.KEYFRAME)
                f1 = reader.read()
                assert f1 is not None

                # Write frames 2 and 3 without reading, then write 4
                writer.write(b"f2", frame_id=2)
                writer.write(b"f3", frame_id=3)
                writer.write(b"f4_no_kf", frame_id=4)

                # Reader expects frame_seq=2 but gets frame_seq=4 → lap detected
                frame = reader.read()
                # Should be None because non-keyframe during resync
                assert frame is None
                assert reader.lapped_count == 1
                assert reader.awaiting_keyframe

                # Write a keyframe to resync
                writer.write(b"f5_kf", frame_id=5, flags=FrameFlags.KEYFRAME)
                frame = reader.read()
                assert frame is not None
                assert frame.payload == b"f5_kf"
                assert not reader.awaiting_keyframe

    def test_non_keyframe_skipped_during_resync(self, shm_path):
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                # Establish baseline
                writer.write(b"f1", frame_id=1, flags=FrameFlags.KEYFRAME)
                reader.read()

                # Force lap
                writer.write(b"f2", frame_id=2)
                writer.write(b"f10", frame_id=10)  # seq gap: 2→3 expected, got 3
                # Actually the frame_seq is monotonic from the writer's perspective
                # Writer's frame_seq will be 3, reader expects 2
                # So seq 3 != expected 2 → lap

                frame = reader.read()
                # frame_seq=3, expected=2, gap detected
                # f10 has no KEYFRAME flag → skipped
                assert frame is None
                assert reader.awaiting_keyframe


class TestEdgeCases:
    def test_empty_payload(self, shm_path):
        with ShmRingWriter(shm_path, DEFAULT_BUFFER_SIZE) as writer:
            ok = writer.write(b"", frame_id=1, flags=FrameFlags.KEYFRAME)
            assert ok

            with ShmRingReader(shm_path, DEFAULT_BUFFER_SIZE) as reader:
                frame = reader.read()
                assert frame is not None
                assert frame.payload == b""
                assert frame.header.size == 0

    def test_max_payload_fits(self, shm_path):
        buf_size = 4096
        max_payload = buf_size - CONTROL_REGION_SIZE - FRAME_HEADER_SIZE
        payload = b"x" * max_payload

        with ShmRingWriter(shm_path, buf_size) as writer:
            ok = writer.write(payload, frame_id=1, flags=FrameFlags.KEYFRAME)
            assert ok

            with ShmRingReader(shm_path, buf_size) as reader:
                frame = reader.read()
                assert frame is not None
                assert len(frame.payload) == max_payload

    def test_writer_context_manager(self, shm_path):
        with ShmRingWriter(shm_path) as writer:
            writer.write(b"test", frame_id=1, flags=FrameFlags.KEYFRAME)
        # Should not raise after close

    def test_reader_context_manager(self, shm_path):
        with ShmRingWriter(shm_path) as writer:
            writer.write(b"test", frame_id=1, flags=FrameFlags.KEYFRAME)
        with ShmRingReader(shm_path) as reader:
            frame = reader.read()
            assert frame is not None
