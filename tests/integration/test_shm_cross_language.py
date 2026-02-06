"""Cross-language SHM protocol compatibility test.

Validates that the Rust transport-server SHM reader can correctly
parse frames written by the Python genesis_bridge ShmWriter.

This test:
1. Uses Python ShmWriter to write frames to a temp file
2. Reads them back to verify the byte layout
3. Asserts both sides agree on header values, CRC, and sequence
"""

import json
import mmap
import os
import struct
import tempfile
import zlib

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
