"""SHM ringbuffer for zero-copy frame transport between processes.

Protocol from SDR_OS multi-service backend design (Section 3).

Layout (at start of mmap region):
    [0:8]   sequence_number  uint64  (written LAST - commit marker)
    [8:16]  write_index      uint64
    [16:]   frame data (header + payload, wrapping)

Frame Header (32 bytes):
    frame_id:   uint64  # Application-level ID
    frame_seq:  uint64  # Monotonic, never resets (lap detection)
    size:       uint32  # Payload size in bytes
    flags:      uint16  # FLAG_KEYFRAME, FLAG_WRAP_MARKER, etc.
    codec:      uint16  # 1=H.264, 2=HEVC
    crc32:      uint32  # zlib.crc32 of payload
    reserved:   uint32

Atomic Commit Pattern:
    Writer: write header + payload → write write_index → write sequence LAST
    Reader: read seq1 → read write_index → read seq2 → accept only if seq1 == seq2

Lap Detection:
    Reader tracks last_frame_seq.
    If header.frame_seq > expected: writer lapped → resync to IDR
    If header.frame_seq < expected: corruption → resync to IDR
    On resync: last_frame_seq = 0, awaiting_keyframe = True, request IDR

Drop Policy:
    Writer: "Latest wins" — always overwrites with freshest frame.
"""
import mmap
import os
import struct
import threading
import zlib
from dataclasses import dataclass
from enum import IntFlag
from pathlib import Path
from typing import Optional


# ──────────────── Constants ────────────────

CONTROL_REGION_SIZE = 16  # 8 (sequence) + 8 (write_index)
FRAME_HEADER_SIZE = 32
FRAME_HEADER_FORMAT = "<QQIHHIxxxx"  # Note: xxxx for 4 bytes reserved padding
# Actually let's be precise: Q=8, Q=8, I=4, H=2, H=2, I=4, I=4 = 32
FRAME_HEADER_FORMAT = "<QQIHHII"  # frame_id, frame_seq, size, flags, codec, crc32, reserved

DEFAULT_BUFFER_SIZE = 4 * 1024 * 1024  # 4 MB
DEFAULT_SHM_PATH = "/dev/shm/sdr_os_ipc/frames"


class FrameFlags(IntFlag):
    NONE = 0x0000
    KEYFRAME = 0x0001
    WRAP_MARKER = 0x0002


class Codec(int):
    H264 = 1
    HEVC = 2
    JPEG = 3


@dataclass(frozen=True, slots=True)
class FrameHeader:
    frame_id: int
    frame_seq: int
    size: int
    flags: int
    codec: int
    crc32: int
    reserved: int = 0

    def pack(self) -> bytes:
        return struct.pack(
            FRAME_HEADER_FORMAT,
            self.frame_id,
            self.frame_seq,
            self.size,
            self.flags,
            self.codec,
            self.crc32,
            self.reserved,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "FrameHeader":
        if len(data) < FRAME_HEADER_SIZE:
            raise ValueError(f"Header too short: {len(data)} < {FRAME_HEADER_SIZE}")
        fields = struct.unpack(FRAME_HEADER_FORMAT, data[:FRAME_HEADER_SIZE])
        return cls(*fields)


@dataclass(frozen=True, slots=True)
class Frame:
    header: FrameHeader
    payload: bytes


# ──────────────── Writer ────────────────


class ShmRingWriter:
    """Writes frames to a shared memory ringbuffer.

    Uses "latest wins" drop policy — always overwrites with the freshest frame.
    Single-slot design: one frame at a time, reader must keep up.

    Args:
        path: Path to the SHM file.
        buffer_size: Total mmap region size in bytes (including control region).
    """

    def __init__(self, path: str = DEFAULT_SHM_PATH, buffer_size: int = DEFAULT_BUFFER_SIZE,
                 crc_enabled: bool = True):
        self._path = path
        self._buffer_size = buffer_size
        self._data_region_size = buffer_size - CONTROL_REGION_SIZE
        self._sequence: int = 0
        self._frame_seq: int = 0
        self._crc_enabled = crc_enabled
        self._lock = threading.Lock()

        # Ensure parent directory exists
        Path(path).parent.mkdir(parents=True, exist_ok=True)

        # Create or open the SHM file
        fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
        try:
            os.ftruncate(fd, buffer_size)
            self._mm = mmap.mmap(fd, buffer_size)
        finally:
            os.close(fd)

        # Zero the control region
        self._mm[0:CONTROL_REGION_SIZE] = b"\x00" * CONTROL_REGION_SIZE

    @property
    def data_region_offset(self) -> int:
        return CONTROL_REGION_SIZE

    def write(self, payload: bytes, frame_id: int, flags: int = 0, codec: int = Codec.H264) -> bool:
        """Write a frame to the ringbuffer.

        Args:
            payload: Encoded frame data (NAL units, etc.)
            frame_id: Application-level frame ID.
            flags: FrameFlags bitmask.
            codec: Codec identifier (1=H.264, 2=HEVC).

        Returns:
            True if written successfully, False if payload too large.
        """
        total_size = FRAME_HEADER_SIZE + len(payload)
        if total_size > self._data_region_size:
            return False

        with self._lock:
            self._frame_seq += 1
            crc = zlib.crc32(payload) & 0xFFFFFFFF if self._crc_enabled else 0

            header = FrameHeader(
                frame_id=frame_id,
                frame_seq=self._frame_seq,
                size=len(payload),
                flags=flags,
                codec=codec,
                crc32=crc,
            )

            # Write header + payload into data region (always start at offset 0 in data region)
            write_offset = self.data_region_offset
            header_bytes = header.pack()
            self._mm[write_offset:write_offset + FRAME_HEADER_SIZE] = header_bytes
            self._mm[write_offset + FRAME_HEADER_SIZE:write_offset + total_size] = payload

            # Update write_index (points to end of written data)
            write_end = total_size
            struct.pack_into("<Q", self._mm, 8, write_end)

            # Commit: write sequence number LAST (atomic marker)
            self._sequence += 1
            struct.pack_into("<Q", self._mm, 0, self._sequence)

        return True

    def close(self):
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


# ──────────────── Reader ────────────────


class ShmRingReader:
    """Reads frames from a shared memory ringbuffer.

    Implements lap detection: tracks last_frame_seq and resyncs on gaps.

    Args:
        path: Path to the SHM file.
        buffer_size: Total mmap region size (must match writer).
    """

    def __init__(self, path: str = DEFAULT_SHM_PATH, buffer_size: int = DEFAULT_BUFFER_SIZE):
        self._path = path
        self._buffer_size = buffer_size
        self._last_sequence: int = 0
        self._last_frame_seq: int = 0
        self._awaiting_keyframe: bool = False
        self._lapped_count: int = 0

        fd = os.open(path, os.O_RDONLY)
        try:
            self._mm = mmap.mmap(fd, buffer_size, access=mmap.ACCESS_READ)
        finally:
            os.close(fd)

    @property
    def data_region_offset(self) -> int:
        return CONTROL_REGION_SIZE

    @property
    def awaiting_keyframe(self) -> bool:
        return self._awaiting_keyframe

    @property
    def lapped_count(self) -> int:
        return self._lapped_count

    def read(self) -> Optional[Frame]:
        """Read the latest frame from the ringbuffer.

        Returns:
            Frame if a new, valid frame is available; None otherwise.
            Returns None if:
            - No new data (sequence unchanged)
            - Torn read detected (seq1 != seq2)
            - CRC mismatch (corruption)
            - Awaiting keyframe after lap detection
        """
        # Atomic read: seq1 → data → seq2; accept only if seq1 == seq2
        seq1 = struct.unpack_from("<Q", self._mm, 0)[0]
        write_end = struct.unpack_from("<Q", self._mm, 8)[0]
        seq2 = struct.unpack_from("<Q", self._mm, 0)[0]

        # Torn read check
        if seq1 != seq2:
            return None

        # No new data
        if seq1 == self._last_sequence:
            return None

        if write_end < FRAME_HEADER_SIZE:
            return None

        # Read header
        offset = self.data_region_offset
        header_bytes = bytes(self._mm[offset:offset + FRAME_HEADER_SIZE])
        header = FrameHeader.unpack(header_bytes)

        # Lap detection
        expected_seq = self._last_frame_seq + 1
        if self._last_frame_seq > 0 and header.frame_seq != expected_seq:
            # Writer lapped us or corruption — resync
            self._lapped_count += 1
            self._last_frame_seq = 0
            self._awaiting_keyframe = True

        # Read payload
        payload_start = offset + FRAME_HEADER_SIZE
        payload_end = payload_start + header.size
        if payload_end > self._buffer_size:
            # Payload extends beyond buffer — corruption
            self._awaiting_keyframe = True
            self._last_frame_seq = 0
            return None

        payload = bytes(self._mm[payload_start:payload_end])

        # CRC check
        computed_crc = zlib.crc32(payload) & 0xFFFFFFFF
        if computed_crc != header.crc32:
            # Corruption detected — resync
            self._awaiting_keyframe = True
            self._last_frame_seq = 0
            return None

        # If awaiting keyframe, only accept keyframes
        if self._awaiting_keyframe:
            if not (header.flags & FrameFlags.KEYFRAME):
                # Skip non-keyframes during resync
                self._last_sequence = seq1
                return None
            self._awaiting_keyframe = False

        # Accept frame
        self._last_sequence = seq1
        self._last_frame_seq = header.frame_seq
        return Frame(header=header, payload=payload)

    def close(self):
        if self._mm is not None:
            self._mm.close()
            self._mm = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
