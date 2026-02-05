"""Binary frame protocol for WebSocket video streaming.

Header format (12 bytes, big-endian):
    Offset  Size  Field
    0       1     version (0x01)
    1       1     type (0x00=JPEG, 0x01=H.264)
    2       4     payload_len (u32)
    6       4     frame_id (u32)
    10      2     flags (u16)
"""
import struct
from enum import IntEnum, IntFlag


PROTOCOL_VERSION = 0x01
HEADER_SIZE = 12
HEADER_FORMAT = '>BBIIH'  # version, type, payload_len, frame_id, flags


class FrameType(IntEnum):
    JPEG = 0x00
    H264 = 0x01


class FrameFlags(IntFlag):
    NONE = 0x00
    KEYFRAME = 0x01
    HAS_SPS_PPS = 0x02
    ANNEXB = 0x04


def pack_frame_header(
    frame_type: FrameType,
    payload_len: int,
    frame_id: int,
    flags: int = 0
) -> bytes:
    """Pack frame header as 12-byte big-endian binary."""
    return struct.pack(
        HEADER_FORMAT,
        PROTOCOL_VERSION,
        int(frame_type),
        payload_len,
        frame_id,
        flags
    )


def unpack_frame_header(header: bytes) -> dict:
    """Unpack 12-byte header into dict."""
    if len(header) < HEADER_SIZE:
        raise ValueError(f"Header too short: {len(header)} < {HEADER_SIZE}")

    version, frame_type, payload_len, frame_id, flags = struct.unpack(
        HEADER_FORMAT, header[:HEADER_SIZE]
    )

    return {
        'version': version,
        'frame_type': FrameType(frame_type),
        'payload_len': payload_len,
        'frame_id': frame_id,
        'flags': flags
    }


def annexb_to_avcc(annexb: bytes) -> bytes:
    """Convert Annex B (start codes) to AVCC (length-prefixed).

    Scans for 00 00 00 01 or 00 00 01 start codes and replaces
    with 4-byte big-endian NAL unit length.

    Note: Any bytes before the first start code are silently discarded.
    This is intentional to handle partial frames from stream concatenation.

    Args:
        annexb: Annex B formatted H.264 data

    Returns:
        AVCC formatted data with 4-byte length prefixes
    """
    if not annexb:
        return b''

    result = bytearray()
    i = 0
    n = len(annexb)

    while i < n:
        # Find start code (00 00 00 01 or 00 00 01)
        start_code_len = 0
        if i + 4 <= n and annexb[i:i+4] == b'\x00\x00\x00\x01':
            start_code_len = 4
        elif i + 3 <= n and annexb[i:i+3] == b'\x00\x00\x01':
            start_code_len = 3

        if start_code_len == 0:
            # No start code at current position, skip byte
            i += 1
            continue

        # Find next start code (or end of data)
        nal_start = i + start_code_len
        nal_end = n

        for j in range(nal_start, n - 2):
            if annexb[j:j+3] == b'\x00\x00\x01':
                # Check if it's 4-byte or 3-byte start code
                if j > nal_start and annexb[j-1] == 0:
                    nal_end = j - 1
                else:
                    nal_end = j
                break

        # Extract NAL and add with length prefix
        nal_data = annexb[nal_start:nal_end]
        nal_len = len(nal_data)
        result.extend(struct.pack('>I', nal_len))
        result.extend(nal_data)

        i = nal_end

    return bytes(result)
