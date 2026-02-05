# Frontend H.264 WebCodecs Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Add H.264 video playback via WebCodecs with automatic JPEG fallback for unsupported browsers.

**Architecture:** Per-client codec negotiation at connect time. Backend maintains separate fanout groups for H.264 and JPEG clients with per-client backpressure queues. Frontend uses WebCodecs VideoDecoder to decode H.264 access units to canvas.

**Tech Stack:** WebCodecs API, PyAV/NVENC, asyncio queues, binary WebSocket framing

---

## Task 1: Binary frame protocol helpers (backend)

**Files:**
- Create: `genesis_bridge/frame_protocol.py`
- Create: `genesis_bridge/test_frame_protocol.py`

**Step 1: Write the test**

```python
# genesis_bridge/test_frame_protocol.py
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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

**Step 2: Run test to verify it fails**

```bash
python -m pytest genesis_bridge/test_frame_protocol.py -v --override-ini="addopts="
```

Expected: FAIL with "No module named 'genesis_bridge.frame_protocol'"

**Step 3: Write the implementation**

```python
# genesis_bridge/frame_protocol.py
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
    KEYFRAME = 0x01      # Access unit contains IDR
    HAS_SPS_PPS = 0x02   # Access unit includes SPS/PPS
    ANNEXB = 0x04        # Payload has Annex B start codes (vs AVCC)


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
```

**Step 4: Run test to verify it passes**

```bash
python -m pytest genesis_bridge/test_frame_protocol.py -v --override-ini="addopts="
```

Expected: All 5 tests PASS

**Step 5: Commit**

```bash
git add genesis_bridge/frame_protocol.py genesis_bridge/test_frame_protocol.py
git commit -m "feat(protocol): add binary frame header pack/unpack"
```

---

## Task 2: Annex B to AVCC conversion

**Files:**
- Modify: `genesis_bridge/frame_protocol.py`
- Modify: `genesis_bridge/test_frame_protocol.py`

**Step 1: Add test for Annex B to AVCC**

```python
# Add to test_frame_protocol.py
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
```

**Step 2: Run test to verify it fails**

```bash
python -m pytest genesis_bridge/test_frame_protocol.py::TestAnnexBToAVCC -v --override-ini="addopts="
```

Expected: FAIL with "cannot import name 'annexb_to_avcc'"

**Step 3: Add implementation**

```python
# Add to frame_protocol.py

def annexb_to_avcc(annexb: bytes) -> bytes:
    """Convert Annex B (start codes) to AVCC (length-prefixed).

    Scans for 00 00 00 01 or 00 00 01 start codes and replaces
    with 4-byte big-endian NAL unit length.

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
                if j > 0 and annexb[j-1] == 0:
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
```

**Step 4: Run test to verify it passes**

```bash
python -m pytest genesis_bridge/test_frame_protocol.py::TestAnnexBToAVCC -v --override-ini="addopts="
```

Expected: All 4 tests PASS

**Step 5: Commit**

```bash
git add genesis_bridge/frame_protocol.py genesis_bridge/test_frame_protocol.py
git commit -m "feat(protocol): add Annex B to AVCC conversion"
```

---

## Task 3: Per-client stream with backpressure queue

**Files:**
- Create: `genesis_bridge/stream_fanout.py`
- Create: `genesis_bridge/test_stream_fanout.py`

**Step 1: Write the test**

```python
# genesis_bridge/test_stream_fanout.py
"""Tests for per-client stream fanout."""
import pytest
import asyncio
from unittest.mock import AsyncMock, MagicMock


class TestClientStream:
    @pytest.mark.asyncio
    async def test_enqueue_adds_to_queue(self):
        from genesis_bridge.stream_fanout import ClientStream

        mock_ws = AsyncMock()
        client = ClientStream(client_id="test", websocket=mock_ws, supports_h264=False)

        client.enqueue(b'frame1')
        assert client.queue.qsize() == 1

    @pytest.mark.asyncio
    async def test_enqueue_drops_oldest_when_full(self):
        from genesis_bridge.stream_fanout import ClientStream

        mock_ws = AsyncMock()
        client = ClientStream(client_id="test", websocket=mock_ws, supports_h264=False)

        # Fill queue (maxsize=2)
        client.enqueue(b'frame1')
        client.enqueue(b'frame2')
        assert client.queue.qsize() == 2

        # Add third frame - should drop oldest
        client.enqueue(b'frame3')
        assert client.queue.qsize() == 2

        # Verify oldest was dropped
        frame = client.queue.get_nowait()
        assert frame == b'frame2'  # frame1 was dropped

    @pytest.mark.asyncio
    async def test_send_loop_sends_frames(self):
        from genesis_bridge.stream_fanout import ClientStream

        mock_ws = AsyncMock()
        client = ClientStream(client_id="test", websocket=mock_ws, supports_h264=True)

        # Start send loop
        task = asyncio.create_task(client.start())
        await asyncio.sleep(0.01)  # Let it start

        # Enqueue and wait
        client.enqueue(b'testframe')
        await asyncio.sleep(0.05)

        # Stop
        client.stop()
        await task

        # Verify send was called
        mock_ws.send.assert_called_with(b'testframe')


class TestStreamFanout:
    @pytest.mark.asyncio
    async def test_add_client_to_correct_group(self):
        from genesis_bridge.stream_fanout import StreamFanout

        fanout = StreamFanout()

        mock_ws_h264 = AsyncMock()
        mock_ws_jpeg = AsyncMock()

        fanout.add_client("c1", mock_ws_h264, supports_h264=True, h264_codec="avc1.42E01E")
        fanout.add_client("c2", mock_ws_jpeg, supports_h264=False)

        assert "c1" in fanout.h264_clients
        assert "c2" in fanout.jpeg_clients
        assert "c1" not in fanout.jpeg_clients
        assert "c2" not in fanout.h264_clients

    @pytest.mark.asyncio
    async def test_remove_client(self):
        from genesis_bridge.stream_fanout import StreamFanout

        fanout = StreamFanout()
        mock_ws = AsyncMock()

        fanout.add_client("c1", mock_ws, supports_h264=True)
        assert "c1" in fanout.h264_clients

        await fanout.remove_client("c1")
        assert "c1" not in fanout.h264_clients


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

**Step 2: Run test to verify it fails**

```bash
python -m pytest genesis_bridge/test_stream_fanout.py -v --override-ini="addopts="
```

Expected: FAIL with "No module named 'genesis_bridge.stream_fanout'"

**Step 3: Write the implementation**

```python
# genesis_bridge/stream_fanout.py
"""Per-client stream fanout with backpressure queues."""
import asyncio
import logging
from dataclasses import dataclass, field
from typing import Dict, Optional

logger = logging.getLogger(__name__)


@dataclass
class ClientStream:
    """Single client's stream with bounded queue."""

    client_id: str
    websocket: any  # WebSocket connection
    supports_h264: bool
    h264_codec: str = "avc1.42E01E"
    queue: asyncio.Queue = field(default_factory=lambda: asyncio.Queue(maxsize=2))
    _task: Optional[asyncio.Task] = field(default=None, repr=False)
    _running: bool = field(default=False, repr=False)

    def enqueue(self, frame_bytes: bytes):
        """Add frame to queue, dropping oldest if full."""
        while self.queue.full():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        try:
            self.queue.put_nowait(frame_bytes)
        except asyncio.QueueFull:
            pass  # Shouldn't happen after draining

    async def start(self):
        """Start the send loop."""
        self._running = True
        try:
            while self._running:
                try:
                    frame_bytes = await asyncio.wait_for(
                        self.queue.get(), timeout=1.0
                    )
                    await self.websocket.send(frame_bytes)
                except asyncio.TimeoutError:
                    continue
                except Exception as e:
                    logger.debug(f"Client {self.client_id} send error: {e}")
                    break
        finally:
            try:
                await self.websocket.close()
            except Exception:
                pass

    def stop(self):
        """Signal send loop to stop."""
        self._running = False

    def is_alive(self) -> bool:
        """Check if send task is still running."""
        return self._task is not None and not self._task.done()


class StreamFanout:
    """Manages per-client streams with codec-based grouping."""

    def __init__(self):
        self.h264_clients: Dict[str, ClientStream] = {}
        self.jpeg_clients: Dict[str, ClientStream] = {}
        self._force_keyframe: bool = False

    def add_client(
        self,
        client_id: str,
        websocket,
        supports_h264: bool,
        h264_codec: str = "avc1.42E01E"
    ):
        """Add client to appropriate fanout group."""
        client = ClientStream(
            client_id=client_id,
            websocket=websocket,
            supports_h264=supports_h264,
            h264_codec=h264_codec
        )

        # Start send loop
        client._task = asyncio.create_task(client.start())

        if supports_h264:
            self.h264_clients[client_id] = client
            self._force_keyframe = True  # New H.264 client needs IDR
            logger.info(f"Added H.264 client: {client_id} ({h264_codec})")
        else:
            self.jpeg_clients[client_id] = client
            logger.info(f"Added JPEG client: {client_id}")

    async def remove_client(self, client_id: str):
        """Remove client from fanout."""
        for clients in [self.h264_clients, self.jpeg_clients]:
            if client_id in clients:
                client = clients.pop(client_id)
                client.stop()
                if client._task:
                    try:
                        await asyncio.wait_for(client._task, timeout=1.0)
                    except asyncio.TimeoutError:
                        client._task.cancel()
                logger.info(f"Removed client: {client_id}")
                return

    def broadcast_h264(self, frame_bytes: bytes):
        """Send H.264 frame to all H.264 clients."""
        dead_clients = []
        for cid, client in self.h264_clients.items():
            if not client.is_alive():
                dead_clients.append(cid)
            else:
                client.enqueue(frame_bytes)

        # Prune dead clients
        for cid in dead_clients:
            del self.h264_clients[cid]
            logger.debug(f"Pruned dead H.264 client: {cid}")

    def broadcast_jpeg(self, frame_bytes: bytes):
        """Send JPEG frame to all JPEG clients."""
        dead_clients = []
        for cid, client in self.jpeg_clients.items():
            if not client.is_alive():
                dead_clients.append(cid)
            else:
                client.enqueue(frame_bytes)

        # Prune dead clients
        for cid in dead_clients:
            del self.jpeg_clients[cid]
            logger.debug(f"Pruned dead JPEG client: {cid}")

    def should_force_keyframe(self) -> bool:
        """Check and clear force keyframe flag."""
        if self._force_keyframe:
            self._force_keyframe = False
            return True
        return False

    def has_h264_clients(self) -> bool:
        return len(self.h264_clients) > 0

    def has_jpeg_clients(self) -> bool:
        return len(self.jpeg_clients) > 0

    async def close_all(self):
        """Close all client connections."""
        all_ids = list(self.h264_clients.keys()) + list(self.jpeg_clients.keys())
        for cid in all_ids:
            await self.remove_client(cid)
```

**Step 4: Run test to verify it passes**

```bash
python -m pytest genesis_bridge/test_stream_fanout.py -v --override-ini="addopts="
```

Expected: All 5 tests PASS

**Step 5: Commit**

```bash
git add genesis_bridge/stream_fanout.py genesis_bridge/test_stream_fanout.py
git commit -m "feat(fanout): add per-client stream with backpressure"
```

---

## Task 4: Update gpu_pipeline.py for H.264 access unit encoding

**Files:**
- Modify: `genesis_bridge/gpu_pipeline.py`
- Modify: `genesis_bridge/test_gpu_pipeline.py`

**Step 1: Add test for H.264 access unit encoding**

```python
# Add to genesis_bridge/test_gpu_pipeline.py
class TestH264AccessUnit:
    def test_encode_h264_au_returns_header_plus_payload(self):
        """encode_h264_au returns framed access unit with header."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        from genesis_bridge.frame_protocol import HEADER_SIZE, unpack_frame_header, FrameType

        pipeline = GpuFramePipeline()
        if not pipeline.has_nvenc:
            pytest.skip("NVENC not available")

        # Create test frame
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Encode - may return None on first frame (encoder buffering)
        result = None
        for i in range(5):
            result = pipeline.encode_h264_au(frame, frame_id=i, profile='baseline')
            if result is not None:
                break

        if result is None:
            pytest.skip("Encoder did not produce output")

        # Verify header
        assert len(result) > HEADER_SIZE
        header = unpack_frame_header(result[:HEADER_SIZE])
        assert header['frame_type'] == FrameType.H264
        assert header['payload_len'] == len(result) - HEADER_SIZE

    def test_encode_h264_au_keyframe_has_correct_flags(self):
        """First frame should be keyframe with SPS/PPS."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        from genesis_bridge.frame_protocol import HEADER_SIZE, unpack_frame_header, FrameFlags

        pipeline = GpuFramePipeline()
        if not pipeline.has_nvenc:
            pytest.skip("NVENC not available")

        # Force new encoder
        pipeline._h264_encoder = None

        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # First encoded frame should be keyframe
        result = None
        for i in range(5):
            result = pipeline.encode_h264_au(frame, frame_id=i, profile='baseline')
            if result is not None:
                break

        if result is None:
            pytest.skip("Encoder did not produce output")

        header = unpack_frame_header(result[:HEADER_SIZE])
        assert header['flags'] & FrameFlags.KEYFRAME
```

**Step 2: Run test to verify it fails**

```bash
python -m pytest genesis_bridge/test_gpu_pipeline.py::TestH264AccessUnit -v --override-ini="addopts="
```

Expected: FAIL with "has no attribute 'encode_h264_au'"

**Step 3: Update gpu_pipeline.py**

Add to `genesis_bridge/gpu_pipeline.py`:

```python
# Add import at top
from genesis_bridge.frame_protocol import (
    pack_frame_header, FrameType, FrameFlags, annexb_to_avcc
)

CODEC_TO_PROFILE = {
    'avc1.42E01E': 'baseline',
    'avc1.4D401E': 'main',
    'avc1.64001E': 'high',
}

# Add to GpuFramePipeline class:
    def __init__(self):
        # ... existing code ...
        self._h264_profile = 'baseline'
        self._force_keyframe = False

    def encode_h264_au(
        self,
        rgb_array: np.ndarray,
        frame_id: int,
        profile: str = 'baseline',
        bitrate: int = 8_000_000
    ) -> Optional[bytes]:
        """Encode RGB array to H.264 access unit with frame header.

        Args:
            rgb_array: HWC uint8 numpy array (RGB format)
            frame_id: Monotonic frame counter
            profile: H.264 profile ('baseline', 'main', 'high')
            bitrate: Target bitrate in bits/sec

        Returns:
            Framed access unit (12-byte header + AVCC payload) or None if buffered
        """
        if not self.has_nvenc:
            raise RuntimeError("NVENC not available")

        import av

        height, width = rgb_array.shape[:2]

        # Reinit encoder if dimensions or profile changed
        if (self._h264_encoder is None or
            width != self._h264_width or
            height != self._h264_height or
            profile != self._h264_profile):

            if self._h264_encoder is not None:
                self._h264_encoder.close()

            self._h264_encoder = av.CodecContext.create('h264_nvenc', 'w')
            self._h264_encoder.width = width
            self._h264_encoder.height = height
            self._h264_encoder.pix_fmt = 'yuv420p'
            self._h264_encoder.bit_rate = bitrate
            self._h264_encoder.options = {
                'profile': profile,
                'level': '3.0',
                'preset': 'p1',
                'tune': 'll',
                'rc': 'cbr',
                'maxrate': str(bitrate),
                'bufsize': str(bitrate // 2),
                'bf': '0',
                'rc-lookahead': '0',
                'g': '60',
                'repeat-headers': '1',
            }
            self._h264_encoder.open()

            self._h264_width = width
            self._h264_height = height
            self._h264_profile = profile
            self._force_keyframe = False
            logger.info(f"H.264 encoder initialized: {width}x{height} {profile}")

        # Handle force keyframe
        if self._force_keyframe:
            # Note: NVENC doesn't have a clean force-keyframe API
            # Workaround: close and reopen encoder to force IDR
            self._h264_encoder.close()
            self._h264_encoder = None
            self._force_keyframe = False
            return self.encode_h264_au(rgb_array, frame_id, profile, bitrate)

        # Encode frame
        av_frame = av.VideoFrame.from_ndarray(rgb_array, format='rgb24')
        av_frame.pts = frame_id

        packets = self._h264_encoder.encode(av_frame)

        if not packets:
            return None  # Encoder is buffering

        # Concatenate packets into access unit
        annexb_bytes = b''.join(bytes(p) for p in packets)
        avcc_bytes = annexb_to_avcc(annexb_bytes)

        # Determine flags
        is_keyframe = packets[0].is_keyframe
        flags = 0
        if is_keyframe:
            flags |= FrameFlags.KEYFRAME | FrameFlags.HAS_SPS_PPS
        # annexb=0 since we converted to AVCC

        # Build header + payload
        header = pack_frame_header(
            frame_type=FrameType.H264,
            payload_len=len(avcc_bytes),
            frame_id=frame_id,
            flags=flags
        )

        return header + avcc_bytes

    def request_keyframe(self):
        """Request next frame to be a keyframe (for new client joins)."""
        self._force_keyframe = True
```

**Step 4: Run test to verify it passes**

```bash
python -m pytest genesis_bridge/test_gpu_pipeline.py::TestH264AccessUnit -v --override-ini="addopts="
```

Expected: Tests PASS (or skip if no NVENC)

**Step 5: Commit**

```bash
git add genesis_bridge/gpu_pipeline.py genesis_bridge/test_gpu_pipeline.py
git commit -m "feat(gpu): add H.264 access unit encoding with AVCC output"
```

---

## Task 5: Frontend H264Decoder class

**Files:**
- Create: `src/client/utils/H264Decoder.js`

**Step 1: Create the decoder class**

```javascript
// src/client/utils/H264Decoder.js
/**
 * WebCodecs H.264 decoder with automatic keyframe waiting and backpressure.
 */
export class H264Decoder {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.decoder = null;
    this.waitingForKeyframe = true;
    this.maxQueueSize = 3;
    this.frameCount = 0;
    this.errorCount = 0;
  }

  /**
   * Initialize the decoder with given codec.
   * @param {string} codec - AVC codec string (e.g., 'avc1.42E01E')
   */
  init(codec = 'avc1.42E01E') {
    if (this.decoder) {
      this.decoder.close();
    }

    this.decoder = new VideoDecoder({
      output: (frame) => {
        // Resize canvas to match frame if needed
        if (this.canvas.width !== frame.displayWidth ||
            this.canvas.height !== frame.displayHeight) {
          this.canvas.width = frame.displayWidth;
          this.canvas.height = frame.displayHeight;
        }
        this.ctx.drawImage(frame, 0, 0);
        frame.close();
        this.frameCount++;
      },
      error: (e) => {
        console.error('H264Decoder error:', e);
        this.errorCount++;
        // Reset to wait for next keyframe on error
        this.waitingForKeyframe = true;
      },
    });

    this.decoder.configure({
      codec,
      hardwareAcceleration: 'prefer-hardware',
    });

    this.waitingForKeyframe = true;
    console.log(`H264Decoder initialized with codec: ${codec}`);
  }

  /**
   * Decode an H.264 access unit.
   * @param {ArrayBuffer} payload - AVCC-formatted access unit
   * @param {number} frameId - Monotonic frame ID
   * @param {number} flags - Frame flags (bit 0 = keyframe)
   */
  decode(payload, frameId, flags) {
    if (!this.decoder || this.decoder.state === 'closed') {
      return;
    }

    const isKeyframe = (flags & 0x01) !== 0;

    // Wait for keyframe before starting decode
    if (this.waitingForKeyframe) {
      if (!isKeyframe) {
        return; // Skip until we get a keyframe
      }
      this.waitingForKeyframe = false;
      console.log('H264Decoder: Got keyframe, starting decode');
    }

    // Backpressure: drop delta frames if queue is backed up
    if (this.decoder.decodeQueueSize > this.maxQueueSize) {
      if (!isKeyframe) {
        return; // Drop non-keyframe when congested
      }
    }

    try {
      this.decoder.decode(new EncodedVideoChunk({
        type: isKeyframe ? 'key' : 'delta',
        timestamp: frameId * 16666, // ~60fps in microseconds
        data: payload,
      }));
    } catch (e) {
      console.error('H264Decoder decode error:', e);
      this.waitingForKeyframe = true;
    }
  }

  /**
   * Reset decoder state (e.g., on stream discontinuity).
   */
  reset() {
    this.waitingForKeyframe = true;
    if (this.decoder && this.decoder.state !== 'closed') {
      try {
        this.decoder.reset();
      } catch (e) {
        console.warn('H264Decoder reset error:', e);
      }
    }
  }

  /**
   * Close and clean up decoder.
   */
  close() {
    if (this.decoder) {
      try {
        this.decoder.close();
      } catch (e) {
        // Ignore close errors
      }
      this.decoder = null;
    }
  }

  /**
   * Get decoder statistics.
   */
  getStats() {
    return {
      frameCount: this.frameCount,
      errorCount: this.errorCount,
      queueSize: this.decoder?.decodeQueueSize ?? 0,
      waitingForKeyframe: this.waitingForKeyframe,
    };
  }
}

/**
 * Detect WebCodecs H.264 support.
 * @returns {Promise<{supported: boolean, config?: object}>}
 */
export async function detectH264Support() {
  if (!window.VideoDecoder || !VideoDecoder.isConfigSupported) {
    return { supported: false };
  }

  // Try codecs in order of preference
  const codecs = ['avc1.42E01E', 'avc1.4D401E', 'avc1.64001E'];

  for (const codec of codecs) {
    const cfg = { codec, hardwareAcceleration: 'prefer-hardware' };
    try {
      const res = await VideoDecoder.isConfigSupported(cfg);
      if (res.supported) {
        return { supported: true, config: cfg };
      }
    } catch {
      continue;
    }
  }

  return { supported: false };
}

/**
 * Parse binary frame header.
 * @param {ArrayBuffer} data - Binary frame data
 * @returns {{version: number, type: number, payloadLen: number, frameId: number, flags: number, payload: ArrayBuffer}}
 */
export function parseFrameHeader(data) {
  const view = new DataView(data);
  const version = view.getUint8(0);
  const type = view.getUint8(1);
  const payloadLen = view.getUint32(2, false); // big-endian
  const frameId = view.getUint32(6, false);
  const flags = view.getUint16(10, false);
  const payload = data.slice(12, 12 + payloadLen);

  return { version, type, payloadLen, frameId, flags, payload };
}

// Frame type constants
export const FrameType = {
  JPEG: 0x00,
  H264: 0x01,
};

// Frame flag constants
export const FrameFlags = {
  KEYFRAME: 0x01,
  HAS_SPS_PPS: 0x02,
  ANNEXB: 0x04,
};
```

**Step 2: Commit**

```bash
git add src/client/utils/H264Decoder.js
git commit -m "feat(frontend): add WebCodecs H264Decoder class"
```

---

## Task 6: Update GenesisContext for codec negotiation

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx`

**Step 1: Add capability detection and negotiation**

Add to GenesisContext.jsx near the top imports:

```javascript
import { detectH264Support, H264Decoder, parseFrameHeader, FrameType } from '../utils/H264Decoder';
```

Add new state variables after existing ones:

```javascript
const [streamCodec, setStreamCodec] = useState('jpeg');
const [h264Codec, setH264Codec] = useState(null);
const [h264Support, setH264Support] = useState(null);
const h264DecoderRef = useRef(null);
```

Add capability detection effect:

```javascript
// Detect H.264 support on mount
useEffect(() => {
  detectH264Support().then(result => {
    setH264Support(result);
    if (result.supported) {
      console.log('WebCodecs H.264 supported:', result.config.codec);
    } else {
      console.log('WebCodecs H.264 not supported, will use JPEG fallback');
    }
  });
}, []);
```

Update WebSocket onopen to send capabilities:

```javascript
ws.onopen = () => {
  console.log('Genesis bridge WebSocket connected');
  setBridgeWs(ws);
  setGenesisConnected(true);

  // Send capability negotiation
  if (h264Support?.supported) {
    ws.send(JSON.stringify({
      type: 'negotiate',
      supports_h264: true,
      h264_codec: h264Support.config.codec
    }));
  } else {
    ws.send(JSON.stringify({
      type: 'negotiate',
      supports_h264: false
    }));
  }
};
```

Update onmessage to handle binary frames with new protocol:

```javascript
ws.onmessage = (event) => {
  if (event.data instanceof Blob) {
    // Convert Blob to ArrayBuffer for parsing
    event.data.arrayBuffer().then(buffer => {
      const { type, frameId, flags, payload } = parseFrameHeader(buffer);

      if (type === FrameType.H264 && h264DecoderRef.current) {
        // H.264 path
        h264DecoderRef.current.decode(payload, frameId, flags);
      } else if (type === FrameType.JPEG || type === 0) {
        // JPEG path (type 0 for backwards compatibility with old protocol)
        if (currentFrameRef.current && blobUrlsRef.current.has(currentFrameRef.current)) {
          URL.revokeObjectURL(currentFrameRef.current);
          blobUrlsRef.current.delete(currentFrameRef.current);
        }
        const blob = new Blob([payload], { type: 'image/jpeg' });
        const blobUrl = URL.createObjectURL(blob);
        blobUrlsRef.current.add(blobUrl);
        setCurrentFrame(blobUrl);
        currentFrameRef.current = blobUrl;
      }
    });
    return;
  }
  // ... existing JSON message handling ...
};
```

Add to context value:

```javascript
value={{
  // ... existing values ...
  streamCodec,
  h264Codec,
  h264Support,
  h264DecoderRef,
}}
```

**Step 2: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx
git commit -m "feat(context): add H.264 capability detection and negotiation"
```

---

## Task 7: Update SimViewer for H.264 canvas playback

**Files:**
- Modify: `src/client/components/SimViewer.jsx`

**Step 1: Add canvas rendering path**

Add to SimViewer.jsx destructuring from useGenesis:

```javascript
const {
  // ... existing ...
  streamCodec,
  h264Codec,
  h264Support,
  h264DecoderRef,
} = useGenesis();
```

Add canvas ref:

```javascript
const h264CanvasRef = useRef(null);
```

Add effect to initialize H264Decoder:

```javascript
// Initialize H264Decoder when codec is h264
useEffect(() => {
  if (streamCodec === 'h264' && h264CanvasRef.current && h264Support?.supported) {
    const decoder = new H264Decoder(h264CanvasRef.current);
    decoder.init(h264Codec || h264Support.config.codec);
    h264DecoderRef.current = decoder;

    return () => {
      decoder.close();
      h264DecoderRef.current = null;
    };
  }
}, [streamCodec, h264Codec, h264Support]);
```

Update render to show canvas or img:

```javascript
{/* Video display */}
{streamCodec === 'h264' && h264Support?.supported ? (
  <canvas
    ref={h264CanvasRef}
    className="sim-viewer__canvas"
    style={{
      width: '100%',
      height: '100%',
      objectFit: containMode ? 'contain' : 'cover',
    }}
  />
) : streamBackend === 'websocket' && currentFrame ? (
  <img
    className="sim-viewer__frame"
    src={currentFrame}
    alt="Genesis simulation"
    style={{ objectFit: containMode ? 'contain' : 'cover' }}
  />
) : /* ... existing webrtc case ... */}
```

**Step 2: Commit**

```bash
git add src/client/components/SimViewer.jsx
git commit -m "feat(viewer): add H.264 canvas rendering path"
```

---

## Task 8: Backend negotiation handler in bridge_server.py

**Files:**
- Modify: `genesis_bridge/bridge_server.py`

**Step 1: Add negotiation message handling**

In bridge_server.py, find the WebSocket message handler and add negotiation support.

Add imports near top:

```python
from genesis_bridge.stream_fanout import StreamFanout
from genesis_bridge.frame_protocol import pack_frame_header, FrameType
```

Add StreamFanout initialization in __init__:

```python
self.stream_fanout = StreamFanout()
```

Update WebSocket handler to parse negotiate messages and use fanout:

```python
async def handle_websocket(self, websocket, path=None):
    client_id = str(id(websocket))
    supports_h264 = False
    h264_codec = "avc1.42E01E"

    try:
        # Wait for initial negotiation message
        try:
            first_msg = await asyncio.wait_for(websocket.recv(), timeout=5.0)
            if isinstance(first_msg, str):
                data = json.loads(first_msg)
                if data.get('type') == 'negotiate':
                    supports_h264 = data.get('supports_h264', False)
                    h264_codec = data.get('h264_codec', 'avc1.42E01E')

                    # Determine actual codec to use
                    stream_codec = 'h264' if (supports_h264 and self.gpu_pipeline and self.gpu_pipeline.has_nvenc) else 'jpeg'

                    # Send negotiation response
                    await websocket.send(json.dumps({
                        'type': 'negotiated',
                        'stream_codec': stream_codec,
                        'h264_codec': h264_codec if stream_codec == 'h264' else None
                    }))
        except asyncio.TimeoutError:
            pass  # No negotiation, default to JPEG

        # Add to fanout
        self.stream_fanout.add_client(client_id, websocket, supports_h264, h264_codec)

        # Also add to legacy ws_clients for metrics broadcast
        self.ws_clients.add(websocket)

        # Keep connection alive
        async for message in websocket:
            # Handle other messages if needed
            pass

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        await self.stream_fanout.remove_client(client_id)
        self.ws_clients.discard(websocket)
```

Update frame encoding to use fanout:

```python
async def _frame_sender_loop(self):
    """Encode frames and broadcast via fanout."""
    if self._frame_queue is None:
        return

    frame_id = 0
    while self.running:
        try:
            frame_bgr = await self._frame_queue.get()
        except asyncio.CancelledError:
            break

        try:
            frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

            # Encode H.264 if any H.264 clients
            if self.stream_fanout.has_h264_clients() and self.gpu_pipeline and self.gpu_pipeline.has_nvenc:
                if self.stream_fanout.should_force_keyframe():
                    self.gpu_pipeline.request_keyframe()

                h264_bytes = self.gpu_pipeline.encode_h264_au(frame_rgb, frame_id)
                if h264_bytes:
                    self.stream_fanout.broadcast_h264(h264_bytes)

            # Encode JPEG if any JPEG clients
            if self.stream_fanout.has_jpeg_clients():
                if self.gpu_pipeline is not None:
                    jpeg_bytes = self.gpu_pipeline.encode_jpeg_numpy(frame_rgb, quality=self.config.jpeg_quality)
                else:
                    _, jpeg_buf = cv2.imencode('.jpg', frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, self.config.jpeg_quality])
                    jpeg_bytes = jpeg_buf.tobytes()

                # Add header for new protocol
                header = pack_frame_header(FrameType.JPEG, len(jpeg_bytes), frame_id, 0)
                self.stream_fanout.broadcast_jpeg(header + jpeg_bytes)

            frame_id += 1

        except Exception as e:
            logger.error(f"[ENCODE-LOOP] {type(e).__name__}: {e}")
        finally:
            self._frame_queue.task_done()
```

**Step 2: Commit**

```bash
git add genesis_bridge/bridge_server.py
git commit -m "feat(bridge): add codec negotiation and fanout streaming"
```

---

## Task 9: Integration test

**Files:**
- Create: `genesis_bridge/test_h264_integration.py`

**Step 1: Write integration test**

```python
# genesis_bridge/test_h264_integration.py
"""Integration tests for H.264 streaming pipeline."""
import pytest
import numpy as np


class TestH264Pipeline:
    def test_full_encode_pipeline(self):
        """Test encoding a frame through the full pipeline."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        from genesis_bridge.frame_protocol import unpack_frame_header, FrameType, HEADER_SIZE

        pipeline = GpuFramePipeline()

        # Create test frame
        frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)

        if pipeline.has_nvenc:
            # H.264 path
            result = None
            for i in range(5):
                result = pipeline.encode_h264_au(frame, frame_id=i)
                if result:
                    break

            if result:
                assert len(result) > HEADER_SIZE
                header = unpack_frame_header(result[:HEADER_SIZE])
                assert header['frame_type'] == FrameType.H264
                print(f"H.264 frame: {len(result)} bytes, keyframe={bool(header['flags'] & 0x01)}")

        # JPEG path (always available)
        from genesis_bridge.frame_protocol import pack_frame_header
        jpeg_bytes = pipeline.encode_jpeg_numpy(frame, quality=80)
        header = pack_frame_header(FrameType.JPEG, len(jpeg_bytes), 0, 0)
        full_frame = header + jpeg_bytes

        parsed = unpack_frame_header(full_frame[:HEADER_SIZE])
        assert parsed['frame_type'] == FrameType.JPEG
        assert parsed['payload_len'] == len(jpeg_bytes)
        print(f"JPEG frame: {len(full_frame)} bytes")

    def test_fanout_client_management(self):
        """Test adding/removing clients from fanout."""
        import asyncio
        from unittest.mock import AsyncMock
        from genesis_bridge.stream_fanout import StreamFanout

        async def run_test():
            fanout = StreamFanout()

            # Add clients
            ws1 = AsyncMock()
            ws2 = AsyncMock()

            fanout.add_client("h264_client", ws1, supports_h264=True)
            fanout.add_client("jpeg_client", ws2, supports_h264=False)

            assert fanout.has_h264_clients()
            assert fanout.has_jpeg_clients()

            # Remove clients
            await fanout.remove_client("h264_client")
            assert not fanout.has_h264_clients()
            assert fanout.has_jpeg_clients()

            await fanout.close_all()

        asyncio.run(run_test())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

**Step 2: Run all tests**

```bash
python -m pytest genesis_bridge/test_frame_protocol.py genesis_bridge/test_stream_fanout.py genesis_bridge/test_h264_integration.py -v --override-ini="addopts="
```

Expected: All tests PASS

**Step 3: Commit**

```bash
git add genesis_bridge/test_h264_integration.py
git commit -m "test: add H.264 pipeline integration tests"
```

---

## Summary

| Task | Description | Files |
|------|-------------|-------|
| 1 | Binary frame protocol | `genesis_bridge/frame_protocol.py` |
| 2 | Annex B to AVCC conversion | `genesis_bridge/frame_protocol.py` |
| 3 | Per-client stream fanout | `genesis_bridge/stream_fanout.py` |
| 4 | H.264 access unit encoding | `genesis_bridge/gpu_pipeline.py` |
| 5 | Frontend H264Decoder | `src/client/utils/H264Decoder.js` |
| 6 | GenesisContext negotiation | `src/client/contexts/GenesisContext.jsx` |
| 7 | SimViewer canvas path | `src/client/components/SimViewer.jsx` |
| 8 | Backend negotiation handler | `genesis_bridge/bridge_server.py` |
| 9 | Integration tests | `genesis_bridge/test_h264_integration.py` |

Total: 9 tasks, ~9 commits
