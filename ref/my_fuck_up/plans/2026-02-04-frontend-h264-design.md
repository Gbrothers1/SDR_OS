# Frontend H.264 WebCodecs Playback Design

**Date:** 2026-02-04
**Status:** Draft
**Branch:** TBD

## Overview

Add H.264 video playback to the frontend using WebCodecs API, with automatic fallback to JPEG for unsupported browsers (Firefox). This completes the GPU render pipeline by enabling low-latency H.264 streaming over WebSocket.

## Goals

- Low-latency H.264 playback via WebCodecs (~40ms less than MSE)
- Automatic JPEG fallback for browsers without WebCodecs support
- Per-client codec negotiation (H.264 vs JPEG)
- Backend fanout to multiple codec groups without duplicate encoding overhead

## Non-Goals

- MSE/fMP4 fallback (too much latency for teleop)
- Firefox WebCodecs polyfill
- Audio streaming

---

## Architecture

### Capability Negotiation

```
Frontend                              Backend (bridge_server.py)
   |                                        |
   |--- genesis_connect ------------------->|
   |    { supports_h264: true,              |
   |      h264_codec: 'avc1.4D401E' }       |
   |                                        |
   |<-- genesis_status --------------------|
   |    { stream_codec: 'h264'|'jpeg',      |
   |      h264_codec: 'avc1.4D401E' }       |
   |                                        |
   |<== binary frame stream ===============|
```

### Frontend Capability Detection

```javascript
const detectH264Support = async () => {
  if (!window.VideoDecoder) return { supported: false };

  // Prefer Baseline (widest HW support), fallback to Main/High
  const codecs = ['avc1.42E01E', 'avc1.4D401E', 'avc1.64001E'];

  for (const codec of codecs) {
    const cfg = { codec, hardwareAcceleration: 'prefer-hardware' };
    try {
      const res = await VideoDecoder.isConfigSupported(cfg);
      if (res.supported) return { supported: true, config: cfg };
    } catch {
      continue;
    }
  }
  return { supported: false };
};
```

---

## Binary Frame Protocol

### Header (12 bytes, big-endian)

| Offset | Size | Field       | Description                                    |
|--------|------|-------------|------------------------------------------------|
| 0      | 1    | version     | Protocol version (0x01)                        |
| 1      | 1    | type        | 0x00=JPEG, 0x01=H.264                          |
| 2      | 4    | payload_len | Payload size (u32 BE)                          |
| 6      | 4    | frame_id    | Monotonic frame counter (u32 BE)               |
| 10     | 2    | flags       | Bit 0: keyframe, Bit 1: has_sps_pps, Bit 2: annexb |

### Payload

- **JPEG (type=0x00):** Raw JPEG bytes (unchanged from current)
- **H.264 (type=0x01):** One access unit (all NAL units for one frame, AVCC format)

### Flags

| Bit | Name        | Description                              |
|-----|-------------|------------------------------------------|
| 0   | keyframe    | Access unit contains IDR                 |
| 1   | has_sps_pps | Access unit includes SPS/PPS headers     |
| 2   | annexb      | Payload has Annex B start codes (0 = AVCC) |

Backend sends AVCC format (annexb=0) to keep frontend simple.

---

## Backend Changes

### Stream Fanout Architecture

```python
class ClientStream:
    websocket: WebSocket
    supports_h264: bool
    h264_codec: str  # 'avc1.42E01E', etc.
    queue: asyncio.Queue  # maxsize=2
    send_task: asyncio.Task

    async def _send_loop(self):
        try:
            while True:
                frame_bytes = await self.queue.get()
                await self.websocket.send(frame_bytes)
        except Exception:
            pass
        finally:
            await self.websocket.close()

    def enqueue(self, frame_bytes: bytes):
        """Drop oldest if full, always keep newest."""
        while self.queue.full():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        self.queue.put_nowait(frame_bytes)


class StreamFanout:
    h264_clients: dict[str, ClientStream]
    jpeg_clients: dict[str, ClientStream]

    async def broadcast_frame(self, rgb_tensor, frame_id):
        # H.264 fanout
        if self.h264_clients and self.gpu_pipeline.has_nvenc:
            h264_bytes = self._encode_h264_au(rgb_tensor, frame_id)
            if h264_bytes is not None:
                for cid, client in list(self.h264_clients.items()):
                    if client.send_task.done():
                        del self.h264_clients[cid]
                    else:
                        client.enqueue(h264_bytes)

        # JPEG fanout
        if self.jpeg_clients:
            jpeg_bytes = self._encode_jpeg_frame(rgb_tensor, frame_id)
            for cid, client in list(self.jpeg_clients.items()):
                if client.send_task.done():
                    del self.jpeg_clients[cid]
                else:
                    client.enqueue(jpeg_bytes)
```

### NVENC Encoder Configuration

```python
CODEC_TO_PROFILE = {
    'avc1.42E01E': 'baseline',
    'avc1.4D401E': 'main',
    'avc1.64001E': 'high',
}

def _init_h264_encoder(self, width, height, codec_string):
    profile = CODEC_TO_PROFILE.get(codec_string, 'baseline')

    encoder = av.CodecContext.create('h264_nvenc', 'w')
    encoder.width = width
    encoder.height = height
    encoder.pix_fmt = 'yuv420p'
    encoder.bit_rate = 8_000_000
    encoder.options = {
        'profile': profile,
        'level': '3.0',
        'preset': 'p1',
        'tune': 'll',
        'rc': 'cbr',
        'maxrate': '8000000',
        'bufsize': '4000000',
        'bf': '0',
        'rc-lookahead': '0',
        'g': '60',
        'repeat-headers': '1',
    }
    encoder.open()
    return encoder
```

### Access Unit Encoding (Annex B → AVCC)

```python
def _encode_h264_au(self, rgb_tensor, frame_id) -> bytes | None:
    frame_np = rgb_tensor.cpu().numpy()
    av_frame = av.VideoFrame.from_ndarray(frame_np, format='rgb24')
    av_frame.pts = frame_id

    # Encode may yield 0, 1, or N packets
    packets = self._h264_encoder.encode(av_frame)

    if not packets:
        return None  # Buffered, no output yet

    # Concatenate all packets, convert Annex B → AVCC
    annexb_bytes = b''.join(bytes(p) for p in packets)
    avcc_bytes = self._annexb_to_avcc(annexb_bytes)

    # Detect keyframe
    is_keyframe = packets[0].is_keyframe
    has_sps_pps = is_keyframe  # repeat-headers ensures this

    # Build header (annexb=0 since we converted to AVCC)
    flags = (is_keyframe << 0) | (has_sps_pps << 1) | (0 << 2)
    header = struct.pack('>BBIIH', 0x01, 0x01, len(avcc_bytes), frame_id, flags)

    return header + avcc_bytes

def _annexb_to_avcc(self, annexb: bytes) -> bytes:
    """Convert Annex B (start codes) to AVCC (length-prefixed)."""
    # Scan for 00 00 00 01 or 00 00 01 start codes
    # Replace with 4-byte big-endian NAL length
    ...
```

### Force IDR on Client Connect

```python
def on_h264_client_connect(self, client_id):
    self._force_keyframe = True

def _encode_h264_au(self, ...):
    if self._force_keyframe:
        # Insert keyframe request into encoder
        self._h264_encoder.options['force_key_frames'] = 'expr:1'
        self._force_keyframe = False
```

---

## Frontend Changes

### H264Decoder Class

```javascript
// src/client/utils/H264Decoder.js
export class H264Decoder {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.decoder = null;
    this.waitingForKeyframe = true;
    this.maxQueueSize = 3;
  }

  init(codec = 'avc1.42E01E') {
    this.decoder = new VideoDecoder({
      output: (frame) => {
        // Resize canvas to match frame
        if (this.canvas.width !== frame.displayWidth ||
            this.canvas.height !== frame.displayHeight) {
          this.canvas.width = frame.displayWidth;
          this.canvas.height = frame.displayHeight;
        }
        this.ctx.drawImage(frame, 0, 0);
        frame.close();
      },
      error: (e) => console.error('Decoder error:', e),
    });

    this.decoder.configure({
      codec,
      hardwareAcceleration: 'prefer-hardware',
    });
  }

  decode(payload, frameId, flags) {
    const isKeyframe = (flags & 0x01) !== 0;

    // Wait for keyframe to start
    if (this.waitingForKeyframe) {
      if (!isKeyframe) return;
      this.waitingForKeyframe = false;
    }

    // Backpressure: drop delta frames if queue backed up
    if (this.decoder.decodeQueueSize > this.maxQueueSize) {
      if (!isKeyframe) return;
    }

    this.decoder.decode(new EncodedVideoChunk({
      type: isKeyframe ? 'key' : 'delta',
      timestamp: frameId * 16666,  // Monotonic, ~60fps in microseconds
      data: payload,
    }));
  }

  reset() {
    this.waitingForKeyframe = true;
    this.decoder?.reset();
  }

  close() {
    this.decoder?.close();
  }
}
```

### Binary Frame Parsing

```javascript
// In GenesisContext.jsx or SimViewer.jsx
const handleBinaryFrame = (data) => {
  const view = new DataView(data);
  const version = view.getUint8(0);
  const type = view.getUint8(1);
  const payloadLen = view.getUint32(2, false);  // Explicit big-endian
  const frameId = view.getUint32(6, false);
  const flags = view.getUint16(10, false);
  const payload = data.slice(12, 12 + payloadLen);

  if (type === 0x01 && h264DecoderRef.current) {
    // H.264 path
    h264DecoderRef.current.decode(payload, frameId, flags);
  } else if (type === 0x00) {
    // JPEG path: create blob URL (existing code)
    const blob = new Blob([payload], { type: 'image/jpeg' });
    setCurrentFrame(URL.createObjectURL(blob));
  }
};
```

### SimViewer Integration

```javascript
// SimViewer.jsx additions
const h264DecoderRef = useRef(null);
const canvasRef = useRef(null);

useEffect(() => {
  if (streamCodec === 'h264' && canvasRef.current) {
    h264DecoderRef.current = new H264Decoder(canvasRef.current);
    h264DecoderRef.current.init(h264Codec);
  }
  return () => {
    h264DecoderRef.current?.close();
    h264DecoderRef.current = null;
  };
}, [streamCodec, h264Codec]);

// In render:
{streamCodec === 'h264' ? (
  <canvas ref={canvasRef} className="sim-viewer__canvas" />
) : (
  <img src={currentFrame} ... />
)}
```

---

## File Structure

### New Files

```
src/client/utils/H264Decoder.js       # WebCodecs decoder class
genesis_bridge/stream_fanout.py       # Per-client queues, codec fanout
```

### Modified Files

```
genesis_bridge/bridge_server.py       # Wire up fanout, negotiation
genesis_bridge/gpu_pipeline.py        # AVCC output, force_keyframe()
src/client/contexts/GenesisContext.jsx    # Capability detection, negotiation
src/client/components/SimViewer.jsx       # H264Decoder + canvas path
src/client/contexts/SettingsContext.jsx   # Add h264 codec preference
```

---

## Testing Strategy

1. **Unit tests:**
   - Binary header pack/unpack roundtrip
   - Annex B → AVCC conversion
   - H264Decoder mock (jsdom with VideoDecoder stub)

2. **Integration tests:**
   - Capability negotiation handshake
   - Fanout to mixed H.264/JPEG clients
   - Backpressure: slow client doesn't block fast client

3. **Manual tests:**
   - Chrome: H.264 playback, verify low latency
   - Firefox: JPEG fallback, verify no errors
   - Safari: H.264 playback (if WebCodecs available)
   - New client mid-stream: receives IDR, starts cleanly

---

## Rollout

1. **Phase 1:** Backend protocol + fanout (no frontend changes)
2. **Phase 2:** Frontend H264Decoder + capability detection
3. **Phase 3:** Settings UI for codec preference (optional)

---

## Browser Support

| Browser | WebCodecs H.264 | Fallback |
|---------|-----------------|----------|
| Chrome 94+ | Yes | - |
| Edge 94+ | Yes | - |
| Safari 16.4+ | Yes | - |
| Firefox | No | JPEG |
| Mobile Chrome | Yes | - |
| Mobile Safari | Yes (iOS 16.4+) | JPEG |
