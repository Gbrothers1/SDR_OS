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
      optimizeForLatency: true,
    });

    this.waitingForKeyframe = true;
    console.log(`H264Decoder initialized with codec: ${codec}`);
  }

  /**
   * Decode a raw Annex-B H.264 access unit.
   * @param {Uint8Array} payload - Raw Annex-B NAL units
   * @param {boolean} isKeyframe - Whether this frame is a keyframe (IDR)
   */
  decode(payload, isKeyframe) {
    if (!this.decoder || this.decoder.state === 'closed') {
      return;
    }

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
        timestamp: this.frameCount * 16666, // ~60fps in microseconds
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

  // Try codecs in order of preference (42C01F = NVENC baseline L3.1)
  const codecs = ['avc1.42C01F', 'avc1.42E01E', 'avc1.4D401E', 'avc1.64001E'];

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

// Transport server wire format message types (first byte of WS binary frame)
export const MsgType = {
  VIDEO: 0x01,
  TELEMETRY: 0x02,
  SIGNALING: 0x03,
  COMMAND: 0x04,
};

// Codec types in frame header
export const CodecType = {
  H264: 1,
  HEVC: 2,
  JPEG: 3,
};

/**
 * Parse the 32-byte LE video frame header (after the 0x01 type byte).
 * Wire format: [0x01][32-byte header][Annex-B payload]
 *
 * Header layout (little-endian):
 *   [0:8]   frame_id    u64
 *   [8:16]  frame_seq   u64
 *   [16:20] size        u32   payload size in bytes
 *   [20:22] flags       u16   0x0001 = keyframe
 *   [22:24] codec       u16   1 = H.264, 2 = HEVC
 *   [24:28] crc32       i32
 *   [28:32] reserved    u32
 *
 * @param {DataView} view - DataView over the full WS message (including type byte)
 * @returns {{frameId: number, frameSeq: number, size: number, flags: number, isKeyframe: boolean, codec: number, payload: Uint8Array}}
 */
function readU64LE(view, offset) {
  if (typeof view.getBigUint64 === 'function') {
    return Number(view.getBigUint64(offset, true));
  }
  // Safari fallback: compose from two u32s
  const lo = view.getUint32(offset, true);
  const hi = view.getUint32(offset + 4, true);
  return hi * 0x100000000 + lo;
}

export function parseVideoHeader(view) {
  const buffer = view.buffer;
  const frameId = readU64LE(view, 1);
  const frameSeq = readU64LE(view, 9);
  const size = view.getUint32(17, true);
  const flags = view.getUint16(21, true);
  const codec = view.getUint16(23, true);
  const isKeyframe = (flags & 0x0001) !== 0;
  const payload = new Uint8Array(buffer, 33, size);

  return { frameId, frameSeq, size, flags, isKeyframe, codec, payload };
}
