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
