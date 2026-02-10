import React from 'react';
import '../styles/StreamStats.css';

const StreamStats = ({
  fps = 0,
  frameId = 0,
  lastKeyframeTime = null,
  wsBytesPerSec = 0,
  rtcStats = null,
  mode = 'ws',
  decoderQueueSize = 0,
  maxQueueSize = 3,
}) => {
  // FPS color class
  const fpsClass = fps >= 50
    ? 'stream-stats__val--fps-good'
    : fps >= 25
      ? 'stream-stats__val--fps-warn'
      : 'stream-stats__val--fps-bad';

  // Keyframe age
  const kfAge = lastKeyframeTime
    ? ((Date.now() - lastKeyframeTime) / 1000).toFixed(1)
    : null;
  const kfDotClass = kfAge !== null && parseFloat(kfAge) < 5
    ? 'stream-stats__kf-dot--ok'
    : 'stream-stats__kf-dot--waiting';

  // WS throughput — adaptive scale
  let wsThroughput, wsUnit;
  if (wsBytesPerSec >= 1024 * 1024) {
    wsThroughput = (wsBytesPerSec / (1024 * 1024)).toFixed(1);
    wsUnit = 'MB/s';
  } else if (wsBytesPerSec >= 1024) {
    wsThroughput = (wsBytesPerSec / 1024).toFixed(0);
    wsUnit = 'KB/s';
  } else {
    wsThroughput = Math.round(wsBytesPerSec);
    wsUnit = 'B/s';
  }

  // Mode badge class
  const modeClass = mode === 'rtc'
    ? 'stream-stats__mode--rtc'
    : mode === 'h264'
      ? 'stream-stats__mode--h264'
      : 'stream-stats__mode--ws';

  // Queue segments
  const queueSegs = [];
  for (let i = 0; i < maxQueueSize; i++) {
    const filled = i < decoderQueueSize;
    const overflow = decoderQueueSize > maxQueueSize && i === maxQueueSize - 1;
    queueSegs.push(
      <div
        key={i}
        className={`stream-stats__queue-seg${filled ? ' stream-stats__queue-seg--filled' : ''}${overflow ? ' stream-stats__queue-seg--overflow' : ''}`}
      />
    );
  }

  return (
    <div className="stream-stats">
      {/* Mode badge */}
      <div className="stream-stats__cell">
        <span className={`stream-stats__mode ${modeClass}`}>
          {mode.toUpperCase()}
        </span>
      </div>

      {/* FPS */}
      <div className="stream-stats__cell">
        <span className="stream-stats__label">FPS</span>
        <span className={`stream-stats__val ${fpsClass}`}>{fps}</span>
      </div>

      {/* Frame ID */}
      <div className="stream-stats__cell">
        <span className="stream-stats__label">Frame</span>
        <span className="stream-stats__val">{frameId}</span>
      </div>

      {/* Keyframe indicator */}
      <div className="stream-stats__cell">
        <span className="stream-stats__label">KF</span>
        <span className={`stream-stats__kf-dot ${kfDotClass}`} />
        {kfAge !== null && (
          <span className="stream-stats__val">{kfAge}<span className="stream-stats__unit">s</span></span>
        )}
      </div>

      {/* WS throughput */}
      <div className="stream-stats__cell">
        <span className="stream-stats__label">WS</span>
        <span className="stream-stats__val">{wsThroughput}<span className="stream-stats__unit">{wsUnit}</span></span>
      </div>

      {/* RTC stats (only when WebRTC active) */}
      {rtcStats && (
        <div className="stream-stats__cell">
          <span className="stream-stats__label">RTC</span>
          <span className="stream-stats__val">
            {rtcStats.kbps}<span className="stream-stats__unit">KB/s</span>
          </span>
          <span className="stream-stats__val">
            {rtcStats.rtt}<span className="stream-stats__unit">ms</span>
          </span>
        </div>
      )}

      <div className="stream-stats__spacer" />

      {/* Decoder queue */}
      {mode !== 'rtc' && (
        <div className="stream-stats__cell">
          <span className="stream-stats__label">Q</span>
          <div className="stream-stats__queue-bar">{queueSegs}</div>
        </div>
      )}

      <span className="stream-stats__toggle-hint">S to toggle</span>
    </div>
  );
};

export default StreamStats;
