import React, { useCallback, useEffect, useRef, useState } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/BlendPanel.css';

const ALPHA_SPARK_MAX = 30;

const AlphaSparkline = ({ history, width = 160, height = 32 }) => {
  if (!history || history.length < 2) return null;

  const len = history.length;
  const stepX = width / (ALPHA_SPARK_MAX - 1);
  const offsetX = (ALPHA_SPARK_MAX - len) * stepX;

  const points = history.map((v, i) =>
    `${offsetX + i * stepX},${height - v * (height - 2) - 1}`
  ).join(' ');

  const fillPoints = `${offsetX},${height} ${points} ${offsetX + (len - 1) * stepX},${height}`;

  const latest = history[len - 1];
  let stroke, fill;
  if (latest < 0.3) {
    stroke = 'rgba(46, 230, 166, 0.9)';
    fill = 'rgba(46, 230, 166, 0.12)';
  } else if (latest < 0.7) {
    stroke = 'rgba(255, 180, 84, 0.9)';
    fill = 'rgba(255, 180, 84, 0.12)';
  } else {
    stroke = 'rgba(83, 164, 255, 0.9)';
    fill = 'rgba(83, 164, 255, 0.12)';
  }

  return (
    <svg className="blend-panel__sparkline" width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      <polygon points={fillPoints} fill={fill} />
      <polyline points={points} fill="none" stroke={stroke} strokeWidth="1.5" strokeLinejoin="round" strokeLinecap="round" />
      <circle
        cx={offsetX + (len - 1) * stepX}
        cy={height - latest * (height - 2) - 1}
        r="2" fill={stroke}
      />
    </svg>
  );
};

const SAFETY_FLAG_LABELS = {
  vel_clamped: 'VEL',
  accel_clamped: 'ACCEL',
  jerk_clamped: 'JERK',
  auto_stopped: 'STOP',
  clip_clamped: 'CLIP',
};

const BlendPanel = ({ onExpandChange }) => {
  const {
    blendAlpha,
    setAlpha,
    deadmanActive,
    actorTag,
    trainingMetrics,
    policyLoaded,
    policyCheckpoint,
    listPolicies,
    loadPolicy,
    policyList,
    policyLoadStatus,
    estop,
  } = useGenesis();

  const alphaHistoryRef = useRef([]);
  const [alphaHistory, setAlphaHistory] = useState([]);

  // Accumulate alpha history for sparkline
  useEffect(() => {
    const buf = alphaHistoryRef.current;
    buf.push(blendAlpha);
    if (buf.length > ALPHA_SPARK_MAX) buf.shift();
    setAlphaHistory([...buf]);
  }, [blendAlpha]);

  const m = trainingMetrics || {};
  const confidence = m.confidence;
  const confidenceGating = confidence != null && confidence < 0.5;
  const safetyFlags = m.safety_flags || {};
  const activeFlags = Object.entries(safetyFlags)
    .filter(([, v]) => v)
    .map(([k]) => SAFETY_FLAG_LABELS[k] || k);
  const anomalyCount = m.anomaly_count ?? 0;
  const consecutiveAnomalies = m.consecutive_anomalies ?? 0;

  const step = m.step_count ?? 0;
  const sps = m.fps != null ? Math.round(m.fps) : 0;
  const reward = m.total_reward?.toFixed(2) ?? '0.00';
  const memMb = m.memory_mb;

  // Alpha color class
  const alphaClass = blendAlpha < 0.3 ? 'human' : blendAlpha < 0.7 ? 'blend' : 'policy';

  const handleAlphaChange = useCallback((e) => {
    setAlpha(parseFloat(e.target.value));
  }, [setAlpha]);

  const handleAlphaBump = useCallback((delta) => {
    setAlpha(Math.max(0, Math.min(1, blendAlpha + delta)));
  }, [setAlpha, blendAlpha]);

  const handleAlphaReset = useCallback(() => {
    setAlpha(0);
  }, [setAlpha]);

  // Actor display
  const actorDisplay = actorTag?.startsWith('override_')
    ? actorTag.replace('override_', '').toUpperCase()
    : (actorTag || 'teleop').toUpperCase();
  const isOverride = actorTag?.startsWith('override_');

  return (
    <div className="blend-panel">
      {/* Alpha Control — Primary */}
      <div className="blend-panel__section blend-panel__alpha-section">
        <div className="blend-panel__section-heading">Authority</div>
        <div className="blend-panel__alpha-display">
          <span className="blend-panel__alpha-label">{'\u03B1'}</span>
          <span className={`blend-panel__alpha-value blend-panel__alpha-value--${alphaClass}`}>
            {blendAlpha.toFixed(2)}
          </span>
        </div>
        <input
          type="range"
          min="0"
          max="1"
          step="0.05"
          value={blendAlpha}
          onChange={handleAlphaChange}
          className={`blend-panel__slider blend-panel__slider--${alphaClass}`}
          title="Policy blend: 0 = human, 1 = policy"
        />
        <div className="blend-panel__slider-labels">
          <span>HUMAN</span>
          <span>POLICY</span>
        </div>
        <div className="blend-panel__quick-btns">
          <button className="blend-panel__quick-btn" onClick={() => handleAlphaBump(-0.1)}>-0.1</button>
          <button className="blend-panel__quick-btn blend-panel__quick-btn--reset" onClick={handleAlphaReset}>RESET</button>
          <button className="blend-panel__quick-btn" onClick={() => handleAlphaBump(0.1)}>+0.1</button>
        </div>
      </div>

      {/* Actor & Deadman */}
      <div className="blend-panel__section">
        <div className="blend-panel__section-heading">Status</div>
        <div className="blend-panel__row">
          <span className="blend-panel__key">Actor</span>
          <span className={`blend-panel__actor-badge blend-panel__actor-badge--${isOverride ? 'override' : alphaClass}`}>
            {actorDisplay}
          </span>
        </div>
        <div className="blend-panel__row">
          <span className="blend-panel__key">Deadman (L1)</span>
          <span className={`blend-panel__deadman ${deadmanActive ? 'blend-panel__deadman--active' : 'blend-panel__deadman--released'}`}>
            {deadmanActive ? 'HELD' : 'RELEASED'}
          </span>
        </div>
        <div className="blend-panel__row">
          <span className="blend-panel__key">Policy</span>
          <span className="blend-panel__val">
            <span className={`blend-panel__dot blend-panel__dot--${policyLoaded ? 'on' : 'off'}`} />
            {policyLoaded ? (policyCheckpoint || 'Loaded') : 'None'}
          </span>
        </div>
      </div>

      {/* Confidence */}
      {confidence != null && (
        <div className="blend-panel__section">
          <div className="blend-panel__section-heading">Confidence</div>
          <div className="blend-panel__row">
            <span className="blend-panel__key">Score</span>
            <span className={`blend-panel__val ${confidence < 0.5 ? 'blend-panel__val--warn' : ''}`}>
              {(confidence * 100).toFixed(0)}%
            </span>
          </div>
          <div className="blend-panel__bar-wrap">
            <div
              className={`blend-panel__bar ${confidence < 0.5 ? 'blend-panel__bar--warn' : ''}`}
              style={{ width: `${Math.min(confidence * 100, 100)}%` }}
            />
          </div>
          {confidenceGating && (
            <div className="blend-panel__gating-warn">CONFIDENCE GATING ACTIVE</div>
          )}
        </div>
      )}

      {/* Safety */}
      <div className="blend-panel__section">
        <div className="blend-panel__section-heading">Safety</div>
        {activeFlags.length > 0 ? (
          <div className="blend-panel__flags">
            {activeFlags.map(flag => (
              <span key={flag} className="blend-panel__flag-pill blend-panel__flag-pill--active">{flag}</span>
            ))}
          </div>
        ) : (
          <div className="blend-panel__flags">
            {Object.values(SAFETY_FLAG_LABELS).map(flag => (
              <span key={flag} className="blend-panel__flag-pill">{flag}</span>
            ))}
          </div>
        )}
        <div className="blend-panel__row">
          <span className="blend-panel__key">Anomalies</span>
          <span className={`blend-panel__val ${anomalyCount > 0 ? 'blend-panel__val--warn' : ''}`}>
            {anomalyCount}
          </span>
        </div>
        {consecutiveAnomalies > 0 && (
          <div className="blend-panel__row">
            <span className="blend-panel__key">Consecutive</span>
            <span className={`blend-panel__val ${consecutiveAnomalies >= 2 ? 'blend-panel__val--warn' : ''}`}>
              {consecutiveAnomalies} / 3
            </span>
          </div>
        )}
      </div>

      {/* Alpha History */}
      <div className="blend-panel__section">
        <div className="blend-panel__section-heading">Blend History</div>
        <div className="blend-panel__graph">
          <AlphaSparkline history={alphaHistory} width={260} height={36} />
        </div>
      </div>

      {/* Metrics */}
      <div className="blend-panel__section">
        <div className="blend-panel__section-heading">Metrics</div>
        <div className="blend-panel__row">
          <span className="blend-panel__key">Step</span>
          <span className="blend-panel__val">{step.toLocaleString()}</span>
        </div>
        <div className="blend-panel__row">
          <span className="blend-panel__key">SPS</span>
          <span className="blend-panel__val">{sps}</span>
        </div>
        <div className="blend-panel__row">
          <span className="blend-panel__key">Reward</span>
          <span className="blend-panel__val">{reward}</span>
        </div>
        {memMb != null && (
          <div className="blend-panel__row">
            <span className="blend-panel__key">Memory</span>
            <span className="blend-panel__val">{Math.round(memMb)} MB</span>
          </div>
        )}
      </div>
    </div>
  );
};

export default BlendPanel;
