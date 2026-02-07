import React, { useCallback, useState, useRef, useEffect, useMemo } from 'react';
import { usePhase } from '../contexts/PhaseContext';
import { useGenesis } from '../contexts/GenesisContext';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import '../styles/TrustStrip.css';

const MODES = [
  { key: 'teleop', label: 'TELEM' },
  { key: 'policy', label: 'POLICY' },
  { key: 'blend', label: 'BLEND' },
];

const TREND_ARROWS = {
  rising: '\u2197',
  falling: '\u2198',
  flat: '\u2192',
};

const STAGE_LABELS = {
  teleop: 'TELEOP',
  train: 'TRAIN',
  eval: 'EVAL',
};

const HEALTH_STATUS = {
  ok: 'LIVE',
  degraded: 'WARN',
  dead: 'IDLE',
};

const StageCell = ({ stage, data, collapsed, onClick }) => {
  const cellClass = [
    'stage-cell',
    collapsed && 'stage-cell--collapsed',
  ].filter(Boolean).join(' ');

  const dotClass = [
    'stage-cell__dot',
    `stage-cell__dot--${data.health}`,
    stage === 'teleop' && 'stage-cell__dot--heartbeat',
  ].filter(Boolean).join(' ');

  return (
    <div className={cellClass} onClick={onClick}>
      <span className={dotClass} />
      {!collapsed && (
        <>
          <span className="stage-cell__label">{STAGE_LABELS[stage] || stage}</span>
          <span className={`stage-cell__status stage-cell__status--${data.health}`}>
            {HEALTH_STATUS[data.health] || ''}
          </span>
          {stage === 'teleop' && (
            <span className="stage-cell__signal">
              {data.recording
                ? `${data.recordingStep ?? '?'}/${data.recordingTotal ?? '?'}`
                : data.commandSource || data.actor || ''}
            </span>
          )}
          {stage === 'train' && (
            <>
              <span className={`stage-cell__trend stage-cell__trend--${data.trend}${data.unstable ? ' stage-cell__trend--unstable' : ''}`}>
                {TREND_ARROWS[data.trend] || '\u2192'}
              </span>
              <span className="stage-cell__signal">
                {data.reward != null ? data.reward.toFixed(1) + 'r' : '\u2014'}
              </span>
            </>
          )}
          {stage === 'eval' && (
            <span className={`stage-cell__risk stage-cell__risk--${data.riskLevel || 'nominal'}`}>
              {data.riskLevel === 'nominal' ? 'nominal' :
                data.riskLevel ? `${data.riskLevel}${data.clampCount > 0 ? ` \u00d7${data.clampCount}` : ''}` : 'idle'}
            </span>
          )}
          {data.error && (
            <span className="stage-cell__error" title={data.error}>
              {data.error.length > 30 ? data.error.slice(0, 30) + '...' : data.error}
            </span>
          )}
        </>
      )}
    </div>
  );
};

const SPARK_MAX = 30; // 30 data points (~30s at 1 update/sec)
const SPARK_CEIL = 120; // FPS ceiling for graph scale

const ConnectionSparkline = ({ history, width = 120, height = 24 }) => {
  if (!history || history.length < 2) return null;

  const len = history.length;
  const max = Math.max(SPARK_CEIL, ...history);
  const stepX = width / (SPARK_MAX - 1);
  const offsetX = (SPARK_MAX - len) * stepX;

  // Build polyline points
  const points = history.map((v, i) =>
    `${offsetX + i * stepX},${height - (v / max) * (height - 2) - 1}`
  ).join(' ');

  // Filled polygon (line + close along bottom)
  const fillPoints = `${offsetX},${height} ${points} ${offsetX + (len - 1) * stepX},${height}`;

  // Color based on latest value
  const latest = history[len - 1];
  let stroke, fill;
  if (latest >= 30) {
    stroke = 'rgba(62, 207, 142, 0.9)';
    fill = 'rgba(62, 207, 142, 0.15)';
  } else if (latest >= 15) {
    stroke = 'rgba(240, 180, 41, 0.9)';
    fill = 'rgba(240, 180, 41, 0.15)';
  } else {
    stroke = 'rgba(239, 83, 80, 0.9)';
    fill = 'rgba(239, 83, 80, 0.15)';
  }

  return (
    <svg className="conn-sparkline" width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
      {/* Threshold line at 30 FPS */}
      <line
        x1={0} y1={height - (30 / max) * (height - 2) - 1}
        x2={width} y2={height - (30 / max) * (height - 2) - 1}
        stroke="rgba(255,255,255,0.08)" strokeWidth="1" strokeDasharray="3,3"
      />
      <polygon points={fillPoints} fill={fill} />
      <polyline points={points} fill="none" stroke={stroke} strokeWidth="1.5" strokeLinejoin="round" strokeLinecap="round" />
      {/* Latest value dot */}
      <circle
        cx={offsetX + (len - 1) * stepX}
        cy={height - (latest / max) * (height - 2) - 1}
        r="2" fill={stroke}
      />
    </svg>
  );
};

const SimHoverPopup = ({ streamBackend, bridgeConnected, genesisConnected, visualFps, trainingMetrics, fpsHistory }) => {
  const streamLabel = streamBackend === 'webrtc' ? 'WebRTC' : 'WS';
  const isConnected = bridgeConnected && genesisConnected;
  const sps = trainingMetrics?.fps != null ? Math.round(trainingMetrics.fps) : 0;
  const reward = trainingMetrics?.total_reward?.toFixed(2) || '0.00';

  return (
    <div className="sim-hover-popup">
      <div className="sim-hover-popup__row">
        <span className={`sim-hover-popup__dot sim-hover-popup__dot--${isConnected ? 'on' : 'off'}`} />
        <span className="sim-hover-popup__stream">{streamLabel}</span>
      </div>
      <div className="sim-hover-popup__row">
        <span className="sim-hover-popup__stat">FPS {visualFps}</span>
        <span className="sim-hover-popup__sep" />
        <span className="sim-hover-popup__stat">SPS {sps}</span>
        <span className="sim-hover-popup__sep" />
        <span className="sim-hover-popup__stat">{reward}r</span>
      </div>
      <ConnectionSparkline history={fpsHistory} width={120} height={20} />
    </div>
  );
};

const SimDetailPopup = ({ streamBackend, bridgeConnected, genesisConnected, visualFps, trainingMetrics, envInfo, frameStats, fpsHistory }) => {
  const streamLabel = streamBackend === 'webrtc' ? 'WebRTC' : 'WebSocket';
  const isConnected = bridgeConnected && genesisConnected;
  const sps = trainingMetrics?.fps != null ? Math.round(trainingMetrics.fps) : 0;
  const reward = trainingMetrics?.total_reward?.toFixed(2) || '0.00';

  // Estimate stream bitrate (avg frame size * fps -> bits/sec -> Mibps)
  const bitrateMibps = useMemo(() => {
    const avg = frameStats?.avg_frame_size;
    const fps = frameStats?.frame_rate;
    if (!avg || !fps) return null;
    const bitsPerSec = avg * fps * 8;
    return bitsPerSec / (1024 * 1024);
  }, [frameStats]);

  // Compute speed stats from history
  const avgFps = fpsHistory.length > 0 ? Math.round(fpsHistory.reduce((a, b) => a + b, 0) / fpsHistory.length) : 0;
  const minFps = fpsHistory.length > 0 ? Math.min(...fpsHistory) : 0;
  const maxFps = fpsHistory.length > 0 ? Math.max(...fpsHistory) : 0;
  const jitter = fpsHistory.length > 2
    ? Math.round(Math.sqrt(fpsHistory.reduce((sum, v) => sum + (v - avgFps) ** 2, 0) / fpsHistory.length))
    : 0;

  // Frame delivery stats
  const frameSizeKB = frameStats?.avg_frame_size ? (frameStats.avg_frame_size / 1024).toFixed(1) : null;
  const dropRate = (frameStats?.frames_sent > 0 && frameStats?.frames_failed != null)
    ? ((frameStats.frames_failed / (frameStats.frames_sent + frameStats.frames_failed)) * 100).toFixed(1)
    : null;
  const clients = frameStats?.clients_connected ?? null;

  // Safety / confidence
  const confidence = trainingMetrics?.confidence;
  const safetyFlags = trainingMetrics?.safety_flags;
  const clampedAxes = safetyFlags
    ? Object.entries(safetyFlags).filter(([, v]) => v).map(([k]) => k)
    : [];
  const memMb = trainingMetrics?.memory_mb;

  return (
    <div className="sim-detail-popup">
      {/* Connection */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Connection</div>
        <div className="sim-detail-popup__row">
          <span className={`sim-detail-popup__dot sim-detail-popup__dot--${isConnected ? 'on' : 'off'}`} />
          <span className="sim-detail-popup__label">{isConnected ? 'Connected' : bridgeConnected ? 'Waiting' : 'Disconnected'}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Stream</span>
          <span className="sim-detail-popup__val">{streamLabel}</span>
        </div>
        {bitrateMibps != null && (
          <div className="sim-detail-popup__row">
            <span className="sim-detail-popup__key">Bitrate</span>
            <span className="sim-detail-popup__val">{bitrateMibps.toFixed(1)}Mibps</span>
          </div>
        )}
        {clients != null && (
          <div className="sim-detail-popup__row">
            <span className="sim-detail-popup__key">Clients</span>
            <span className="sim-detail-popup__val">{clients}</span>
          </div>
        )}
      </div>

      {/* Performance */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Performance</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">FPS</span>
          <span className="sim-detail-popup__val">{visualFps}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">SPS</span>
          <span className="sim-detail-popup__val">{sps}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Reward</span>
          <span className="sim-detail-popup__val">{reward}</span>
        </div>
        <div className="sim-detail-popup__graph">
          <ConnectionSparkline history={fpsHistory} width={160} height={36} />
        </div>
        {fpsHistory.length > 2 && (
          <div className="sim-detail-popup__speed-text">
            <span>avg {avgFps}</span>
            <span className="sim-detail-popup__speed-sep">/</span>
            <span>min {minFps}</span>
            <span className="sim-detail-popup__speed-sep">/</span>
            <span>max {maxFps}</span>
            {jitter > 0 && (
              <>
                <span className="sim-detail-popup__speed-sep">/</span>
                <span className={jitter > 10 ? 'sim-detail-popup__speed-warn' : ''}>{'\u00b1'}{jitter}</span>
              </>
            )}
          </div>
        )}
      </div>

      {/* Frame delivery */}
      {(frameSizeKB || dropRate != null) && (
        <div className="sim-detail-popup__section">
          <div className="sim-detail-popup__heading">Frame Delivery</div>
          {frameSizeKB && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Avg Size</span>
              <span className="sim-detail-popup__val">{frameSizeKB} KB</span>
            </div>
          )}
          {dropRate != null && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Drop Rate</span>
              <span className={`sim-detail-popup__val ${parseFloat(dropRate) > 1 ? 'sim-detail-popup__val--warn' : ''}`}>{dropRate}%</span>
            </div>
          )}
        </div>
      )}

      {/* Safety / confidence */}
      {(confidence != null || clampedAxes.length > 0 || memMb != null) && (
        <div className="sim-detail-popup__section">
          <div className="sim-detail-popup__heading">Health</div>
          {confidence != null && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Confidence</span>
              <span className={`sim-detail-popup__val ${confidence < 0.5 ? 'sim-detail-popup__val--warn' : ''}`}>{(confidence * 100).toFixed(0)}%</span>
            </div>
          )}
          {clampedAxes.length > 0 && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Clamped</span>
              <span className="sim-detail-popup__val sim-detail-popup__val--warn">{clampedAxes.join(', ')}</span>
            </div>
          )}
          {memMb != null && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Memory</span>
              <span className="sim-detail-popup__val">{Math.round(memMb)} MB</span>
            </div>
          )}
        </div>
      )}

      {/* Environment */}
      {envInfo && (
        <div className="sim-detail-popup__section">
          <div className="sim-detail-popup__heading">Environment</div>
          <div className="sim-detail-popup__row">
            <span className="sim-detail-popup__key">Robot</span>
            <span className="sim-detail-popup__val">{envInfo.robot_name || 'Unknown'}</span>
          </div>
          <div className="sim-detail-popup__row">
            <span className="sim-detail-popup__key">Envs</span>
            <span className="sim-detail-popup__val">{envInfo.num_envs || 0}</span>
          </div>
          {envInfo.mode && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Mode</span>
              <span className="sim-detail-popup__val">{envInfo.mode}</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

const TrainDetailPopup = ({ trainingMetrics, stages }) => {
  const m = trainingMetrics || {};
  const step = m.step_count ?? 0;
  const reward = m.total_reward?.toFixed(2) ?? '0.00';
  const sps = m.fps != null ? Math.round(m.fps) : 0;

  const epLenMean = m.episode_lengths?.mean?.toFixed(0) ?? '\u2014';
  const epLenMin = m.episode_lengths?.min ?? '\u2014';
  const epLenMax = m.episode_lengths?.max ?? '\u2014';

  const policyCheckpoint = m.policy_checkpoint ?? 'none';
  const policyLoaded = m.policy_loaded ?? false;

  const rewardBreakdown = m.reward_breakdown;
  const valueLoss = m.value_loss?.toFixed(4);
  const surrogateLoss = m.surrogate_loss?.toFixed(4);
  const actionNoiseStd = m.action_noise_std?.toFixed(2);

  return (
    <div className="sim-detail-popup train-detail-popup">
      {/* Progress */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Progress</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Step</span>
          <span className="sim-detail-popup__val">{step.toLocaleString()}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Reward</span>
          <span className="sim-detail-popup__val">{reward}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">SPS</span>
          <span className="sim-detail-popup__val">{sps}</span>
        </div>
      </div>

      {/* Episode */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Episode</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Mean Len</span>
          <span className="sim-detail-popup__val">{epLenMean}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Min / Max</span>
          <span className="sim-detail-popup__val">{epLenMin} / {epLenMax}</span>
        </div>
      </div>

      {/* Policy */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Policy</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Loaded</span>
          <span className="sim-detail-popup__val">
            <span className={`sim-detail-popup__dot sim-detail-popup__dot--${policyLoaded ? 'on' : 'off'}`} style={{ display: 'inline-block', marginRight: 4 }} />
            {policyLoaded ? policyCheckpoint : 'none'}
          </span>
        </div>
      </div>

      {/* Reward Components */}
      {rewardBreakdown && Object.keys(rewardBreakdown).length > 0 && (
        <div className="sim-detail-popup__section">
          <div className="sim-detail-popup__heading">Reward Components</div>
          {Object.entries(rewardBreakdown).map(([key, val]) => (
            <div className="sim-detail-popup__row" key={key}>
              <span className="sim-detail-popup__key">{key.replace(/_/g, ' ')}</span>
              <span className={`sim-detail-popup__val ${val < 0 ? 'sim-detail-popup__val--warn' : ''}`}>
                {typeof val === 'number' ? val.toFixed(3) : String(val)}
              </span>
            </div>
          ))}
        </div>
      )}

      {/* Loss */}
      {(valueLoss || surrogateLoss) && (
        <div className="sim-detail-popup__section">
          <div className="sim-detail-popup__heading">Loss</div>
          {valueLoss && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Value</span>
              <span className="sim-detail-popup__val">{valueLoss}</span>
            </div>
          )}
          {surrogateLoss && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Surrogate</span>
              <span className="sim-detail-popup__val">{surrogateLoss}</span>
            </div>
          )}
          {actionNoiseStd && (
            <div className="sim-detail-popup__row">
              <span className="sim-detail-popup__key">Noise Std</span>
              <span className="sim-detail-popup__val">{actionNoiseStd}</span>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

const TrainHoverPopup = ({ trainingMetrics, stages }) => {
  const m = trainingMetrics || {};
  const trainData = stages?.train || {};
  const sps = m.fps != null ? Math.round(m.fps) : 0;
  const reward = m.total_reward?.toFixed(2) || '0.00';
  const step = m.step_count ?? 0;

  return (
    <div className="sim-hover-popup">
      <div className="sim-hover-popup__row">
        <span className={`sim-hover-popup__dot sim-hover-popup__dot--${trainData.health === 'ok' ? 'on' : 'off'}`} />
        <span className="sim-hover-popup__stream">
          {trainData.trend === 'rising' ? '\u2197' : trainData.trend === 'falling' ? '\u2198' : '\u2192'} {reward}r
        </span>
      </div>
      <div className="sim-hover-popup__row">
        <span className="sim-hover-popup__stat">Step {step.toLocaleString()}</span>
        <span className="sim-hover-popup__sep" />
        <span className="sim-hover-popup__stat">SPS {sps}</span>
      </div>
    </div>
  );
};

const EvalHoverPopup = ({ trainingMetrics, stages }) => {
  const m = trainingMetrics || {};
  const evalData = stages?.eval || {};
  const riskLevel = evalData.riskLevel || 'nominal';
  const clampCount = evalData.clampCount ?? m.safety_clamp_count ?? 0;
  const confidence = m.confidence;

  return (
    <div className="sim-hover-popup">
      <div className="sim-hover-popup__row">
        <span className={`stage-cell__risk stage-cell__risk--${riskLevel}`} style={{ fontSize: 10 }}>
          {riskLevel}
        </span>
        {clampCount > 0 && (
          <span className="sim-hover-popup__stat">{'\u00d7'}{clampCount}</span>
        )}
      </div>
      {confidence != null && (
        <div className="sim-hover-popup__row">
          <span className="sim-hover-popup__stat">Conf</span>
          <span className={`sim-hover-popup__stat ${confidence < 0.5 ? 'sim-detail-popup__val--warn' : ''}`}>
            {(confidence * 100).toFixed(0)}%
          </span>
        </div>
      )}
    </div>
  );
};

const SAFETY_FLAG_LABELS = {
  vel_clamped: 'VEL',
  accel_clamped: 'ACCEL',
  jerk_clamped: 'JERK',
  auto_stopped: 'STOP',
  clip_clamped: 'CLIP',
};

const EvalDetailPopup = ({ trainingMetrics, stages }) => {
  const m = trainingMetrics || {};
  const evalData = stages?.eval || {};

  const riskLevel = evalData.riskLevel || 'nominal';
  const clampCount = evalData.clampCount ?? m.safety_clamp_count ?? 0;
  const safetyFlags = m.safety_flags || evalData.safetyFlags || {};
  const activeFlags = Object.entries(safetyFlags)
    .filter(([, v]) => v)
    .map(([k]) => SAFETY_FLAG_LABELS[k] || k);

  const confidence = m.confidence;
  const confidenceGating = confidence != null && confidence < 0.5;

  const anomalyCount = m.anomaly_count ?? 0;
  const consecutiveAnomalies = m.consecutive_anomalies ?? 0;

  const epLenMean = m.episode_lengths?.mean?.toFixed(0) ?? '\u2014';
  const epLenMin = m.episode_lengths?.min ?? '\u2014';
  const epLenMax = m.episode_lengths?.max ?? '\u2014';

  const step = m.step_count ?? 0;
  const reward = m.total_reward?.toFixed(2) ?? '0.00';
  const sps = m.fps != null ? Math.round(m.fps) : 0;
  const memMb = m.memory_mb;

  return (
    <div className="sim-detail-popup eval-detail-popup">
      {/* Risk & Safety */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Safety</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Risk</span>
          <span className={`stage-cell__risk stage-cell__risk--${riskLevel}`}>
            {riskLevel}
          </span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Clamps</span>
          <span className={`sim-detail-popup__val ${clampCount > 0 ? 'sim-detail-popup__val--warn' : ''}`}>
            {clampCount}
          </span>
        </div>
        {activeFlags.length > 0 && (
          <div className="eval-detail-popup__flags">
            {activeFlags.map(flag => (
              <span key={flag} className="eval-detail-popup__flag-pill">{flag}</span>
            ))}
          </div>
        )}
      </div>

      {/* Confidence */}
      {confidence != null && (
        <div className="sim-detail-popup__section">
          <div className="sim-detail-popup__heading">Confidence</div>
          <div className="sim-detail-popup__row">
            <span className="sim-detail-popup__key">Policy</span>
            <span className={`sim-detail-popup__val ${confidence < 0.5 ? 'sim-detail-popup__val--warn' : ''}`}>
              {(confidence * 100).toFixed(0)}%
            </span>
          </div>
          <div className="eval-detail-popup__bar-wrap">
            <div
              className={`eval-detail-popup__bar ${confidence < 0.5 ? 'eval-detail-popup__bar--warn' : ''}`}
              style={{ width: `${Math.min(confidence * 100, 100)}%` }}
            />
          </div>
          {confidenceGating && (
            <div className="eval-detail-popup__gating-warn">GATING ACTIVE</div>
          )}
        </div>
      )}

      {/* Anomalies */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Anomalies</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Total</span>
          <span className={`sim-detail-popup__val ${anomalyCount > 0 ? 'sim-detail-popup__val--warn' : ''}`}>
            {anomalyCount}
          </span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Consecutive</span>
          <span className={`sim-detail-popup__val ${consecutiveAnomalies >= 2 ? 'sim-detail-popup__val--warn' : ''}`}>
            {consecutiveAnomalies} / 3
          </span>
        </div>
      </div>

      {/* Episode */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Episode</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Mean Len</span>
          <span className="sim-detail-popup__val">{epLenMean}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Min / Max</span>
          <span className="sim-detail-popup__val">{epLenMin} / {epLenMax}</span>
        </div>
      </div>

      {/* Performance */}
      <div className="sim-detail-popup__section">
        <div className="sim-detail-popup__heading">Performance</div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Step</span>
          <span className="sim-detail-popup__val">{step.toLocaleString()}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">Reward</span>
          <span className="sim-detail-popup__val">{reward}</span>
        </div>
        <div className="sim-detail-popup__row">
          <span className="sim-detail-popup__key">SPS</span>
          <span className="sim-detail-popup__val">{sps}</span>
        </div>
        {memMb != null && (
          <div className="sim-detail-popup__row">
            <span className="sim-detail-popup__key">Memory</span>
            <span className="sim-detail-popup__val">{Math.round(memMb)} MB</span>
          </div>
        )}
      </div>
    </div>
  );
};

const TrustStrip = ({ onStageClick, testMode = false, onModeChange, effectiveViewer = 'idle', canToggleViewer = false, onViewerToggle }) => {
  const { activePhase, authority, stages } = usePhase();
  const {
    genesisConnected,
    genesisMode,
    setMode,
    setAlpha,
    blendAlpha,
    estop,
    estopClear,
    safetyState,
    videoHealthy,
    currentRobot,
    streamBackend,
    bridgeConnected,
    visualFps,
    trainingMetrics,
    envInfo,
    frameStats,
  } = useGenesis();
  const { isOpen: trainingPanelOpen, togglePanel: toggleTrainingPanel } = useTrainingPanel();

  const stripClass = `trust-strip trust-strip--${authority}`;
  const isSim = genesisConnected || testMode;

  // Connection label: genesis→SIM, ros→REAL, idle→OFFLINE
  const connLabel = effectiveViewer === 'genesis' ? 'SIM' : effectiveViewer === 'ros' ? 'REAL' : 'OFFLINE';
  const connAlive = effectiveViewer !== 'idle';

  const [testActiveMode, setTestActiveMode] = useState('teleop');
  const [teleopHover, setTeleopHover] = useState(false);
  const [trainHover, setTrainHover] = useState(false);
  const [evalHover, setEvalHover] = useState(false);
  const [detailOpen, setDetailOpen] = useState(false);
  const [trainDetailOpen, setTrainDetailOpen] = useState(false);
  const [evalDetailOpen, setEvalDetailOpen] = useState(false);
  const teleopCellRef = useRef(null);
  const fpsHistoryRef = useRef([]);
  const [fpsHistory, setFpsHistory] = useState([]);

  // Accumulate FPS samples for the sparkline (~1 per second from SimViewer)
  useEffect(() => {
    if (!isSim) return;
    const buf = fpsHistoryRef.current;
    buf.push(visualFps);
    if (buf.length > SPARK_MAX) buf.shift();
    setFpsHistory([...buf]);
  }, [visualFps, isSim]);

  // Never collapse in test mode — always show labels + status
  const isTeleopCollapsed = !testMode && stages.teleop.health === 'dead' && activePhase !== 'teleop';
  const isTrainCollapsed = !testMode && stages.train.health === 'dead' && activePhase !== 'train';
  const isEvalCollapsed = !testMode && stages.eval.health === 'dead' && activePhase !== 'eval';

  const handleModeClick = useCallback((mode) => {
    setMode(mode);
    if (testMode) setTestActiveMode(mode);
    if (onModeChange) onModeChange(mode);
    setDetailOpen(false);
    setTrainDetailOpen(false);
    setEvalDetailOpen(false);
  }, [setMode, testMode, onModeChange]);

  const handleAlphaChange = useCallback((e) => {
    setAlpha(parseFloat(e.target.value));
  }, [setAlpha]);

  const handleEstop = useCallback(() => {
    if (estop) estop();
  }, [estop]);

  const handleTeleopClick = useCallback(() => {
    if (isSim || testMode) {
      setDetailOpen(prev => !prev);
      setTrainDetailOpen(false);
      setEvalDetailOpen(false);
    } else {
      onStageClick?.('teleop');
    }
  }, [isSim, testMode, onStageClick]);

  const handleTrainClick = useCallback(() => {
    if (isSim || testMode) {
      setTrainDetailOpen(prev => !prev);
      setDetailOpen(false);
      setEvalDetailOpen(false);
    } else {
      onStageClick?.('train');
    }
  }, [isSim, testMode, onStageClick]);

  const handleEvalClick = useCallback(() => {
    if (isSim || testMode) {
      setEvalDetailOpen(prev => !prev);
      setDetailOpen(false);
      setTrainDetailOpen(false);
    } else {
      onStageClick?.('eval');
    }
  }, [isSim, testMode, onStageClick]);

  // Map genesis mode names to our mode keys (use local state in test mode)
  const activeModeKey = testMode ? testActiveMode : (() => {
    if (genesisMode === 'teleop_record' || genesisMode === 'teleop') return 'teleop';
    if (genesisMode === 'eval' || genesisMode === 'policy') return 'policy';
    if (genesisMode === 'hil_blend' || genesisMode === 'blend' || genesisMode === 'online_finetune') return 'blend';
    return 'teleop';
  })();

  return (
    <div className={stripClass}>
      {/* Connection + Robot */}
      <div
        className={`trust-strip__connection ${canToggleViewer ? 'trust-strip__connection--clickable' : ''}`}
        onClick={canToggleViewer ? onViewerToggle : undefined}
        title={canToggleViewer ? `Click to switch to ${connLabel === 'SIM' ? 'REAL' : 'SIM'}` : undefined}
      >
        <span className={`trust-strip__conn-dot trust-strip__conn-dot--${connAlive ? 'on' : 'off'}`} />
        <span className="trust-strip__conn-label">{connLabel}</span>
        {currentRobot && (
          <span className="trust-strip__robot-name">{currentRobot.label || currentRobot.name}</span>
        )}
      </div>

      <div className="trust-strip__divider" />

      {/* Stage cells */}
      <div
        className="stage-cell-wrap"
        ref={teleopCellRef}
        onMouseEnter={() => (isSim || testMode) && setTeleopHover(true)}
        onMouseLeave={() => setTeleopHover(false)}
      >
        <StageCell stage="teleop" data={stages.teleop} collapsed={isTeleopCollapsed} onClick={handleTeleopClick} />
        {(isSim || testMode) && teleopHover && !detailOpen && (
          <SimHoverPopup
            streamBackend={streamBackend}
            bridgeConnected={bridgeConnected}
            genesisConnected={genesisConnected}
            visualFps={visualFps}
            trainingMetrics={trainingMetrics}
            fpsHistory={fpsHistory}
          />
        )}
        {(isSim || testMode) && detailOpen && (
          <SimDetailPopup
            streamBackend={streamBackend}
            bridgeConnected={bridgeConnected}
            genesisConnected={genesisConnected}
            visualFps={visualFps}
            trainingMetrics={trainingMetrics}
            envInfo={envInfo}
            frameStats={frameStats}
            fpsHistory={fpsHistory}
          />
        )}
      </div>
      <div className="trust-strip__divider" />
      <div
        className="stage-cell-wrap"
        onMouseEnter={() => (isSim || testMode) && setTrainHover(true)}
        onMouseLeave={() => setTrainHover(false)}
      >
        <StageCell stage="train" data={stages.train} collapsed={isTrainCollapsed} onClick={handleTrainClick} />
        {(isSim || testMode) && trainHover && !trainDetailOpen && (
          <TrainHoverPopup
            trainingMetrics={trainingMetrics}
            stages={stages}
          />
        )}
        {(isSim || testMode) && trainDetailOpen && (
          <TrainDetailPopup
            trainingMetrics={trainingMetrics}
            stages={stages}
          />
        )}
      </div>
      <div className="trust-strip__divider" />
      <div
        className="stage-cell-wrap"
        onMouseEnter={() => (isSim || testMode) && setEvalHover(true)}
        onMouseLeave={() => setEvalHover(false)}
      >
        <StageCell stage="eval" data={stages.eval} collapsed={isEvalCollapsed} onClick={handleEvalClick} />
        {(isSim || testMode) && evalHover && !evalDetailOpen && (
          <EvalHoverPopup
            trainingMetrics={trainingMetrics}
            stages={stages}
          />
        )}
        {(isSim || testMode) && evalDetailOpen && (
          <EvalDetailPopup
            trainingMetrics={trainingMetrics}
            stages={stages}
          />
        )}
      </div>

      <div className="trust-strip__spacer" />

      {/* Mode buttons — show when Genesis connected OR in test mode */}
      {(genesisConnected || testMode) && (
        <div className="trust-strip__mode-group">
          {MODES.map(({ key, label }) => (
            <button
              key={key}
              className={`trust-strip__mode-btn ${activeModeKey === key ? 'trust-strip__mode-btn--active' : ''}`}
              onClick={() => handleModeClick(key)}
            >
              {label}
            </button>
          ))}

          {/* Alpha slider — only in blend mode */}
          {activeModeKey === 'blend' && (
            <div className="trust-strip__alpha" title="Policy blend: 0 = human, 1 = policy">
              <span className="trust-strip__alpha-label">{'\u03B1'}</span>
              <input
                type="range"
                min="0"
                max="1"
                step="0.05"
                value={blendAlpha}
                onChange={handleAlphaChange}
                className="trust-strip__alpha-slider"
              />
              <span className="trust-strip__alpha-value">{blendAlpha.toFixed(2)}</span>
            </div>
          )}
        </div>
      )}

      {/* Lab Button (training session manager) */}
      {(genesisConnected || testMode) && (
        <button
          className={`trust-strip__train-btn ${trainingPanelOpen ? 'trust-strip__train-btn--active' : ''}`}
          onClick={toggleTrainingPanel}
          title="Open Training Lab"
        >
          LAB
        </button>
      )}

      {/* Safety badge */}
      {safetyState.mode !== 'ARMED' && (
        <span className={`trust-strip__safety-badge trust-strip__safety-badge--${safetyState.mode.toLowerCase()}`}>
          {safetyState.mode}
        </span>
      )}

      {/* E-STOP */}
      <button
        className={`trust-strip__estop ${safetyState.mode === 'ESTOP' ? 'trust-strip__estop--active' : ''}`}
        onClick={() => safetyState.mode === 'ESTOP' ? estopClear() : handleEstop()}
        title={safetyState.mode === 'ESTOP' ? 'Clear E-STOP (re-arm)' : 'Emergency stop'}
      >
        {safetyState.mode === 'ESTOP' ? 'RE-ARM' : 'E-STOP'}
      </button>
    </div>
  );
};

export default TrustStrip;
