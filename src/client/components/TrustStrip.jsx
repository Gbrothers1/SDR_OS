import React, { useCallback, useState, useRef, useEffect, useMemo } from 'react';
import { usePhase } from '../contexts/PhaseContext';
import { useGenesis } from '../contexts/GenesisContext';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import '../styles/TrustStrip.css';

const MODES = [
  { key: 'teleop', label: 'TELEOP' },
  { key: 'policy', label: 'POLICY' },
  { key: 'blend', label: 'BLEND' },
];

const TREND_ARROWS = {
  rising: '\u2197',
  falling: '\u2198',
  flat: '\u2192',
};

const StageCell = ({ stage, data, collapsed, onClick }) => {
  const cellClass = [
    'stage-cell',
    collapsed && 'stage-cell--collapsed',
  ].filter(Boolean).join(' ');

  const dotClass = `stage-cell__dot stage-cell__dot--${data.health}`;

  return (
    <div className={cellClass} onClick={onClick}>
      <span className={dotClass} />
      {!collapsed && (
        <>
          <span className="stage-cell__label">{stage}</span>
          {stage === 'teleop' && (
            <span className="stage-cell__signal">
              {data.recording
                ? `${data.recordingStep ?? '?'}/${data.recordingTotal ?? '?'}`
                : data.actor}
            </span>
          )}
          {stage === 'train' && data.health !== 'dead' && (
            <>
              <span className={`stage-cell__trend stage-cell__trend--${data.trend}${data.unstable ? ' stage-cell__trend--unstable' : ''}`}>
                {TREND_ARROWS[data.trend] || '\u2192'}
              </span>
              <span className="stage-cell__signal">
                {data.reward != null ? data.reward.toFixed(1) + 'r' : ''}
              </span>
            </>
          )}
          {stage === 'eval' && data.health !== 'dead' && (
            <span className={`stage-cell__risk stage-cell__risk--${data.riskLevel}`}>
              {data.riskLevel === 'nominal' ? 'nominal' :
                `${data.riskLevel}${data.clampCount > 0 ? ` \u00d7${data.clampCount}` : ''}`}
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
  const streamLabel = streamBackend === 'webrtc' ? 'wRTC' : 'ScK';
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

const TrustStrip = ({ onStageClick, testMode = false }) => {
  const { activePhase, authority, stages, primaryViewer } = usePhase();
  const {
    genesisConnected,
    genesisMode,
    setMode,
    setAlpha,
    blendAlpha,
    estop,
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
  const isSim = (genesisConnected && primaryViewer !== 'ros') || testMode;

  const [teleopHover, setTeleopHover] = useState(false);
  const [detailOpen, setDetailOpen] = useState(false);
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

  const isTeleopCollapsed = stages.teleop.health === 'dead' && activePhase !== 'teleop';
  const isTrainCollapsed = stages.train.health === 'dead' && activePhase !== 'train';
  const isEvalCollapsed = stages.eval.health === 'dead' && activePhase !== 'eval';

  const handleModeClick = useCallback((mode) => {
    setMode(mode);
    setDetailOpen(false); // leave detail popup only when a mode button is pressed
  }, [setMode]);

  const handleAlphaChange = useCallback((e) => {
    setAlpha(parseFloat(e.target.value));
  }, [setAlpha]);

  const handleEstop = useCallback(() => {
    if (estop) estop();
  }, [estop]);

  const handleTeleopClick = useCallback(() => {
    if (isSim) {
      setDetailOpen(prev => !prev); // toggle stays until mode button closes it
    } else {
      onStageClick?.('teleop');
    }
  }, [isSim, onStageClick]);

  // Map genesis mode names to our mode keys
  const activeModeKey = (() => {
    if (genesisMode === 'teleop_record' || genesisMode === 'teleop') return 'teleop';
    if (genesisMode === 'eval' || genesisMode === 'policy') return 'policy';
    if (genesisMode === 'hil_blend' || genesisMode === 'blend' || genesisMode === 'online_finetune') return 'blend';
    return 'teleop';
  })();

  return (
    <div className={stripClass}>
      {/* Connection + Robot */}
      <div className="trust-strip__connection">
        <span className={`trust-strip__conn-dot trust-strip__conn-dot--${(primaryViewer === 'ros' || genesisConnected) ? 'on' : testMode ? 'test' : 'off'}`} />
        <span className="trust-strip__conn-label">
          {primaryViewer === 'ros' ? 'REAL' : genesisConnected ? 'SIM' : testMode ? 'TEST' : 'OFFLINE'}
        </span>
        {currentRobot && (
          <span className="trust-strip__robot-name">{currentRobot.label || currentRobot.name}</span>
        )}
      </div>

      <div className="trust-strip__divider" />

      {/* Stage cells */}
      <div
        className="stage-cell-wrap"
        ref={teleopCellRef}
        onMouseEnter={() => isSim && setTeleopHover(true)}
        onMouseLeave={() => setTeleopHover(false)}
      >
        <StageCell stage="teleop" data={stages.teleop} collapsed={isTeleopCollapsed} onClick={handleTeleopClick} />
        {isSim && teleopHover && !detailOpen && (
          <SimHoverPopup
            streamBackend={streamBackend}
            bridgeConnected={bridgeConnected}
            genesisConnected={genesisConnected}
            visualFps={visualFps}
            trainingMetrics={trainingMetrics}
            fpsHistory={fpsHistory}
          />
        )}
        {isSim && detailOpen && (
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
      <StageCell stage="train" data={stages.train} collapsed={isTrainCollapsed} onClick={() => onStageClick?.('train')} />
      <div className="trust-strip__divider" />
      <StageCell stage="eval" data={stages.eval} collapsed={isEvalCollapsed} onClick={() => onStageClick?.('eval')} />

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
            <div className="trust-strip__alpha">
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

      {/* Train Button */}
      {(genesisConnected || testMode) && (
        <button
          className={`trust-strip__train-btn ${trainingPanelOpen ? 'trust-strip__train-btn--active' : ''}`}
          onClick={toggleTrainingPanel}
          title="Toggle Training Panel"
        >
          TRAIN
        </button>
      )}

      {/* E-STOP */}
      <button className="trust-strip__estop" onClick={handleEstop}>
        E-STOP
      </button>
    </div>
  );
};

export default TrustStrip;
