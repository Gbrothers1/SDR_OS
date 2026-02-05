import React, { useEffect, useMemo, useState } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/EvalControlPanel.css';

const formatNumber = (value, digits = 2) => {
  if (value === null || value === undefined || Number.isNaN(value)) {
    return '0.00';
  }
  return Number(value).toFixed(digits);
};

const EvalControlPanel = () => {
  const {
    genesisMode,
    genesisConnected,
    trainingMetrics,
    reset,
    pause,
    isPaused,
    setAlpha,
    blendAlpha,
    deadmanActive,
    actorTag
  } = useGenesis();

  const [policyOverride, setPolicyOverride] = useState(true);
  const [targetAlpha, setTargetAlpha] = useState(1);

  useEffect(() => {
    if (genesisMode !== 'eval') {
      return;
    }
    if (policyOverride) {
      setTargetAlpha(1);
      setAlpha(1);
    }
  }, [genesisMode, policyOverride, setAlpha]);

  const metrics = trainingMetrics || {};
  const episodeStats = useMemo(() => ({
    mean: metrics.episode_lengths?.mean ?? 0,
    min: metrics.episode_lengths?.min ?? 0,
    max: metrics.episode_lengths?.max ?? 0
  }), [metrics.episode_lengths]);

  const handleAlphaChange = (event) => {
    const nextAlpha = parseFloat(event.target.value);
    setTargetAlpha(nextAlpha);
    setAlpha(nextAlpha);
  };

  return (
    <div className="eval-control-content">
        <div className="eval-section">
          <div className="eval-label">Status</div>
          <div className={`eval-status ${genesisConnected ? 'connected' : 'disconnected'}`}>
            <span className="eval-status-dot" />
            <span>{genesisConnected ? 'Connected' : 'Disconnected'}</span>
          </div>
          <div className="eval-status-row">
            <span>Actor</span>
            <strong>{actorTag || 'teleop'}</strong>
          </div>
          <div className="eval-status-row">
            <span>Deadman</span>
            <strong className={deadmanActive ? 'ok' : 'warn'}>
              {deadmanActive ? 'ACTIVE' : 'INACTIVE'}
            </strong>
          </div>
        </div>

        <div className="eval-section">
          <div className="eval-label">Controls</div>
          <div className="eval-button-row">
            <button className="eval-button" onClick={reset}>
              Reset
            </button>
            <button
              className={`eval-button ${isPaused ? 'active' : ''}`}
              onClick={() => pause(!isPaused)}
            >
              {isPaused ? 'Resume' : 'Pause'}
            </button>
          </div>
          <button
            className={`eval-toggle ${policyOverride ? 'active' : ''}`}
            onClick={() => setPolicyOverride(prev => !prev)}
          >
            {policyOverride ? 'Policy Locked' : 'Policy Unlocked'}
          </button>
          <div className="eval-slider-row">
            <label>
              Target Alpha: {formatNumber(targetAlpha, 2)}
            </label>
            <input
              type="range"
              min="0"
              max="1"
              step="0.01"
              value={policyOverride ? 1 : targetAlpha}
              onChange={handleAlphaChange}
              disabled={policyOverride}
            />
            <div className="eval-hint">1.0 = policy only</div>
          </div>
          {policyOverride && !deadmanActive && (
            <div className="eval-warning">
              Deadman is inactive. Hold L1 to allow policy actions.
            </div>
          )}
        </div>

        <div className="eval-section">
          <div className="eval-label">Metrics</div>
          <div className="eval-metric-grid">
            <div className="eval-metric">
              <span className="eval-metric-label">Step</span>
              <span className="eval-metric-value">{metrics.step_count || 0}</span>
            </div>
            <div className="eval-metric">
              <span className="eval-metric-label">FPS</span>
              <span className="eval-metric-value">{metrics.fps || 0}</span>
            </div>
            <div className="eval-metric">
              <span className="eval-metric-label">Reward</span>
              <span className="eval-metric-value">{formatNumber(metrics.total_reward, 2)}</span>
            </div>
            <div className="eval-metric">
              <span className="eval-metric-label">Alpha</span>
              <span className="eval-metric-value">{formatNumber(blendAlpha, 2)}</span>
            </div>
          </div>
          <div className="eval-subgrid">
            <div>
              <span>Episode Mean</span>
              <strong>{formatNumber(episodeStats.mean, 1)}</strong>
            </div>
            <div>
              <span>Episode Min</span>
              <strong>{formatNumber(episodeStats.min, 1)}</strong>
            </div>
            <div>
              <span>Episode Max</span>
              <strong>{formatNumber(episodeStats.max, 1)}</strong>
            </div>
          </div>
        </div>
    </div>
  );
};

export default EvalControlPanel;
