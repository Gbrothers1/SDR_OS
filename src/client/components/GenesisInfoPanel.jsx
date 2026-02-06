import React, { useState } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/GenesisInfoPanel.css';

const GenesisInfoPanel = () => {
  const {
    genesisMode,
    genesisConnected,
    currentRobot,
    envInfo,
    trainingMetrics,
    memoryEstimate,
    blendAlpha,
    deadmanActive,
    actorTag,
    isPaused
  } = useGenesis();

  const [showPanel, setShowPanel] = useState(true);

  const formatBytes = (bytes) => {
    if (!bytes) return 'N/A';
    const gb = bytes / (1024 ** 3);
    return `${gb.toFixed(2)} GB`;
  };

  const getActorColor = () => {
    if (actorTag === 'teleop') return '#00ff00';
    if (actorTag === 'policy') return '#00aaff';
    if (actorTag === 'blend') return '#ffaa00';
    return '#888';
  };

  return (
    <div className={`genesis-info-panel ${!showPanel ? 'collapsed' : ''}`}>
      <div className="panel-header">
        <h3>Simulation Info</h3>
        <button className="toggle-button" onClick={() => setShowPanel(!showPanel)}>
          <span>{showPanel ? "▲" : "▼"}</span>
        </button>
      </div>
      
      {showPanel && (
        <div className="panel-content">
          {/* Connection Status */}
          <div className="info-section">
            <h4>Connection</h4>
            <div className={`status-indicator ${genesisConnected ? 'connected' : 'disconnected'}`}>
              <span className="status-dot"></span>
              <span>{genesisConnected ? 'Connected' : 'Disconnected'}</span>
            </div>
          </div>

          {/* Current Robot */}
          {currentRobot && (
            <div className="info-section">
              <h4>Robot</h4>
              <div className="info-grid">
                <div className="info-row">
                  <span className="info-label">Name:</span>
                  <span className="info-value">{currentRobot.label || currentRobot.name}</span>
                </div>
                <div className="info-row">
                  <span className="info-label">Config:</span>
                  <span className="info-value">{currentRobot.name}</span>
                </div>
              </div>
            </div>
          )}

          {/* Mode */}
          <div className="info-section">
            <h4>Mode</h4>
            <div className="mode-badge">{genesisMode}</div>
            {isPaused && <div className="pause-indicator">⏸ PAUSED</div>}
          </div>

          {/* Actor & Control Status */}
          {genesisMode !== 'Real Robot' && trainingMetrics && (
            <div className="info-section">
              <h4>Control Status</h4>
              <div className="info-grid">
                <div className="info-row">
                  <span className="info-label">Actor:</span>
                  <span className="info-value" style={{ color: getActorColor() }}>
                    {actorTag || 'N/A'}
                  </span>
                </div>
                <div className="info-row">
                  <span className="info-label">Deadman:</span>
                  <span className={`info-value ${deadmanActive ? 'active' : ''}`}>
                    {deadmanActive ? '✓ Active' : '✗ Inactive'}
                  </span>
                </div>
                {(genesisMode === 'hil_blend' || genesisMode === 'online_finetune') && (
                  <div className="info-row">
                    <span className="info-label">Blend α:</span>
                    <span className="info-value">{blendAlpha.toFixed(2)}</span>
                  </div>
                )}
              </div>
            </div>
          )}

          {/* Environment Info */}
          {envInfo && (
            <div className="info-section">
              <h4>Environment</h4>
              <div className="info-grid">
                <div className="info-row">
                  <span className="info-label">Num Envs:</span>
                  <span className="info-value">{envInfo.num_envs || 1}</span>
                </div>
                {envInfo.episode && (
                  <div className="info-row">
                    <span className="info-label">Episode:</span>
                    <span className="info-value">{envInfo.episode}</span>
                  </div>
                )}
              </div>
            </div>
          )}

          {/* Training Metrics */}
          {trainingMetrics && (
            <div className="info-section">
              <h4>Training Metrics</h4>
              <div className="info-grid">
                {trainingMetrics.step_count !== undefined && (
                  <div className="info-row">
                    <span className="info-label">Step:</span>
                    <span className="info-value">{trainingMetrics.step_count}</span>
                  </div>
                )}
                {trainingMetrics.total_reward !== undefined && (
                  <div className="info-row">
                    <span className="info-label">Reward:</span>
                    <span className="info-value">{trainingMetrics.total_reward.toFixed(2)}</span>
                  </div>
                )}
                {trainingMetrics.fps !== undefined && (
                  <div className="info-row">
                    <span className="info-label">SPS:</span>
                    <span className="info-value">{trainingMetrics.fps.toFixed(1)}</span>
                  </div>
                )}
              </div>
            </div>
          )}

          {/* Memory Estimate */}
          {memoryEstimate && (
            <div className="info-section">
              <h4>Memory Usage</h4>
              <div className="info-grid">
                <div className="info-row">
                  <span className="info-label">VRAM:</span>
                  <span className="info-value">{formatBytes(memoryEstimate.vram)}</span>
                </div>
                <div className="info-row">
                  <span className="info-label">DRAM:</span>
                  <span className="info-value">{formatBytes(memoryEstimate.dram)}</span>
                </div>
                {memoryEstimate.total && (
                  <div className="info-row">
                    <span className="info-label">Total:</span>
                    <span className="info-value">{formatBytes(memoryEstimate.total)}</span>
                  </div>
                )}
              </div>
            </div>
          )}

          {/* No robot loaded message */}
          {!currentRobot && genesisConnected && (
            <div className="info-section">
              <div className="no-data-message">
                No robot loaded. Use the Sim Control panel to select and load a robot.
              </div>
            </div>
          )}

          {/* Disconnected message */}
          {!genesisConnected && (
            <div className="info-section">
              <div className="no-data-message">
                Sim bridge not connected. Start the bridge server:
                <code>python genesis_bridge/bridge_server.py</code>
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default GenesisInfoPanel;
