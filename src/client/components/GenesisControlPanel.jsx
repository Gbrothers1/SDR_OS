import React from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import FloatingPanel from './FloatingPanel';
import '../styles/GenesisControlPanel.css';

const GenesisControlPanel = () => {
  const {
    genesisMode,
    genesisConnected,
    robotList,
    setMode,
    loadRobot,
    reset,
    pause,
    isPaused: paused,
    setAlpha,
    blendAlpha,
    commandSource,
    deadmanActiveForge,
    policyLoaded,
    policyCheckpoint,
    setCommandSource,
    frameStats,
  } = useGenesis();

  const modes = [
    { id: 'Real Robot', label: 'Real Robot' },
    { id: 'teleop_record', label: 'Teleop Record' },
    { id: 'hil_blend', label: 'HIL Blend' },
    { id: 'online_finetune', label: 'Online Finetune' },
    { id: 'eval', label: 'Evaluation' }
  ];

  return (
    <FloatingPanel
      title="Sim Control"
      defaultPosition={{ x: 20, y: 20 }}
      defaultSize={{ width: 280, height: 400 }}
      className="genesis-control-panel"
    >
      <div className="genesis-control-content">
        {/* Connection Status */}
        <div className="control-section">
          <div className="control-label">Status</div>
          <div className={`status-indicator ${genesisConnected ? 'connected' : 'disconnected'}`}>
            <span className="status-dot"></span>
            <span>{genesisConnected ? 'Connected' : 'Disconnected'}</span>
          </div>
          {genesisConnected && frameStats && (
            <div className="stream-stats">
              <span className="stat-value">{frameStats.bitrate_mibps?.toFixed(2) || '0.00'} Mibps</span>
              <span className="stat-label"> @ {frameStats.frame_rate?.toFixed(1) || '0'} fps</span>
            </div>
          )}
        </div>

        {/* Mode Selection */}
        <div className="control-section">
          <div className="control-label">Mode</div>
          <select
            className="control-select"
            value={genesisMode}
            onChange={(e) => setMode(e.target.value)}
          >
            {modes.map(mode => (
              <option key={mode.id} value={mode.id}>{mode.label}</option>
            ))}
          </select>
        </div>

        {/* Robot Selection */}
        {genesisMode !== 'Real Robot' && robotList.length > 0 && (
          <div className="control-section">
            <div className="control-label">Robot</div>
            <select
              className="control-select"
              onChange={(e) => loadRobot(e.target.value)}
            >
              <option value="">Select Robot...</option>
              {robotList.map(robot => (
                <option key={robot.name} value={robot.name}>
                  {robot.label || robot.name}
                </option>
              ))}
            </select>
          </div>
        )}

        {/* Command Source */}
        {genesisMode !== 'Real Robot' && (
          <div className="control-section">
            <div className="control-label">Command Source</div>
            <select
              className="control-select"
              value={commandSource}
              onChange={(e) => setCommandSource(e.target.value)}
            >
              <option value="gamepad">Gamepad</option>
              <option value="ros">ROS /cmd_vel</option>
            </select>
          </div>
        )}

        {/* Deadman & Policy Status */}
        {genesisMode !== 'Real Robot' && (
          <div className="control-section">
            <div className="control-label">Safety</div>
            <div className={`status-indicator ${deadmanActiveForge ? 'connected' : 'disconnected'}`}>
              <span className="status-dot"></span>
              <span>Deadman (L1): {deadmanActiveForge ? 'Active' : 'Released'}</span>
            </div>
            <div className={`status-indicator ${policyLoaded ? 'connected' : 'disconnected'}`}>
              <span className="status-dot"></span>
              <span>Policy: {policyLoaded ? (policyCheckpoint || 'Loaded') : 'None'}</span>
            </div>
          </div>
        )}

        {/* Simulation Controls */}
        {genesisMode !== 'Real Robot' && (
          <>
            <div className="control-section">
              <div className="control-label">Simulation</div>
              <div className="control-buttons">
                <button className="control-button" onClick={reset}>
                  Reset
                </button>
                <button
                  className={`control-button ${paused ? 'active' : ''}`}
                  onClick={() => pause(!paused)}
                >
                  {paused ? 'Resume' : 'Pause'}
                </button>
              </div>
            </div>

            {/* Blend Alpha */}
            {genesisMode === 'hil_blend' && (
              <div className="control-section">
                <div className="control-label">Blend Alpha: {blendAlpha.toFixed(2)}</div>
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.01"
                  value={blendAlpha}
                  onChange={(e) => setAlpha(parseFloat(e.target.value))}
                  className="control-slider"
                />
              </div>
            )}
          </>
        )}
      </div>
    </FloatingPanel>
  );
};

export default GenesisControlPanel;
