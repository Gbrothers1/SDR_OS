import React, { useState } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/SimControlPanel.css';

const CAMERA_PRESETS = [
  { id: 'follow', label: 'Follow', position: null, lookat: null }, // Auto-follow
  { id: 'front', label: 'Front', position: [2, 0, 1], lookat: [0, 0, 0.5] },
  { id: 'side', label: 'Side', position: [0, 2, 1], lookat: [0, 0, 0.5] },
  { id: 'top', label: 'Top', position: [0, 0, 3], lookat: [0, 0, 0] },
  { id: 'orbit', label: 'Orbit', position: [2, 2, 1.5], lookat: [0, 0, 0.5] }
];

const RESOLUTIONS = [
  { value: '640x480', label: '640×480' },
  { value: '1280x720', label: '1280×720 (HD)' },
  { value: '1920x1080', label: '1920×1080 (Full HD)' }
];

const SimControlPanel = () => {
  const {
    genesisMode,
    envInfo,
    resetEnv,
    pauseSim,
    selectEnv,
    setCameraPos,
    setAlpha
  } = useGenesis();
  
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [isPaused, setIsPaused] = useState(false);
  const [selectedEnvIdx, setSelectedEnvIdx] = useState(0);
  const [resolution, setResolution] = useState('1280x720');
  const [jpegQuality, setJpegQuality] = useState(80);
  const [manualAlpha, setManualAlpha] = useState(0);
  
  // Don't show in Real Robot mode
  if (genesisMode === 'Real Robot') {
    return null;
  }
  
  const numEnvs = envInfo?.num_envs || 1;
  
  const handleReset = () => {
    console.log('Resetting environment');
    resetEnv();
  };
  
  const handlePauseToggle = () => {
    const newPausedState = !isPaused;
    setIsPaused(newPausedState);
    pauseSim(newPausedState);
  };
  
  const handleEnvChange = (event) => {
    const idx = parseInt(event.target.value);
    setSelectedEnvIdx(idx);
    selectEnv(idx);
  };
  
  const handleCameraPreset = (preset) => {
    if (preset.position && preset.lookat) {
      setCameraPos(preset.position, preset.lookat);
    }
    // For 'follow' preset, we'd send a special command (handled in bridge)
  };
  
  const handleResolutionChange = (event) => {
    setResolution(event.target.value);
    // Would send to bridge to update camera resolution
  };
  
  const handleQualityChange = (event) => {
    setJpegQuality(parseInt(event.target.value));
    // Would send to bridge to update JPEG quality
  };
  
  const handleAlphaChange = (event) => {
    const alpha = parseFloat(event.target.value);
    setManualAlpha(alpha);
    setAlpha(alpha);
  };
  
  return (
    <div className={`sim-control-panel ${isCollapsed ? 'collapsed' : ''}`}>
      <div className="sim-control-header" onClick={() => setIsCollapsed(!isCollapsed)}>
        <span>Sim Controls</span>
        <span className="sim-control-toggle">{isCollapsed ? '▼' : '▲'}</span>
      </div>
      
      {!isCollapsed && (
        <div className="sim-control-content">
          {/* Environment control */}
          <div className="sim-control-section">
            <div className="sim-control-label">Environment</div>
            <div className="sim-control-row">
              <button className="sim-control-button" onClick={handleReset}>
                Reset
              </button>
              <button
                className={`sim-control-button ${isPaused ? 'active' : ''}`}
                onClick={handlePauseToggle}
              >
                {isPaused ? 'Resume' : 'Pause'}
              </button>
            </div>
            
            {numEnvs > 1 && (
              <div className="sim-control-row">
                <label className="sim-control-sublabel">View Env:</label>
                <select
                  className="sim-control-select"
                  value={selectedEnvIdx}
                  onChange={handleEnvChange}
                >
                  {Array.from({ length: numEnvs }, (_, i) => (
                    <option key={i} value={i}>
                      Env {i}
                    </option>
                  ))}
                </select>
              </div>
            )}
          </div>
          
          {/* Camera presets */}
          <div className="sim-control-section">
            <div className="sim-control-label">Camera</div>
            <div className="sim-control-grid">
              {CAMERA_PRESETS.map(preset => (
                <button
                  key={preset.id}
                  className="sim-control-button-small"
                  onClick={() => handleCameraPreset(preset)}
                >
                  {preset.label}
                </button>
              ))}
            </div>
          </div>
          
          {/* Render settings */}
          <div className="sim-control-section">
            <div className="sim-control-label">Render Quality</div>
            <div className="sim-control-row">
              <label className="sim-control-sublabel">Resolution:</label>
              <select
                className="sim-control-select"
                value={resolution}
                onChange={handleResolutionChange}
              >
                {RESOLUTIONS.map(res => (
                  <option key={res.value} value={res.value}>
                    {res.label}
                  </option>
                ))}
              </select>
            </div>
            <div className="sim-control-row">
              <label className="sim-control-sublabel">
                JPEG Quality: {jpegQuality}
              </label>
              <input
                type="range"
                min="50"
                max="100"
                value={jpegQuality}
                onChange={handleQualityChange}
                className="sim-control-slider"
              />
            </div>
          </div>
          
          {/* Alpha override (for blend modes) */}
          {(genesisMode === 'hil_blend' || genesisMode === 'online_finetune') && (
            <div className="sim-control-section">
              <div className="sim-control-label">Blend Override</div>
              <div className="sim-control-row">
                <label className="sim-control-sublabel">
                  Alpha: {manualAlpha.toFixed(2)}
                </label>
                <input
                  type="range"
                  min="0"
                  max="1"
                  step="0.01"
                  value={manualAlpha}
                  onChange={handleAlphaChange}
                  className="sim-control-slider"
                />
              </div>
              <div className="sim-control-hint">
                0 = pure teleop, 1 = pure policy
              </div>
            </div>
          )}
        </div>
      )}
    </div>
  );
};

export default SimControlPanel;
