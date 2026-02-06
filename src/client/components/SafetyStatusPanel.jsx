import React from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import FloatingPanel from './FloatingPanel';
import '../styles/SafetyStatusPanel.css';

const SafetyStatusPanel = () => {
  const { genesisMode, blendAlpha, deadmanActive, safetyFlags, actorTag } = useGenesis();

  if (genesisMode === 'Real Robot') {
    return null;
  }

  const getAlphaColor = () => {
    if (blendAlpha < 0.01) return '#00ff00'; // Teleop
    if (blendAlpha > 0.99) return '#00aaff'; // Policy
    return '#ffaa00'; // Blend
  };

  const getActorLabel = () => {
    if (actorTag === 'teleop') return 'Teleop';
    if (actorTag === 'policy') return 'Policy';
    if (actorTag === 'blend') return 'Blend';
    return actorTag || 'Unknown';
  };

  return (
    <FloatingPanel
      title="Safety Status"
      defaultPosition={{ x: 320, y: 20 }}
      defaultSize={{ width: 240, height: 200 }}
      className="safety-status-panel"
    >
      <div className="safety-status-content">
        {/* Blend Alpha */}
        <div className="safety-item">
          <div className="safety-item-label">Blend Alpha</div>
          <div className="safety-item-value" style={{ color: getAlphaColor() }}>
            {blendAlpha.toFixed(2)}
          </div>
          <div className="safety-item-bar">
            <div
              className="safety-item-bar-fill"
              style={{
                width: `${blendAlpha * 100}%`,
                backgroundColor: getAlphaColor()
              }}
            />
          </div>
        </div>

        {/* Deadman Status */}
        <div className="safety-item">
          <div className="safety-item-label">Deadman</div>
          <div className={`safety-item-status ${deadmanActive ? 'active' : 'inactive'}`}>
            <span className="safety-status-dot"></span>
            {deadmanActive ? 'Active' : 'Inactive'}
          </div>
        </div>

        {/* Actor Tag */}
        <div className="safety-item">
          <div className="safety-item-label">Actor</div>
          <div className="safety-item-value">{getActorLabel()}</div>
        </div>

        {/* Safety Flags */}
        {safetyFlags && Object.keys(safetyFlags).length > 0 && (
          <div className="safety-item">
            <div className="safety-item-label">Flags</div>
            <div className="safety-flags">
              {Object.entries(safetyFlags).map(([key, value]) => (
                value && (
                  <span key={key} className="safety-flag">
                    {key}
                  </span>
                )
              ))}
            </div>
          </div>
        )}
      </div>
    </FloatingPanel>
  );
};

export default SafetyStatusPanel;
