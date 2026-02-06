import React, { useEffect, useState } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/SafetyIndicator.css';

const SafetyIndicator = () => {
  const { trainingMetrics, genesisMode } = useGenesis();
  const [flashWarning, setFlashWarning] = useState(false);
  
  // Don't show in Real Robot mode
  if (genesisMode === 'Real Robot') {
    return null;
  }
  
  const blendAlpha = trainingMetrics?.blend_alpha || 0;
  const deadmanActive = trainingMetrics?.deadman_active || false;
  const safetyFlags = trainingMetrics?.safety_flags || {};
  
  // Flash warning when safety clamps trigger
  useEffect(() => {
    const hasClamps = Object.values(safetyFlags).some(flag => flag === true);
    if (hasClamps) {
      setFlashWarning(true);
      const timer = setTimeout(() => setFlashWarning(false), 500);
      return () => clearTimeout(timer);
    }
  }, [safetyFlags]);
  
  // Determine color based on alpha
  const getAlphaColor = () => {
    if (blendAlpha < 0.01) return 'alpha-teleop'; // Pure teleop (green)
    if (blendAlpha > 0.99) return 'alpha-policy'; // Pure policy (blue)
    return 'alpha-blend'; // Blending (yellow)
  };
  
  const getActorLabel = () => {
    if (blendAlpha < 0.01) return 'Teleop';
    if (blendAlpha > 0.99) return 'Policy';
    return 'Blend';
  };
  
  return (
    <div className={`safety-indicator ${flashWarning ? 'flash-warning' : ''}`}>
      {/* Alpha status */}
      <div className={`safety-alpha ${getAlphaColor()}`}>
        <div className="safety-label">α</div>
        <div className="safety-value">{blendAlpha.toFixed(2)}</div>
        <div className="safety-actor">{getActorLabel()}</div>
      </div>
      
      {/* Deadman status */}
      <div className={`safety-deadman ${deadmanActive ? 'active' : 'inactive'}`}>
        <div className="safety-label">Deadman</div>
        <div className="safety-icon">
          {deadmanActive ? '✓' : '✗'}
        </div>
      </div>
      
      {/* Safety flags */}
      {Object.keys(safetyFlags).length > 0 && (
        <div className="safety-flags">
          <div className="safety-label">Safety</div>
          <div className="safety-flags-list">
            {safetyFlags.vel_clamped && <div className="safety-flag">VEL</div>}
            {safetyFlags.accel_clamped && <div className="safety-flag">ACCEL</div>}
            {safetyFlags.jerk_clamped && <div className="safety-flag">JERK</div>}
            {safetyFlags.auto_stopped && <div className="safety-flag critical">STOP</div>}
            {!Object.values(safetyFlags).some(f => f) && (
              <div className="safety-flag ok">OK</div>
            )}
          </div>
        </div>
      )}
    </div>
  );
};

export default SafetyIndicator;
