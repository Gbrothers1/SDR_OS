import React, { useState, useCallback } from 'react';
import { usePhase } from '../contexts/PhaseContext';
import { useGenesis } from '../contexts/GenesisContext';
import RobotViewer from './RobotViewer';
import SimViewer from './SimViewer';
import '../styles/ViewerLayer.css';

const SimPlaceholder = () => (
  <div className="viewer-layer__sim-placeholder">
    <div className="viewer-layer__sim-placeholder-label">SIM</div>
    <div className="viewer-layer__sim-placeholder-hint">Genesis bridge not connected</div>
  </div>
);

const ViewerLayer = ({ ros, appSettings, testMode = false }) => {
  const { rosConnected, genesisConnected } = usePhase();
  const { currentFrame, mediaStream, streamBackend } = useGenesis();

  // Track which source is primary
  // Default: genesis takes priority when connected (richer feed)
  const [primaryOverride, setPrimaryOverride] = useState(null);

  const hasGenesis = genesisConnected && (currentFrame || mediaStream);
  const hasRos = rosConnected;

  let primarySource = 'idle';
  if (primaryOverride) {
    primarySource = primaryOverride;
  } else if (hasGenesis) {
    primarySource = 'genesis';
  } else if (hasRos) {
    primarySource = 'ros';
  } else if (testMode) {
    // In test mode, show the RobotViewer demo scene (grid + axes + orbit controls)
    primarySource = 'ros';
  }

  // Show PiP when both sources are available, or in test mode
  const showPip = hasGenesis && hasRos;
  const showTestPip = testMode && !showPip;
  const pipSource = primarySource === 'genesis' ? 'ros' : 'genesis';

  // Swap primary/pip on PiP click
  const handlePipClick = useCallback(() => {
    setPrimaryOverride(prev => {
      if (prev === 'genesis') return 'ros';
      if (prev === 'ros') return 'genesis';
      // If no override, swap from auto-selected
      return primarySource === 'genesis' ? 'ros' : 'genesis';
    });
  }, [primarySource]);

  // Test mode PiP swap: toggle between ros (3D) and genesis (placeholder)
  const handleTestPipClick = useCallback(() => {
    setPrimaryOverride(prev => {
      if (prev === 'genesis') return 'ros';
      return 'genesis';
    });
  }, []);

  const renderSource = (source, isPip = false) => {
    if (source === 'genesis') {
      if (hasGenesis) {
        return <SimViewer />;
      }
      // Test mode: show placeholder for sim
      return <SimPlaceholder />;
    }
    if (source === 'ros') {
      return (
        <RobotViewer
          ros={ros}
          tfThrottleRate={appSettings?.visualization?.tfThrottleRate ?? 10}
          settings={appSettings}
        />
      );
    }
    return null;
  };

  return (
    <div className="viewer-layer">
      {/* Primary viewer */}
      <div className="viewer-layer__primary">
        {primarySource === 'idle' ? (
          <div className="viewer-layer__idle">
            <div className="viewer-layer__idle-label">SDR_OS</div>
            <div className="viewer-layer__idle-hint">Waiting for connections...</div>
          </div>
        ) : (
          renderSource(primarySource)
        )}
      </div>

      {/* PiP thumbnail — real when both sources available */}
      {showPip && (
        <div className="viewer-layer__pip" onClick={handlePipClick} title="Click to swap">
          {renderSource(pipSource, true)}
        </div>
      )}

      {/* Test mode PiP — swappable between 3D scene and SIM placeholder */}
      {showTestPip && (
        <div className="viewer-layer__pip" onClick={handleTestPipClick} title="Click to swap REAL/SIM">
          {renderSource(primarySource === 'ros' ? 'genesis' : 'ros', true)}
        </div>
      )}
    </div>
  );
};

export default ViewerLayer;
