import React, { useCallback } from 'react';
import { usePhase } from '../contexts/PhaseContext';
import { useGenesis } from '../contexts/GenesisContext';
import RobotViewer from './RobotViewer';
import SimViewer from './SimViewer';
import '../styles/ViewerLayer.css';

const SimPlaceholder = () => (
  <div className="viewer-layer__sim-placeholder">
    <div className="viewer-layer__sim-placeholder-label">SIM</div>
    <div className="viewer-layer__sim-placeholder-hint">Sim bridge not connected</div>
  </div>
);

const ViewerLayer = ({ ros, appSettings, testMode = false, viewerOverride = null, onViewerChange }) => {
  const { rosConnected, genesisConnected } = usePhase();
  const { currentFrame, mediaStream, streamBackend, streamCodec } = useGenesis();

  const hasGenesis = genesisConnected && (currentFrame || mediaStream || streamCodec === 'h264');
  const hasRos = rosConnected;

  // Primary source: external override > genesis > ros > test fallback
  let primarySource = 'idle';
  if (viewerOverride) {
    primarySource = viewerOverride;
  } else if (hasGenesis) {
    primarySource = 'genesis';
  } else if (hasRos) {
    primarySource = 'ros';
  } else if (testMode) {
    primarySource = 'ros';
  }

  // Show PiP when both sources are available, or in test mode
  const showPip = hasGenesis && hasRos;
  const showTestPip = testMode && !showPip;
  const pipSource = primarySource === 'genesis' ? 'ros' : 'genesis';

  // Swap primary/pip on PiP click
  const handlePipClick = useCallback(() => {
    if (onViewerChange) {
      onViewerChange(primarySource === 'genesis' ? 'ros' : 'genesis');
    }
  }, [primarySource, onViewerChange]);

  // Test mode PiP swap
  const handleTestPipClick = useCallback(() => {
    if (onViewerChange) {
      onViewerChange(primarySource === 'genesis' ? 'ros' : 'genesis');
    }
  }, [primarySource, onViewerChange]);

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
