import React, { useState, useEffect, useCallback } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import SimTelemetryPane from './SimTelemetryPane';
import TelemetryPanel from './TelemetryPanel';
import '../styles/TelemetryConsole.css';

const PRESETS = {
  operate: { label: 'ROS', desc: 'ROS-first' },
  sim: { label: 'SIM', desc: 'Simulation-first' },
  split: { label: 'SPLIT', desc: 'Side-by-side' },
};

const TelemetryHeader = ({ rosConnected, genesisConnected, rosFps, simFps }) => (
  <div className="tel-console__header">
    <div className="tel-console__conns">
      <span className={`tel-console__conn ${genesisConnected ? 'tel-console__conn--on' : ''}`}>
        SIM {genesisConnected ? `${simFps}fps` : ''}
      </span>
      <span className={`tel-console__conn ${rosConnected ? 'tel-console__conn--on' : ''}`}>
        ROS {rosConnected ? `${rosFps}Hz` : ''}
      </span>
    </div>
  </div>
);

const TelemetryConsole = ({ ros, rosConnected, appSettings }) => {
  const { genesisConnected, genesisMode, trainingMetrics } = useGenesis();

  const [activePreset, setActivePreset] = useState('sim');
  const [pinned, setPinned] = useState(false);

  // Auto-switch preset based on connection state
  useEffect(() => {
    if (pinned) return;

    if (rosConnected && genesisConnected) {
      const isBlend = genesisMode === 'hil_blend' || genesisMode === 'blend' || genesisMode === 'online_finetune';
      setActivePreset(isBlend ? 'split' : 'sim');
    } else if (genesisConnected) {
      setActivePreset('sim');
    } else if (rosConnected) {
      setActivePreset('operate');
    } else {
      setActivePreset('sim');
    }
  }, [rosConnected, genesisConnected, genesisMode, pinned]);

  const handlePresetClick = useCallback((preset) => {
    setActivePreset(preset);
    setPinned(true);
  }, []);

  const handleUnpin = useCallback(() => {
    setPinned(false);
  }, []);

  const simFps = trainingMetrics?.fps?.toFixed(0) ?? '0';
  const rosFps = '10'; // ROS default rate

  return (
    <div className="tel-console">
      <TelemetryHeader
        rosConnected={rosConnected}
        genesisConnected={genesisConnected}
        rosFps={rosFps}
        simFps={simFps}
      />

      {/* Preset switcher */}
      <div className="tel-console__presets">
        {Object.entries(PRESETS).map(([key, { label }]) => (
          <button
            key={key}
            className={`tel-console__preset-btn ${activePreset === key ? 'tel-console__preset-btn--active' : ''}`}
            onClick={() => handlePresetClick(key)}
          >
            {label}
          </button>
        ))}
        {pinned && (
          <button className="tel-console__unpin-btn" onClick={handleUnpin} title="Auto-switch">
            {'\u2297'}
          </button>
        )}
      </div>

      {/* Layout content */}
      <div className="tel-console__content">
        {activePreset === 'operate' && (
          <>
            <div className="tel-console__primary">
              <TelemetryPanel
                ros={ros}
                updateInterval={appSettings?.telemetry?.updateInterval ?? 100}
                initialShowPanel={true}
              />
            </div>
            {genesisConnected && (
              <SimTelemetryPane compact={true} />
            )}
          </>
        )}

        {activePreset === 'sim' && (
          <>
            <div className="tel-console__primary">
              <SimTelemetryPane />
            </div>
            {rosConnected && (
              <div className="tel-console__strip">
                <span className="tel-console__strip-label">ROS</span>
                <span className={`tel-console__conn ${rosConnected ? 'tel-console__conn--on' : ''}`}>
                  Connected
                </span>
              </div>
            )}
          </>
        )}

        {activePreset === 'split' && (
          <div className="tel-console__split">
            <div className="tel-console__split-pane">
              <div className="tel-console__split-title">SIM</div>
              <SimTelemetryPane />
            </div>
            <div className="tel-console__split-divider" />
            <div className="tel-console__split-pane">
              <div className="tel-console__split-title">ROS</div>
              <TelemetryPanel
                ros={ros}
                updateInterval={appSettings?.telemetry?.updateInterval ?? 100}
                initialShowPanel={true}
              />
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default TelemetryConsole;
