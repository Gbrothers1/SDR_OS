import React, { useState } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/SimulationStarter.css';

const SimulationStarter = () => {
  const {
    genesisConnected,
    socket,
    scriptStatus,
    currentScriptName,
    scriptError,
    frameStats
  } = useGenesis();
  const [selectedFile, setSelectedFile] = useState(null);
  const [fileContent, setFileContent] = useState('');
  const [isStarting, setIsStarting] = useState(false);
  const [status, setStatus] = useState('');

  const getStatusColor = () => {
    switch (scriptStatus) {
      case 'idle': return 'var(--color-text-muted)';
      case 'loading': return 'var(--color-accent-amber)';
      case 'running': return 'var(--color-accent-green)';
      case 'paused': return 'var(--color-accent-amber)';
      case 'error': return 'var(--color-accent-red)';
      default: return 'var(--color-text-muted)';
    }
  };

  const getStatusText = () => {
    if (scriptStatus === 'error' && scriptError) {
      return `ERROR: ${scriptError}`;
    }
    return scriptStatus.toUpperCase();
  };

  const handleFileSelect = (event) => {
    const file = event.target.files[0];
    if (file) {
      setSelectedFile(file);
      const reader = new FileReader();
      reader.onload = (e) => {
        setFileContent(e.target.result);
      };
      reader.readAsText(file);
    }
  };

  const handleStartSimulation = async () => {
    if (!selectedFile && !fileContent) {
      setStatus('Error: Please select or paste a Genesis Python file');
      return;
    }

    setIsStarting(true);
    setStatus('Starting simulation...');

    try {
      if (!socket) {
        setStatus('Error: Socket.IO not available. Make sure server.js is running.');
        setIsStarting(false);
        return;
      }

      socket.emit('genesis_load_script', {
        script_content: fileContent || '',
        script_name: selectedFile?.name || 'simulation.py'
      });

      setStatus('Simulation started! Waiting for frames...');

      setTimeout(() => {
        setStatus('');
        setIsStarting(false);
      }, 3000);

    } catch (error) {
      setStatus(`Error: ${error.message}`);
      setIsStarting(false);
    }
  };

  const handlePasteScript = () => {
    const content = prompt('Paste your Genesis Python script:');
    if (content) {
      setFileContent(content);
      setSelectedFile({ name: 'pasted_script.py' });
    }
  };

  return (
    <div className="sim-starter">
      {/* Script Status Display */}
      <div className="sim-status-display">
        <div className="sim-status-display__header">Script Status:</div>
        <div className="sim-status-display__row">
          <div className="sim-status-dot" style={{ background: getStatusColor() }} />
          <div className="sim-status-text">{getStatusText()}</div>
        </div>
        {currentScriptName && (
          <div className="sim-script-name">Script: {currentScriptName}</div>
        )}
        {frameStats && (
          <div className="sim-frame-stats">
            FPS: {frameStats.frame_rate?.toFixed(1) || '0.0'} |
            Frames: {frameStats.frames_sent || 0} |
            Camera: {frameStats.camera_available ? '\u2713' : '\u2717'}
          </div>
        )}
      </div>

      {status && (
        <div className={`sim-status-message ${status.includes('Error') ? 'sim-status-message--error' : 'sim-status-message--success'}`}>
          {status}
        </div>
      )}

      <div>
        <div className="sim-file-section__label">Load Genesis Python File:</div>
        <input
          type="file"
          accept=".py"
          onChange={handleFileSelect}
          className="sim-file-input"
        />
        {selectedFile && (
          <div className="sim-file-selected">Selected: {selectedFile.name}</div>
        )}
      </div>

      <button className="sdr-btn" onClick={handlePasteScript}>
        Or Paste Script
      </button>

      <button
        className={`sdr-btn${(!selectedFile && !fileContent) || isStarting ? '' : ' sdr-btn--primary'}`}
        onClick={handleStartSimulation}
        disabled={isStarting || (!selectedFile && !fileContent)}
      >
        {isStarting ? 'Starting...' : 'Start Simulation'}
      </button>

      {fileContent && (
        <div className="sim-preview">
          <div className="sim-preview__label">Script Preview:</div>
          <pre className="sim-preview__code">
            {fileContent.substring(0, 500)}{fileContent.length > 500 ? '...' : ''}
          </pre>
        </div>
      )}
    </div>
  );
};

export default SimulationStarter;
