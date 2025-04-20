import React, { useState, useEffect, useCallback } from 'react';
import '../styles/SplashScreen.css';

const SplashScreen = ({ onComplete }) => {
  const [stage, setStage] = useState('initial'); // 'initial', 'diagnostic', 'loading', 'ready'
  const [progress, setProgress] = useState(0);
  const [logs, setLogs] = useState([]);
  const [diagnostics, setDiagnostics] = useState({
    ros: { status: 'pending', label: 'ROS Bridge' },
    sensors: { status: 'pending', label: 'Sensor Array' },
    motors: { status: 'pending', label: 'Motor Controllers' },
    camera: { status: 'pending', label: 'Camera Systems' }
  });
  const [isSystemActive, setIsSystemActive] = useState(false);

  // Enhanced speech synthesis with sound effects
  const playSound = (type) => {
    const oscillator = new (window.AudioContext || window.webkitAudioContext)().createOscillator();
    const gainNode = new (window.AudioContext || window.webkitAudioContext)().createGain();
    
    oscillator.connect(gainNode);
    gainNode.connect(new (window.AudioContext || window.webkitAudioContext)().destination);

    switch(type) {
      case 'startup':
        oscillator.frequency.setValueAtTime(440, 0);
        oscillator.frequency.linearRampToValueAtTime(880, 0.1);
        gainNode.gain.setValueAtTime(0.5, 0);
        gainNode.gain.linearRampToValueAtTime(0, 0.1);
        break;
      case 'success':
        oscillator.frequency.setValueAtTime(587.33, 0);
        oscillator.frequency.linearRampToValueAtTime(880, 0.1);
        gainNode.gain.setValueAtTime(0.2, 0);
        gainNode.gain.linearRampToValueAtTime(0, 0.2);
        break;
      case 'error':
        oscillator.frequency.setValueAtTime(440, 0);
        oscillator.frequency.linearRampToValueAtTime(220, 0.1);
        gainNode.gain.setValueAtTime(0.3, 0);
        gainNode.gain.linearRampToValueAtTime(0, 0.2);
        break;
    }

    oscillator.start();
    oscillator.stop(0.2);
  };

  const speak = useCallback((text, priority = 'normal') => {
    if ('speechSynthesis' in window) {
      const utterance = new SpeechSynthesisUtterance(text);
      utterance.voice = speechSynthesis.getVoices()
        .find(voice => voice.name.includes('Female')) || speechSynthesis.getVoices()[0];
      utterance.pitch = 1.2;
      utterance.rate = 0.9;
      
      switch(priority) {
        case 'high':
          utterance.volume = 1.0;
          playSound('startup');
          break;
        case 'success':
          utterance.volume = 0.8;
          playSound('success');
          break;
        case 'error':
          utterance.volume = 0.9;
          playSound('error');
          break;
        default:
          utterance.volume = 0.7;
      }
      
      speechSynthesis.speak(utterance);
    }
  }, []);

  const addLog = useCallback((message, type = 'info') => {
    setLogs(prev => [...prev, { message, type, timestamp: new Date().toISOString() }]);
    speak(message, type);
  }, [speak]);

  const runDiagnostics = useCallback(async () => {
    console.log('Running diagnostics...');
    setStage('diagnostic');
    addLog('Initiating system diagnostics...', 'high');

    const runSingleDiagnostic = async (key, delay) => {
      console.log(`Running diagnostic for: ${key}`);
      await new Promise(resolve => setTimeout(resolve, delay));
      const success = Math.random() > 0.1;
      setDiagnostics(prev => ({
        ...prev,
        [key]: { ...prev[key], status: success ? 'success' : 'error' }
      }));
      addLog(
        `${diagnostics[key].label} diagnostic ${success ? 'passed' : 'failed'}`,
        success ? 'success' : 'error'
      );
      return success;
    };

    const results = await Promise.all([
      runSingleDiagnostic('ros', 1000),
      runSingleDiagnostic('sensors', 2000),
      runSingleDiagnostic('motors', 3000),
      runSingleDiagnostic('camera', 4000)
    ]);

    const allPassed = results.every(result => result);
    if (allPassed) {
      addLog('All diagnostics passed. System ready for initialization.', 'success');
      setStage('ready');
    } else {
      addLog('Diagnostic failures detected. Manual override required.', 'error');
      setStage('ready'); // Allow continue anyway for demo purposes
    }
  }, [addLog, diagnostics]);

  const startSystem = useCallback(async () => {
    if (stage === 'ready') {
      setStage('loading');
      setIsSystemActive(true);
      addLog('System initialization sequence activated', 'high');

      const steps = [
        { message: 'Establishing ROS bridge connection', progress: 20 },
        { message: 'Calibrating sensor arrays', progress: 40 },
        { message: 'Initializing motor controllers', progress: 60 },
        { message: 'Loading navigation systems', progress: 80 },
        { message: 'System initialization complete', progress: 100 }
      ];

      for (const step of steps) {
        await new Promise(resolve => setTimeout(resolve, 1000));
        setProgress(step.progress);
        addLog(step.message, step.progress === 100 ? 'success' : 'info');
      }

      await new Promise(resolve => setTimeout(resolve, 1000));
      onComplete();
    }
  }, [stage, addLog, onComplete]);

  useEffect(() => {
    console.log('SplashScreen mounted, stage:', stage);
    
    // Ensure the component is mounted before starting diagnostics
    const timeoutId = setTimeout(() => {
      console.log('Starting initial diagnostics');
      setStage('diagnostic');
    }, 100);

    return () => clearTimeout(timeoutId);
  }, []);

  useEffect(() => {
    if (stage === 'initial') {
      addLog('System diagnostic check required', 'high');
      addLog('Press START to begin diagnostic sequence', 'info');
    }
  }, [stage, addLog]);

  return (
    <div className="splash-screen">
      <div className="splash-content">
        <h1 className="splash-title">ROBOT CONTROL INTERFACE</h1>
        <div className="splash-subtitle">v1.0.0</div>

        {stage === 'initial' && (
          <button 
            className="diagnostic-button"
            onClick={() => runDiagnostics()}
          >
            START DIAGNOSTIC
          </button>
        )}

        {stage === 'diagnostic' && (
          <div className="diagnostic-panel">
            {Object.entries(diagnostics).map(([key, value]) => (
              <div key={key} className={`diagnostic-item ${value.status}`}>
                <span className="diagnostic-label">{value.label}</span>
                <span className="diagnostic-status">
                  {value.status === 'pending' ? '...' : value.status === 'success' ? 'OK' : 'FAIL'}
                </span>
              </div>
            ))}
          </div>
        )}

        {stage === 'ready' && (
          <button 
            className={`start-button ${isSystemActive ? 'active' : ''}`}
            onClick={startSystem}
            disabled={isSystemActive}
          >
            INITIALIZE SYSTEM
          </button>
        )}

        {stage === 'loading' && (
          <div className="progress-container">
            <div 
              className="progress-bar" 
              style={{ width: `${progress}%` }}
            />
            <div className="progress-text">{progress}%</div>
          </div>
        )}

        <div className="log-window">
          <div className="log-header">
            SYSTEM LOG
            <div className="log-status">
              {stage.toUpperCase()} MODE
            </div>
          </div>
          <div className="log-content">
            {logs.map((log, index) => (
              <div key={index} className={`log-entry ${log.type}`}>
                <span className="timestamp">
                  {new Date(log.timestamp).toLocaleTimeString()}
                </span>
                <span className="message">{log.message}</span>
              </div>
            ))}
          </div>
        </div>
      </div>
    </div>
  );
};

export default SplashScreen; 