import React, { useState, useEffect, useCallback, useRef } from 'react';
import '../styles/SplashScreen.css';

const SplashScreen = ({ onComplete }) => {
  const [stage, setStage] = useState('initial'); // initial, connecting, override, ready, launching
  const [connectionStatus, setConnectionStatus] = useState({
    ros: { status: 'OFFLINE', code: 'COMM2101-A', ready: false },
    navigation: { status: 'STANDBY', code: 'NAV1001-B', ready: false },
    sensors: { status: 'INITIALIZING', code: 'SENS441-C', ready: false },
    control: { status: 'WAITING', code: 'CTRL992-D', ready: false }
  });
  const [systemLogs, setSystemLogs] = useState([]);
  const [errorState, setErrorState] = useState(null);
  const terminalRef = useRef(null);
  const [autoMode, setAutoMode] = useState(true);
  const [isSpeaking, setIsSpeaking] = useState(false);
  const speechEnabled = localStorage.getItem('enableSpeech') !== 'false';
  const [voices, setVoices] = useState([]);
  const [selectedVoice, setSelectedVoice] = useState(null);
  const savedVoiceName = localStorage.getItem('selectedVoice');

  // Initialize voices when component mounts
  useEffect(() => {
    const loadVoices = () => {
      const availableVoices = window.speechSynthesis.getVoices();
      setVoices(availableVoices);
      
      // Try to find the saved voice first
      if (savedVoiceName) {
        const savedVoice = availableVoices.find(voice => voice.name === savedVoiceName);
        if (savedVoice) {
          setSelectedVoice(savedVoice);
          return;
        }
      }
      
      // If saved voice not found, find the best English voice
      const englishVoices = availableVoices.filter(voice => 
        voice.lang.startsWith('en-')
      );
      
      if (englishVoices.length > 0) {
        // Prefer female voices
        const femaleVoice = englishVoices.find(voice => 
          voice.name.includes('Female') || 
          voice.name.includes('female') ||
          voice.name.includes('Samantha') ||
          voice.name.includes('Google US English Female')
        );
        
        setSelectedVoice(femaleVoice || englishVoices[0]);
      } else if (availableVoices.length > 0) {
        setSelectedVoice(availableVoices[0]);
      }
    };
    
    // Load voices immediately if available
    loadVoices();
    
    // Also listen for voices to be loaded
    if (window.speechSynthesis.onvoiceschanged !== undefined) {
      window.speechSynthesis.onvoiceschanged = loadVoices;
    }
    
    return () => {
      if (window.speechSynthesis.onvoiceschanged !== undefined) {
        window.speechSynthesis.onvoiceschanged = null;
      }
    };
  }, [savedVoiceName]);

  // Enhanced speech synthesis
  const speak = useCallback((text, priority = 'normal') => {
    if (!speechEnabled) {
      playSound(priority === 'error' ? 'error' : priority === 'success' ? 'success' : 'startup');
      return;
    }

    if ('speechSynthesis' in window) {
      // Cancel any ongoing speech
      window.speechSynthesis.cancel();

      if (!autoMode && priority !== 'high' && priority !== 'success') {
        // In manual mode, only play sounds for most actions
        playSound(priority === 'error' ? 'error' : priority === 'success' ? 'success' : 'startup');
        return;
      }

      const utterance = new SpeechSynthesisUtterance(text);
      
      // Use the selected voice if available
      if (selectedVoice) {
        utterance.voice = selectedVoice;
        utterance.lang = selectedVoice.lang;
      }
      
      // Adjust speech parameters for better quality
      utterance.pitch = 1.0;
      utterance.rate = 0.9;  // Slightly slower for clarity
      
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

      setIsSpeaking(true);
      utterance.onend = () => setIsSpeaking(false);
      utterance.onerror = () => setIsSpeaking(false);
      
      window.speechSynthesis.speak(utterance);
    }
  }, [speechEnabled, autoMode, selectedVoice]);

  // Sound effects for feedback
  const playSound = (type) => {
    const audioContext = new (window.AudioContext || window.webkitAudioContext)();
    const oscillator = audioContext.createOscillator();
    const gainNode = audioContext.createGain();
    
    oscillator.connect(gainNode);
    gainNode.connect(audioContext.destination);

    switch(type) {
      case 'startup':
        oscillator.frequency.setValueAtTime(440, audioContext.currentTime);
        oscillator.frequency.exponentialRampToValueAtTime(880, audioContext.currentTime + 0.1);
        gainNode.gain.setValueAtTime(0.3, audioContext.currentTime);
        gainNode.gain.exponentialRampToValueAtTime(0.01, audioContext.currentTime + 0.3);
        break;
      case 'success':
        oscillator.frequency.setValueAtTime(587.33, audioContext.currentTime);
        oscillator.frequency.exponentialRampToValueAtTime(880, audioContext.currentTime + 0.1);
        gainNode.gain.setValueAtTime(0.2, audioContext.currentTime);
        gainNode.gain.exponentialRampToValueAtTime(0.01, audioContext.currentTime + 0.2);
        break;
      case 'error':
        oscillator.type = 'sawtooth';
        oscillator.frequency.setValueAtTime(440, audioContext.currentTime);
        oscillator.frequency.exponentialRampToValueAtTime(220, audioContext.currentTime + 0.2);
        gainNode.gain.setValueAtTime(0.3, audioContext.currentTime);
        gainNode.gain.exponentialRampToValueAtTime(0.01, audioContext.currentTime + 0.2);
        break;
    }

    oscillator.start();
    oscillator.stop(audioContext.currentTime + 0.3);
  };

  const addSystemLog = useCallback((message, type = 'info') => {
    const newLog = {
      timestamp: new Date().toISOString(),
      message,
      type,
      id: `${Date.now()}-${Math.random()}`
    };
    setSystemLogs(prev => [...prev, newLog]);
    
    // Only speak important messages in manual mode
    if (autoMode || type === 'high' || type === 'success' || type === 'error') {
      speak(message, type);
    } else {
      playSound(type === 'error' ? 'error' : type === 'success' ? 'success' : 'startup');
    }
    
    if (terminalRef.current) {
      terminalRef.current.scrollTop = terminalRef.current.scrollHeight;
    }
  }, [speak, autoMode]);

  // Automatic system activation
  const activateSystem = async (system) => {
    const message = `Activating ${system.toUpperCase()} subsystem`;
    addSystemLog(message, 'info');
    
    setConnectionStatus(prev => ({
      ...prev,
      [system]: { ...prev[system], status: 'CONNECTING' }
    }));

    await new Promise(resolve => setTimeout(resolve, 1500));
    
    // Simulate success most of the time, with occasional failures
    const success = Math.random() > 0.1;
    
    if (success) {
      setConnectionStatus(prev => ({
        ...prev,
        [system]: { 
          ...prev[system], 
          status: 'ONLINE',
          ready: true
        }
      }));
      
      addSystemLog(`${system.toUpperCase()} subsystem online`, 'success');
    } else {
      setConnectionStatus(prev => ({
        ...prev,
        [system]: { 
          ...prev[system], 
          status: 'ERROR',
          ready: false
        }
      }));
      
      addSystemLog(`${system.toUpperCase()} subsystem initialization failed`, 'error');
      setAutoMode(false);
      setStage('override');
      setErrorState({
        code: `${system.toUpperCase()}_INIT_FAILURE`,
        message: 'Automatic initialization failed - manual override required'
      });
      return false;
    }
    
    return true;
  };

  // Manual system activation
  const handleSystemActivation = async (system) => {
    addSystemLog(`Manual activation of ${system} subsystem initiated`, 'warning');
    
    setConnectionStatus(prev => ({
      ...prev,
      [system]: { ...prev[system], status: 'CONNECTING' }
    }));

    await new Promise(resolve => setTimeout(resolve, 1500));
    
    // Simulate success most of the time
    const success = Math.random() > 0.05;
    
    if (success) {
      setConnectionStatus(prev => ({
        ...prev,
        [system]: { 
          ...prev[system], 
          status: 'ONLINE',
          ready: true
        }
      }));
      
      addSystemLog(`${system.toUpperCase()} subsystem activated successfully`, 'success');
    } else {
      setConnectionStatus(prev => ({
        ...prev,
        [system]: { 
          ...prev[system], 
          status: 'ERROR',
          ready: false
        }
      }));
      
      addSystemLog(`${system.toUpperCase()} activation failed - retry required`, 'error');
      return false;
    }
    
    // Check if all systems are ready
    const allReady = Object.values(connectionStatus).every(status => status.ready);
    
    if (allReady) {
      addSystemLog('All systems operational - Ready for launch', 'success');
      setStage('ready');
    }
    
    return true;
  };

  const handleManualOverride = () => {
    addSystemLog('MANUAL OVERRIDE INITIATED - BYPASSING INITIALIZATION', 'warning');
    setAutoMode(false);
    // Set all systems to ready state
    setConnectionStatus(prev => {
      const newStatus = {};
      Object.keys(prev).forEach(system => {
        newStatus[system] = {
          ...prev[system],
          status: 'BYPASSED',
          ready: true
        };
      });
      return newStatus;
    });
    // Add a brief delay before launching
    setTimeout(() => {
      addSystemLog('LAUNCHING MAIN SYSTEM INTERFACE', 'success');
      setStage('launching');
      setTimeout(() => {
        onComplete();
      }, 1000);
    }, 500);
  };

  const handleSystemLaunch = () => {
    addSystemLog('LAUNCHING MAIN SYSTEM INTERFACE', 'success');
    setStage('launching');
    setTimeout(() => {
      onComplete();
    }, 1500);
  };

  // Initialize the system
  useEffect(() => {
    const initializeSystem = async () => {
      addSystemLog('SYSTEM INITIALIZATION SEQUENCE STARTED', 'high');
      setStage('connecting');
      
      try {
        // Try automatic initialization first
        if (autoMode) {
          addSystemLog('Automatic initialization in progress', 'info');
          
          // Activate each system in sequence
          const systems = Object.keys(connectionStatus);
          let allSuccess = true;
          
          for (const system of systems) {
            const success = await activateSystem(system);
            if (!success) {
              allSuccess = false;
              break;
            }
            // Short delay between system activations
            await new Promise(resolve => setTimeout(resolve, 500));
          }
          
          if (allSuccess) {
            addSystemLog('Automatic initialization complete - All systems online', 'success');
            setStage('ready');
          }
        } else {
          // Manual override mode
          addSystemLog('MANUAL OVERRIDE MODE ACTIVE - Activate each system manually', 'warning');
          setStage('override');
        }
      } catch (error) {
        addSystemLog('System initialization error - Manual override required', 'error');
        setErrorState({
          code: 'INIT_FAILURE',
          message: 'Manual system activation required'
        });
        setAutoMode(false);
        setStage('override');
      }
    };

    initializeSystem();
  }, [addSystemLog]);

  // Prevent multiple launches
  useEffect(() => {
    if (stage === 'launching') {
      const launchTimeout = setTimeout(() => {
        setStage('complete');
      }, 1500);
      return () => clearTimeout(launchTimeout);
    }
  }, [stage]);

  // Add debug logging
  useEffect(() => {
    console.log('Current stage:', stage);
    console.log('Connection status:', connectionStatus);
  }, [stage, connectionStatus]);

  // Add a useEffect to check if all systems are ready and update the stage
  useEffect(() => {
    // Check if all systems are ready
    const allReady = Object.values(connectionStatus).every(status => status.ready);
    
    if (allReady && stage === 'override') {
      addSystemLog('All systems operational - Ready for launch', 'success');
      setStage('ready');
    }
  }, [connectionStatus, stage, addSystemLog]);

  return (
    <div className="splash-container">
      <div className="splash-logo-only">
        <div className="splash-logo-text">SDR</div>
      </div>
    </div>
  );
};

export default SplashScreen; 
