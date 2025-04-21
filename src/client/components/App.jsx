import React, { useEffect, useState, useCallback } from 'react';
import RobotViewer from './RobotViewer';
import ControlOverlay from './ControlOverlay';
import LogViewer from './LogViewer';
import SettingsIcon from './SettingsIcon';
import SettingsModal from './SettingsModal';
import TelemetryPanel from './TelemetryPanel';
import SplashScreen from './SplashScreen';
import ROSLIB from 'roslib';
import io from 'socket.io-client';
import '../styles/App.css';

const App = () => {
  const [isLoading, setIsLoading] = useState(true);
  const [ros, setRos] = useState(null);
  const [socket, setSocket] = useState(null);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [isLogViewerVisible, setIsLogViewerVisible] = useState(false);
  const [displaySettings, setDisplaySettings] = useState({
    showLogViewer: false,
    showTelemetryPanel: true
  });
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });
  
  // Track if this client has a gamepad connected
  const [hasGamepad, setHasGamepad] = useState(false);

  // Store the full settings object in state to easily pass down parts
  const [appSettings, setAppSettings] = useState(null);

  const initializeConnections = async () => {
    try {
      const rosBridgeUrl = localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090';
      console.log('Attempting ROS connection to:', rosBridgeUrl);

      const newRos = new ROSLIB.Ros({
        url: rosBridgeUrl
      });

      let connectionResolved = false;
      
      newRos.on('connection', () => {
        console.log('Connected to ROS bridge');
        setConnected(true);
        setError(null);
        connectionResolved = true;
      });

      newRos.on('error', (error) => {
        console.error('ROS connection error:', error);
        if (!connectionResolved) {
          connectionResolved = true;
          throw new Error('Failed to connect to ROS bridge server');
        }
      });

      newRos.on('close', () => {
        console.log('ROS connection closed');
        setConnected(false);
      });

      await Promise.race([
        new Promise(resolve => setTimeout(() => {
          if (!connectionResolved) {
            connectionResolved = true;
            throw new Error('ROS connection timeout');
          }
        }, 5000)),
        new Promise(resolve => {
          const checkConnection = setInterval(() => {
            if (connectionResolved) {
              clearInterval(checkConnection);
              resolve();
            }
          }, 100);
        })
      ]);

      setRos(newRos);

      const socketUrl = localStorage.getItem('socketUrl') || 'http://localhost:3000';
      console.log('Connecting to Socket.IO:', socketUrl);
      const newSocket = io(socketUrl);
      setSocket(newSocket);
      
      // Listen for gamepad connection events globally
      window.addEventListener('gamepadconnected', () => {
        console.log('Gamepad connected to this client');
        setHasGamepad(true);
      });
      
      window.addEventListener('gamepaddisconnected', () => {
        console.log('Gamepad disconnected from this client');
        setHasGamepad(false);
      });

      return true;
    } catch (error) {
      console.error('Connection initialization error:', error);
      setError(`Failed to initialize: ${error.message}`);
      return false;
    }
  };

  const toggleLogViewer = () => {
    setIsLogViewerVisible(prev => !prev);
    setDisplaySettings(prev => ({
      ...prev,
      showLogViewer: !prev.showLogViewer
    }));
  };

  const handleSplashComplete = async () => {
    console.log('Splash screen complete, initializing connections...');
    const success = await initializeConnections();
    if (success) {
      console.log('Connections initialized successfully');
      setIsLoading(false);
    } else {
      console.error('Failed to initialize connections');
      setError('Failed to initialize system connections');
    }
  };

  // Memoize handleControlChange to stabilize the prop passed to ControlOverlay
  const handleControlChange = useCallback((newState) => {
    // Update local control state
    setControlState(newState);
  }, []); // No dependencies, function doesn't rely on changing state/props

  const handleSettingsSave = (newSettings) => {
    // Update display settings state
    setDisplaySettings({
      showLogViewer: displaySettings.showLogViewer,
      showTelemetryPanel: newSettings.telemetry.showTelemetryPanel
    });
    // Update full settings state
    setAppSettings(newSettings);
    
    // Save display settings (for initial load)
    localStorage.setItem('displaySettings', JSON.stringify({
      showLogViewer: displaySettings.showLogViewer,
      showTelemetryPanel: newSettings.telemetry.showTelemetryPanel
    }));
    
    // Save the full settings object (used by Settings.jsx and passed down)
    localStorage.setItem('robotControllerSettings', JSON.stringify(newSettings));
  };

  useEffect(() => {
    // Load full settings on initial mount
    const savedFullSettings = localStorage.getItem('robotControllerSettings');
    if (savedFullSettings) {
      try {
        setAppSettings(JSON.parse(savedFullSettings));
      } catch (e) {
        console.error('Error loading full settings:', e);
      }
    }
    // Load display settings (redundant? Could derive from full settings)
    const savedDisplaySettings = localStorage.getItem('displaySettings');
    if (savedDisplaySettings) {
      try {
        const settings = JSON.parse(savedDisplaySettings);
        setDisplaySettings(settings);
        setIsLogViewerVisible(settings.showLogViewer);
      } catch (e) {
        console.error('Error loading display settings:', e);
      }
    }
    
    // Check if a gamepad is already connected
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    if (gamepads.some(gamepad => gamepad !== null)) {
      setHasGamepad(true);
    }
    
    return () => {
      window.removeEventListener('gamepadconnected', () => setHasGamepad(true));
      window.removeEventListener('gamepaddisconnected', () => setHasGamepad(false));
    };
  }, []);

  useEffect(() => {
    localStorage.setItem('displaySettings', JSON.stringify(displaySettings));
  }, [displaySettings]);

  if (isLoading) {
    return <SplashScreen onComplete={handleSplashComplete} />;
  }

  return (
    <div className="app">
      {error && (
        <div className="error-banner">
          {error}
          <button onClick={() => window.location.reload()}>
            Retry Connection
          </button>
        </div>
      )}
      
      <div className="app-container">
        <div className="viewer-container">
          <RobotViewer 
            ros={ros} 
            // Pass throttle rate from state (or default)
            tfThrottleRate={appSettings?.visualization?.tfThrottleRate ?? 10}
          />
          <ControlOverlay 
            ros={ros} 
            socket={socket} 
            controlState={controlState}
            onControlChange={handleControlChange}
          />
          {displaySettings.showTelemetryPanel && 
            <TelemetryPanel 
              ros={ros} 
              // Pass updateInterval from settings state
              updateInterval={appSettings?.telemetry?.updateInterval ?? 100} // Default 100ms
            />
          }
          
          <div className="toggle-buttons">
            <button className="toggle-log" onClick={toggleLogViewer}>
              {isLogViewerVisible ? 'Hide Logs' : 'Show Logs'}
            </button>
          </div>
        </div>
        
        {isLogViewerVisible && (
          <div className="log-container">
            <LogViewer 
              ros={ros} 
              // Pass subscription settings from state (or default)
              subscribeRosout={appSettings?.logging?.subscribeRosout ?? true}
              subscribeDiagnostics={appSettings?.logging?.subscribeDiagnostics ?? true}
            />
          </div>
        )}
      </div>
      
      <SettingsIcon onClick={() => setIsSettingsOpen(true)} />
      
      {isSettingsOpen && (
        <SettingsModal
          isOpen={isSettingsOpen}
          onClose={() => setIsSettingsOpen(false)}
          onSave={handleSettingsSave}
        />
      )}
    </div>
  );
};

export default App; 