import React, { useEffect, useState } from 'react';
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

  const handleControlChange = (newState) => {
    // Update local control state
    setControlState(newState);
  };

  const handleSettingsSave = (settings) => {
    // Update display settings if provided
    if (settings.displaySettings) {
      setDisplaySettings(settings.displaySettings);
    }
    
    // Reconnect to ROS and Socket.IO with new settings
    if (settings.rosbridgeUrl || settings.socketUrl) {
      // Close existing connections
      if (ros) {
        ros.close();
      }
      if (socket) {
        socket.disconnect();
      }
      
      // Reinitialize connections
      initializeConnections();
    }
  };

  useEffect(() => {
    const savedSettings = localStorage.getItem('displaySettings');
    if (savedSettings) {
      try {
        const settings = JSON.parse(savedSettings);
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
          <RobotViewer ros={ros} />
          <ControlOverlay 
            ros={ros} 
            socket={socket} 
            controlState={controlState}
            onControlChange={handleControlChange}
          />
          {displaySettings.showTelemetryPanel && <TelemetryPanel ros={ros} />}
          
          <div className="toggle-buttons">
            <button className="toggle-log" onClick={toggleLogViewer}>
              {isLogViewerVisible ? 'Hide Logs' : 'Show Logs'}
            </button>
          </div>
        </div>
        
        {isLogViewerVisible && (
          <div className="log-container">
            <LogViewer ros={ros} />
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