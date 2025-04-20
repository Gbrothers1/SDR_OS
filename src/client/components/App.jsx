import React, { useEffect, useState } from 'react';
import RobotViewer from './RobotViewer';
import ControlOverlay from './ControlOverlay';
import LogViewer from './LogViewer';
import SettingsIcon from './SettingsIcon';
import SettingsModal from './SettingsModal';
import TelemetryPanel from './TelemetryPanel';
import ROSLIB from 'roslib';
import io from 'socket.io-client';
import '../styles/App.css';
import SplashScreen from './SplashScreen';

const App = () => {
  const [initialized, setInitialized] = useState(false);
  const [isLoading, setIsLoading] = useState(true);
  const [ros, setRos] = useState(null);
  const [socket, setSocket] = useState(null);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [isLogViewerVisible, setIsLogViewerVisible] = useState(true);

  useEffect(() => {
    console.log('App mounted - Showing splash screen');
    setRos(null);
    setSocket(null);
    setConnected(false);
    setError(null);
    setInitialized(false);
    setIsLoading(true);
  }, []);

  const handleSplashComplete = async () => {
    console.log('Splash screen complete - Starting initialization');
    try {
      const rosBridgeUrl = localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090';
      const socketUrl = localStorage.getItem('socketUrl') || 'http://localhost:3000';

      console.log('Initializing connections:', { rosBridgeUrl, socketUrl });

      const newRos = new ROSLIB.Ros({
        url: rosBridgeUrl
      });

      await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('ROS connection timeout')), 5000);

        newRos.on('connection', () => {
          clearTimeout(timeout);
          console.log('ROS connection established');
          setConnected(true);
          resolve();
        });

        newRos.on('error', (error) => {
          clearTimeout(timeout);
          console.error('ROS connection error:', error);
          reject(error);
        });
      });

      setRos(newRos);

      console.log('Connecting to Socket.IO');
      const newSocket = io(socketUrl);
      
      await new Promise((resolve, reject) => {
        const timeout = setTimeout(() => reject(new Error('Socket.IO connection timeout')), 5000);

        newSocket.on('connect', () => {
          clearTimeout(timeout);
          console.log('Socket.IO connected');
          resolve();
        });

        newSocket.on('connect_error', (error) => {
          clearTimeout(timeout);
          console.error('Socket.IO connection error:', error);
          reject(error);
        });
      });

      setSocket(newSocket);
      setInitialized(true);
      setIsLoading(false);
      console.log('All systems initialized');

    } catch (error) {
      console.error('Initialization error:', error);
      setError(`Initialization failed: ${error.message}`);
      setInitialized(true);
      setIsLoading(false);
    }
  };

  if (!initialized || isLoading) {
    console.log('Rendering splash screen');
    return <SplashScreen onComplete={handleSplashComplete} />;
  }

  console.log('Rendering main application');
  return (
    <div className="app">
      {error && (
        <div className="error-banner">
          {error}
          <button onClick={() => window.location.reload()}>
            Retry
          </button>
        </div>
      )}
      <div className="viewer-container">
        <RobotViewer ros={ros} />
        <ControlOverlay ros={ros} socket={socket} />
        <TelemetryPanel ros={ros} />
      </div>
      {isLogViewerVisible && (
        <LogViewer ros={ros} />
      )}
      <SettingsIcon onClick={() => setIsSettingsOpen(true)} />
      {isSettingsOpen && (
        <SettingsModal
          onClose={() => setIsSettingsOpen(false)}
        />
      )}
    </div>
  );
};

export default App; 