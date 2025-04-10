import React, { useEffect, useState } from 'react';
import RobotViewer from './RobotViewer';
import ControlOverlay from './ControlOverlay';
import LogViewer from './LogViewer';
import SettingsIcon from './SettingsIcon';
import SettingsModal from './SettingsModal';
import ROSLIB from 'roslib';
import io from 'socket.io-client';
import '../styles/App.css';

const App = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [socket, setSocket] = useState(null);
  const [error, setError] = useState(null);
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);

  useEffect(() => {
    let rosInstance = null;
    let socketInstance = null;
    let reconnectTimer = null;

    const connectToROS = () => {
      try {
        // Use localhost if running on the same machine
        const rosUrl = window.location.hostname === 'localhost' || window.location.hostname === '127.0.0.1'
          ? 'ws://localhost:9090'
          : 'ws://192.168.12.147:9090';
        
        console.log(`Attempting to connect to ROS at ${rosUrl}`);
        rosInstance = new ROSLIB.Ros({
          url: rosUrl
        });

        rosInstance.on('connection', () => {
          console.log('Connected to ROS bridge');
          setConnected(true);
          setError(null);
          
          // Test if rosapi service is available
          try {
            const service = new ROSLIB.Service({
              ros: rosInstance,
              name: '/rosapi/get_topics',
              serviceType: 'rosapi_msgs/srv/GetTopics'
            });
            
            const request = new ROSLIB.ServiceRequest({});
            
            service.callService(request, (result) => {
              console.log('ROSAPI service test successful:', result);
            }, (error) => {
              console.warn('ROSAPI service test failed:', error);
              // Don't set an error here, just log it. The LogViewer will handle this case.
            });
          } catch (err) {
            console.warn('Error testing ROSAPI service:', err);
            // Don't set an error here, just log it. The LogViewer will handle this case.
          }
        });

        rosInstance.on('error', (error) => {
          console.error('Error connecting to ROS:', error);
          setConnected(false);
          setError('Failed to connect to ROS bridge. Please ensure rosbridge-server is running.');
          
          // Try to reconnect after 5 seconds
          reconnectTimer = setTimeout(connectToROS, 5000);
        });

        rosInstance.on('close', () => {
          console.log('Connection to ROS closed');
          setConnected(false);
          
          // Try to reconnect after 5 seconds
          reconnectTimer = setTimeout(connectToROS, 5000);
        });

        setRos(rosInstance);
      } catch (err) {
        console.error('Error creating ROS instance:', err);
        setError('Failed to initialize ROS connection. Please check if ROSLIB is properly loaded.');
      }
    };

    // Connect to ROS
    connectToROS();

    // Connect to Socket.IO server
    try {
      console.log('Connecting to Socket.IO server');
      socketInstance = io();
      socketInstance.on('connect', () => {
        console.log('Connected to Socket.IO server');
      });

      socketInstance.on('disconnect', () => {
        console.log('Disconnected from Socket.IO server');
      });

      socketInstance.on('robot_control', (data) => {
        console.log('Received robot control data:', data);
        setControlState(data);
      });

      setSocket(socketInstance);
    } catch (err) {
      console.error('Error connecting to Socket.IO server:', err);
      setError('Failed to connect to Socket.IO server');
    }

    return () => {
      if (rosInstance) rosInstance.close();
      if (socketInstance) socketInstance.disconnect();
      if (reconnectTimer) clearTimeout(reconnectTimer);
    };
  }, []);

  const handleControlChange = (newState) => {
    if (socket && connected) {
      console.log('Sending control command:', newState);
      socket.emit('robot_control', newState);
      
      try {
        // Publish to ROS topic
        const cmdVel = new ROSLIB.Topic({
          ros: ros,
          name: '/cmd_vel',
          messageType: 'geometry_msgs/Twist'
        });

        const twist = new ROSLIB.Message({
          linear: newState.linear,
          angular: newState.angular
        });

        cmdVel.publish(twist);
      } catch (err) {
        console.error('Error publishing to ROS topic:', err);
      }
    }
  };

  const handleOpenSettings = () => {
    setIsSettingsOpen(true);
  };

  const handleCloseSettings = () => {
    setIsSettingsOpen(false);
  };

  return (
    <div className="app">
      <SettingsIcon onClick={handleOpenSettings} />
      <div style={styles.container}>
        {error && (
          <div style={styles.error}>
            {error}
          </div>
        )}
        <div style={styles.viewerContainer}>
          <RobotViewer ros={ros} />
          <ControlOverlay 
            onControlChange={handleControlChange}
            controlState={controlState}
          />
        </div>
        <div style={styles.logContainer}>
          <LogViewer ros={ros} />
        </div>
      </div>
      {isSettingsOpen && <SettingsModal onClose={handleCloseSettings} />}
    </div>
  );
};

const styles = {
  container: {
    display: 'flex',
    width: '100%',
    height: '100%',
  },
  viewerContainer: {
    flex: '2',
    position: 'relative',
  },
  logContainer: {
    flex: '1',
    borderLeft: '1px solid #333',
    padding: '10px',
  },
  error: {
    position: 'fixed',
    top: '10px',
    left: '50%',
    transform: 'translateX(-50%)',
    backgroundColor: '#ff4444',
    color: 'white',
    padding: '10px 20px',
    borderRadius: '5px',
    zIndex: 1000,
  },
};

export default App; 