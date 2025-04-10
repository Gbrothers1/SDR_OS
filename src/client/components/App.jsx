import React, { useEffect, useState } from 'react';
import RobotViewer from './RobotViewer';
import ControlOverlay from './ControlOverlay';
import LogViewer from './LogViewer';
import ROSLIB from 'roslib';
import io from 'socket.io-client';

const App = () => {
  const [ros, setRos] = useState(null);
  const [connected, setConnected] = useState(false);
  const [socket, setSocket] = useState(null);
  const [error, setError] = useState(null);
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });

  useEffect(() => {
    let rosInstance = null;
    let socketInstance = null;
    let reconnectTimer = null;

    const connectToROS = () => {
      try {
        console.log('Attempting to connect to ROS at ws://192.168.12.147:9090');
        rosInstance = new ROSLIB.Ros({
          url: 'ws://192.168.12.147:9090'
        });

        rosInstance.on('connection', () => {
          console.log('Connected to ROS bridge');
          setConnected(true);
          setError(null);
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

  return (
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