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
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });

  useEffect(() => {
    // Connect to ROS
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS bridge');
      setConnected(true);
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
      setConnected(false);
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS closed');
      setConnected(false);
    });

    setRos(rosInstance);

    // Connect to Socket.IO server
    const socketInstance = io();
    socketInstance.on('connect', () => {
      console.log('Connected to Socket.IO server');
    });

    socketInstance.on('robot_control', (data) => {
      setControlState(data);
    });

    setSocket(socketInstance);

    return () => {
      if (rosInstance) rosInstance.close();
      if (socketInstance) socketInstance.disconnect();
    };
  }, []);

  const handleControlChange = (newState) => {
    if (socket && connected) {
      socket.emit('robot_control', newState);
      
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
    }
  };

  return (
    <div style={styles.container}>
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
};

export default App; 