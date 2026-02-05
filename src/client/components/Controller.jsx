import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import '../styles/Controller.css';

const Controller = ({ ros }) => {
  const [isConnected, setIsConnected] = useState(false);
  const [error, setError] = useState(null);
  const lastLogTime = useRef(Date.now());
  const logInterval = 1000; // Log only once per second

  useEffect(() => {
    if (!ros) {
      console.warn('ROS connection not available for Controller');
      setError('ROS connection not available');
      return;
    }

    // Create a subscriber for cmd_vel
    const cmdVelSubscriber = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist'
    });

    // Create a publisher for cmd_vel
    const cmdVelPublisher = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/msg/Twist'
    });

    // Subscribe to cmd_vel topic
    cmdVelSubscriber.subscribe((message) => {
      const now = Date.now();
      if (now - lastLogTime.current >= logInterval) {
        console.log('Received cmd_vel message:', message);
        lastLogTime.current = now;
      }
    });

    // Set up keyboard controls
    const handleKeyDown = (event) => {
      const twist = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      });

      switch (event.key) {
        case 'ArrowUp':
          twist.linear.x = 0.5;
          break;
        case 'ArrowDown':
          twist.linear.x = -0.5;
          break;
        case 'ArrowLeft':
          twist.angular.z = 0.5;
          break;
        case 'ArrowRight':
          twist.angular.z = -0.5;
          break;
        default:
          return;
      }

      cmdVelPublisher.publish(twist);
      const now = Date.now();
      if (now - lastLogTime.current >= logInterval) {
        console.log('Published cmd_vel:', twist);
        lastLogTime.current = now;
      }
    };

    const handleKeyUp = (event) => {
      const twist = new ROSLIB.Message({
        linear: { x: 0, y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0 }
      });

      cmdVelPublisher.publish(twist);
      const now = Date.now();
      if (now - lastLogTime.current >= logInterval) {
        console.log('Published cmd_vel (stop):', twist);
        lastLogTime.current = now;
      }
    };

    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    setIsConnected(true);

    return () => {
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
      cmdVelSubscriber.unsubscribe();
    };
  }, [ros]);

  return (
    <div className="controller">
      <h3>Controller</h3>
      {error && <div className="error-message">{error}</div>}
      {isConnected ? (
        <div className="controller-status connected">
          Connected to ROS
        </div>
      ) : (
        <div className="controller-status disconnected">
          Disconnected from ROS
        </div>
      )}
      <div className="controller-instructions">
        <p>Use arrow keys to control the robot:</p>
        <ul>
          <li>↑: Move forward</li>
          <li>↓: Move backward</li>
          <li>←: Rotate left</li>
          <li>→: Rotate right</li>
        </ul>
      </div>
    </div>
  );
};

export default Controller; 