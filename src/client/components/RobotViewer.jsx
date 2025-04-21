import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import ROSLIB from 'roslib';

const RobotViewer = ({ ros, tfThrottleRate }) => {
  const containerRef = useRef();
  const sceneRef = useRef();
  const cameraRef = useRef();
  const rendererRef = useRef();
  const controlsRef = useRef();
  const robotModelRef = useRef();
  const resizeObserverRef = useRef();
  const [isResizing, setIsResizing] = useState(false);
  const resizeTimeoutRef = useRef(null);
  const [imuData, setImuData] = useState(null);

  useEffect(() => {
    // Initialize Three.js scene
    const init = () => {
      // Create scene
      sceneRef.current = new THREE.Scene();
      sceneRef.current.background = new THREE.Color(0x1a1a1a);

      // Create camera
      cameraRef.current = new THREE.PerspectiveCamera(
        75,
        containerRef.current.clientWidth / containerRef.current.clientHeight,
        0.1,
        1000
      );
      cameraRef.current.position.set(2, 2, 2);

      // Create renderer
      rendererRef.current = new THREE.WebGLRenderer({ antialias: true });
      rendererRef.current.setSize(
        containerRef.current.clientWidth,
        containerRef.current.clientHeight
      );
      containerRef.current.appendChild(rendererRef.current.domElement);

      // Add orbit controls
      controlsRef.current = new OrbitControls(
        cameraRef.current,
        rendererRef.current.domElement
      );
      controlsRef.current.enableDamping = true;

      // Add lighting
      const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
      sceneRef.current.add(ambientLight);

      const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
      directionalLight.position.set(5, 5, 5);
      sceneRef.current.add(directionalLight);

      // Add grid helper
      const gridHelper = new THREE.GridHelper(10, 10);
      sceneRef.current.add(gridHelper);

      // Create robot model
      createRobotModel();

      // Start animation loop
      animate();
    };

    // Create enhanced robot model with IMU indicators
    const createRobotModel = () => {
      const robotGroup = new THREE.Group();

      // Base
      const baseGeometry = new THREE.CylinderGeometry(0.5, 0.5, 0.2, 32);
      const baseMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 });
      const base = new THREE.Mesh(baseGeometry, baseMaterial);
      robotGroup.add(base);

      // Body
      const bodyGeometry = new THREE.BoxGeometry(0.8, 1, 0.4);
      const bodyMaterial = new THREE.MeshPhongMaterial({ color: 0x888888 });
      const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
      body.position.y = 0.6;
      robotGroup.add(body);
      
      // Add IMU sensor representation (a small box)
      const imuGeometry = new THREE.BoxGeometry(0.2, 0.1, 0.2);
      const imuMaterial = new THREE.MeshPhongMaterial({ color: 0x4CAF50 });
      const imu = new THREE.Mesh(imuGeometry, imuMaterial);
      imu.position.set(0, 1.2, 0);
      robotGroup.add(imu);
      
      // Add orientation axes for IMU
      const axesHelper = new THREE.AxesHelper(0.3);
      axesHelper.position.copy(imu.position);
      robotGroup.add(axesHelper);

      robotModelRef.current = robotGroup;
      sceneRef.current.add(robotGroup);
    };

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);

      if (controlsRef.current) {
        controlsRef.current.update();
      }

      rendererRef.current.render(sceneRef.current, cameraRef.current);
    };

    // Handle container resize
    const handleResize = () => {
      if (containerRef.current && cameraRef.current && rendererRef.current) {
        // Set resizing state to true
        setIsResizing(true);
        
        // Clear any existing timeout
        if (resizeTimeoutRef.current) {
          clearTimeout(resizeTimeoutRef.current);
        }
        
        // Set a timeout to update the renderer after the CSS transition completes
        resizeTimeoutRef.current = setTimeout(() => {
          const width = containerRef.current.clientWidth;
          const height = containerRef.current.clientHeight;

          cameraRef.current.aspect = width / height;
          cameraRef.current.updateProjectionMatrix();

          rendererRef.current.setSize(width, height);
          
          // Set resizing state to false
          setIsResizing(false);
        }, 100); // Match the CSS transition duration (5x faster)
      }
    };

    // Initialize scene
    init();

    // Add resize observer to detect container size changes
    resizeObserverRef.current = new ResizeObserver(() => {
      handleResize();
    });
    
    if (containerRef.current) {
      resizeObserverRef.current.observe(containerRef.current);
    }

    // Add window resize listener as a fallback
    window.addEventListener('resize', handleResize);

    // ROS Subscriptions
    if (ros) {
      // Calculate throttle interval in milliseconds and ROUND it
      const throttleIntervalMs = tfThrottleRate > 0 ? Math.round(1000 / tfThrottleRate) : 0;

      // Subscribe to IMU topic
      const imuTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/robot/imu',
        messageType: 'sensor_msgs/Imu'
      });

      imuTopic.subscribe((message) => {
        setImuData(message);
      });
      
      // Standard ROS joint states subscription (consider throttling?)
      const jointStatesTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/joint_states',
        messageType: 'sensor_msgs/JointState'
        // throttle_rate: throttleIntervalMs, // Can also throttle this if needed
      });

      jointStatesTopic.subscribe((message) => {
        // Handle joint state messages for standard robot joints
        // console.log('Joint States:', message); 
      });

      // Subscribe to TF and TF_Static with throttling
      const tfTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/tf',
        messageType: 'tf2_msgs/msg/TFMessage',
        throttle_rate: throttleIntervalMs
      });

      tfTopic.subscribe((message) => {
        // Process TF messages (likely handled by a TF client library later)
        // console.log('TF Message (throttled):', message);
      });

      const tfStaticTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/tf_static',
        messageType: 'tf2_msgs/msg/TFMessage',
        throttle_rate: throttleIntervalMs // Throttle static potentially less useful, but consistent
      });

      tfStaticTopic.subscribe((message) => {
        // Process TF static messages
        // console.log('TF Static Message (throttled):', message);
      });

    }

    // Cleanup
    return () => {
      window.removeEventListener('resize', handleResize);
      if (resizeObserverRef.current) {
        resizeObserverRef.current.disconnect();
      }
      if (resizeTimeoutRef.current) {
        clearTimeout(resizeTimeoutRef.current);
      }
      if (containerRef.current && rendererRef.current) {
        containerRef.current.removeChild(rendererRef.current.domElement);
      }
    };
  }, [ros, tfThrottleRate]);

  // Update model when IMU data changes
  useEffect(() => {
    if (robotModelRef.current && imuData) {
      // Apply IMU orientation to the robot model
      const { orientation } = imuData;
      
      // Create a quaternion from the IMU data
      const quaternion = new THREE.Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      );
      
      // Apply the quaternion to the robot's rotation
      // Note: We're only rotating the model, not changing its position
      robotModelRef.current.quaternion.copy(quaternion);
    }
  }, [imuData]);

  return (
    <div
      ref={containerRef}
      style={{
        width: '100%',
        height: '100%',
        opacity: isResizing ? 0.8 : 1,
        transition: 'opacity 0.3s ease'
      }}
    />
  );
};

export default RobotViewer; 