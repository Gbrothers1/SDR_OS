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

      // Base - with sci-fi styling
      const baseGeometry = new THREE.CylinderGeometry(0.5, 0.6, 0.2, 8);
      const baseMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x333333,
        emissive: 0x113311,
        shininess: 100,
        specular: 0x00ff00
      });
      const base = new THREE.Mesh(baseGeometry, baseMaterial);
      // Make sure base is properly aligned with world up
      base.rotation.x = Math.PI / 2; // Rotate to make cylinder stand upright
      robotGroup.add(base);

      // Body - with sci-fi styling
      const bodyGeometry = new THREE.BoxGeometry(0.8, 0.4, 1);
      const bodyMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x444444,
        emissive: 0x001111,
        shininess: 90,
        specular: 0x0088ff 
      });
      const body = new THREE.Mesh(bodyGeometry, bodyMaterial);
      body.position.y = 0.6;
      // Rotate body so "front" is facing forward
      body.rotation.x = Math.PI / 2;
      robotGroup.add(body);
      
      // Add glowing edges to body
      const bodyEdges = new THREE.EdgesGeometry(bodyGeometry);
      const bodyLine = new THREE.LineSegments(
        bodyEdges,
        new THREE.LineBasicMaterial({ color: 0x00ffff, linewidth: 2 })
      );
      bodyLine.position.copy(body.position);
      bodyLine.rotation.copy(body.rotation);
      robotGroup.add(bodyLine);
      
      // Add IMU sensor representation (a small box with glow)
      const imuGeometry = new THREE.BoxGeometry(0.2, 0.1, 0.2);
      const imuMaterial = new THREE.MeshPhongMaterial({ 
        color: 0x22aa22,
        emissive: 0x00ff00, 
        emissiveIntensity: 0.5,
        shininess: 100
      });
      const imu = new THREE.Mesh(imuGeometry, imuMaterial);
      imu.position.set(0, 0.6, -0.5); // Raised IMU position by 10 units
      robotGroup.add(imu);
      
      // Add thicker orientation axes for IMU
      // Create custom axis lines that are thicker and longer
      const axisLength = 0.4;
      const axisWidth = 0.03; // Slightly thicker for better visibility
      
      // Create a separate axes group that will maintain world orientation
      const axesGroup = new THREE.Group();
      
      // X-axis (red)
      const xAxisGeometry = new THREE.BoxGeometry(axisLength, axisWidth, axisWidth);
      const xAxisMaterial = new THREE.MeshBasicMaterial({ 
        color: 0xff0000,
        emissive: 0xff0000,
        emissiveIntensity: 0.5
      });
      const xAxis = new THREE.Mesh(xAxisGeometry, xAxisMaterial);
      xAxis.position.set(axisLength/2, 0, 0);
      
      // Y-axis (green)
      const yAxisGeometry = new THREE.BoxGeometry(axisWidth, axisLength, axisWidth);
      const yAxisMaterial = new THREE.MeshBasicMaterial({ 
        color: 0x00ff00,
        emissive: 0x00ff00,
        emissiveIntensity: 0.5
      });
      const yAxis = new THREE.Mesh(yAxisGeometry, yAxisMaterial);
      yAxis.position.set(0, axisLength/2, 0);
      
      // Z-axis (blue)
      const zAxisGeometry = new THREE.BoxGeometry(axisWidth, axisWidth, axisLength);
      const zAxisMaterial = new THREE.MeshBasicMaterial({ 
        color: 0x0000ff,
        emissive: 0x0000ff,
        emissiveIntensity: 0.5 
      });
      const zAxis = new THREE.Mesh(zAxisGeometry, zAxisMaterial);
      zAxis.position.set(0, 0, axisLength/2);
      
      // Create a sci-fi text label with dark background and futuristic border
      const createTextLabel = (text, color, position, scale = 0.4) => {
        // Create a plane to hold the text
        const canvas = document.createElement('canvas');
        canvas.width = 256;
        canvas.height = 128;
        
        // Get context and draw text on it
        const context = canvas.getContext('2d');
        
        // Clear canvas with transparent background
        context.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw sci-fi style background - solid color with high opacity
        context.fillStyle = 'rgba(0, 20, 30, 0.9)';
        context.fillRect(0, 0, canvas.width, canvas.height);
        
        // Draw bright cyan border
        context.strokeStyle = '#00ffff';
        context.lineWidth = 3;
        context.strokeRect(3, 3, canvas.width - 6, canvas.height - 6);
        
        // Draw inner border
        context.strokeStyle = '#ffffff';
        context.lineWidth = 1;
        context.strokeRect(6, 6, canvas.width - 12, canvas.height - 12);
        
        // Add text with strong contrast
        context.font = 'bold 64px Arial';
        
        // Draw text shadow
        context.fillStyle = '#000000';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(text, canvas.width / 2 + 2, canvas.height / 2 + 2);
        
        // Draw main text in bright color
        context.fillStyle = '#ffffff'; // White text for best contrast
        context.fillText(text, canvas.width / 2, canvas.height / 2);
        
        // Add colored accent based on passed color
        context.fillStyle = color;
        context.fillRect(0, 0, 10, canvas.height);
        
        // Create texture
        const texture = new THREE.CanvasTexture(canvas);
        texture.minFilter = THREE.LinearFilter; // Improve text quality
        texture.needsUpdate = true;
        
        // Create material with the texture
        const material = new THREE.MeshBasicMaterial({
          map: texture,
          side: THREE.DoubleSide,
          transparent: true,
          opacity: 1.0 // Full opacity
        });
        
        // Create plane geometry
        const geometry = new THREE.PlaneGeometry(scale, scale * (canvas.height / canvas.width));
        
        // Create mesh
        const textMesh = new THREE.Mesh(geometry, material);
        textMesh.position.copy(position);
        
        return textMesh;
      };
      
      // Create labels for each axis with the new approach - positioned further out
      const xLabel = createTextLabel('X', '#ff0000', new THREE.Vector3(axisLength + 0.2, 0, 0), 0.25);
      const yLabel = createTextLabel('Y', '#00ff00', new THREE.Vector3(0, -(axisLength + 0.2), 0), 0.25);
      const zLabel = createTextLabel('Z', '#0000ff', new THREE.Vector3(0, 0, axisLength + 0.2), 0.25);
      
      // Add X and Z axes to main axes group
      axesGroup.add(xAxis, xLabel, zAxis, zLabel);
      
      // Create a subgroup for Y axis components that will be rotated
      const yAxisGroup = new THREE.Group();
      yAxisGroup.add(yAxis, yLabel);
      yAxisGroup.rotation.z = Math.PI; // Rotate 180 degrees around Z
      axesGroup.add(yAxisGroup);
      
      // Position axes group at the IMU location
      axesGroup.position.copy(imu.position);
      
      // Rotate axes group to match world orientation
      axesGroup.rotation.x = Math.PI/2; // This keeps Y pointing up in world space
      
      // Add labels for the robot body faces with the new approach - better positions
      const frontLabel = createTextLabel('FRONT', '#00ffff', new THREE.Vector3(0, 1.5, 0), 0.45);
      const backLabel = createTextLabel('BACK', '#00ffff', new THREE.Vector3(0, -0.6, -0.3), 0.45);
      const topLabel = createTextLabel('TOP', '#00ffff', new THREE.Vector3(0, 0.5, -1.0), 0.35);
      const bottomLabel = createTextLabel('BOTTOM', '#00ffff', new THREE.Vector3(0, 0.75, 1.0), 0.35);

      // Store face normals and their corresponding labels
      const faceLabels = [
        { label: frontLabel, normal: new THREE.Vector3(0, 1, 0) },
        { label: backLabel, normal: new THREE.Vector3(0, -1, 0) },
        { label: topLabel, normal: new THREE.Vector3(0, 0, -1) },
        { label: bottomLabel, normal: new THREE.Vector3(0, 0, 1) }
      ];
      
      // Add axes group and labels to robot group
      robotGroup.add(axesGroup);
      robotGroup.add(frontLabel, backLabel, topLabel, bottomLabel);

      // Set initial visibility to false
      faceLabels.forEach(({ label }) => {
        label.visible = false;
      });

      // Store the face labels configuration in the robot model ref for access in animation loop
      robotModelRef.current = robotGroup;
      robotModelRef.current.faceLabels = faceLabels;
      sceneRef.current.add(robotGroup);
    };

    // Animation loop
    const animate = () => {
      requestAnimationFrame(animate);

      if (controlsRef.current) {
        controlsRef.current.update();
      }
      
      // Update text labels to face the camera and check visibility
      if (cameraRef.current && robotModelRef.current) {
        // Calculate camera direction vector
        const cameraPosition = new THREE.Vector3();
        cameraRef.current.getWorldPosition(cameraPosition);
        const cameraDirection = new THREE.Vector3();
        cameraRef.current.getWorldDirection(cameraDirection);

        robotModelRef.current.traverse((child) => {
          // Find all text meshes (they use PlaneGeometry)
          if (child.geometry && child.geometry.type === 'PlaneGeometry') {
            // Make the label face the camera
            child.lookAt(cameraPosition);
          }
        });

        // Update face label visibility based on camera angle
        if (robotModelRef.current.faceLabels) {
          robotModelRef.current.faceLabels.forEach(({ label, normal }) => {
            // Transform the normal to world space
            const worldNormal = normal.clone();
            worldNormal.applyQuaternion(robotModelRef.current.quaternion);
            
            // Calculate dot product between camera direction and face normal
            const dotProduct = cameraDirection.dot(worldNormal);
            
            // Show label only when camera is facing the surface (dot product < -0.2)
            // Using -0.2 instead of 0 gives a bit more viewing angle flexibility
            label.visible = dotProduct < -0.2;
          });
        }
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
      
      // Extract euler angles from quaternion
      if (orientation && orientation.w !== undefined) {
        try {
      // Create a quaternion from the IMU data
          const imuQuaternion = new THREE.Quaternion(
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
      );
      
          // Extract Euler angles from quaternion to understand current orientation
          const euler = new THREE.Euler().setFromQuaternion(imuQuaternion, 'XYZ');
          console.log('IMU Euler angles (rad):', {
            x: euler.x, // roll
            y: euler.y, // pitch
            z: euler.z  // yaw
          });
          
          // Create a corrected quaternion with adjusted axis mapping for proper robot orientation
          // This ensures the model faces the right direction according to ROS convention
          const correctedQuaternion = new THREE.Quaternion();
          correctedQuaternion.setFromEuler(
            new THREE.Euler(
              -euler.x, // Invert X rotation (roll) to match expected behavior
              euler.y,  // Keep Y rotation (pitch) as is
              euler.z,  // Keep Z rotation (yaw) as is
              'XYZ'
            )
          );
          
          // Apply base orientation correction (Z-up in ROS to Y-up in Three.js)
          const baseCorrection = new THREE.Quaternion().setFromEuler(
            new THREE.Euler(-Math.PI/2, 0, 0, 'XYZ')
          );
          
          // Combine the quaternions: base correction first, then IMU orientation
          const finalQuaternion = baseCorrection.multiply(correctedQuaternion);
          
          // Apply to robot model
          robotModelRef.current.quaternion.copy(finalQuaternion);
        } catch (err) {
          console.error('Error applying orientation:', err);
        }
      }
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