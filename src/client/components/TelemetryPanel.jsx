import React, { useEffect, useState, useRef, useCallback } from 'react';
import ROSLIB from 'roslib';
import '../styles/TelemetryPanel.css';

// Throttle utility function
const throttle = (func, limit) => {
  let inThrottle;
  let lastFunc;
  let lastRan;
  return function throttled(...args) {
    const context = this;
    if (!inThrottle) {
      func.apply(context, args);
      lastRan = Date.now();
      inThrottle = true;
      setTimeout(() => {
        inThrottle = false;
        if (lastFunc) {
          lastFunc();
          lastFunc = null;
        }
      }, limit);
    } else {
      // Store the latest arguments and function call
      lastFunc = () => {
        if (Date.now() - lastRan >= limit) {
          func.apply(context, args);
          lastRan = Date.now();
        }
      };
    }
  };
};

const TelemetryPanel = ({ ros, updateInterval }) => {
  const [telemetryData, setTelemetryData] = useState(null);
  const [error, setError] = useState(null);
  const [showPanel, setShowPanel] = useState(true);
  const chartCanvasRef = useRef(null);
  
  // Chart configuration
  const chartConfig = {
    width: 300,
    height: 200,
    padding: 20,
    dataPoints: 50, // Number of data points to show
    colors: {
      accelX: '#FF6384', // Red
      accelY: '#36A2EB', // Blue
      accelZ: '#FFCE56', // Yellow
      gyroX: '#4BC0C0',  // Teal
      gyroY: '#9966FF',  // Purple
      gyroZ: '#FF9F40',  // Orange
    }
  };
  
  // Store history of values for charts
  const [dataHistory, setDataHistory] = useState({
    accelX: Array(chartConfig.dataPoints).fill(0),
    accelY: Array(chartConfig.dataPoints).fill(0),
    accelZ: Array(chartConfig.dataPoints).fill(0),
    gyroX: Array(chartConfig.dataPoints).fill(0),
    gyroY: Array(chartConfig.dataPoints).fill(0),
    gyroZ: Array(chartConfig.dataPoints).fill(0),
  });

  // Memoized and throttled function to update state
  const throttledUpdateState = useCallback(
    throttle((data) => {
      try {
        // Log the parsed data structure - check browser console!
        console.log('>>> TelemetryPanel PARSED data (throttled):', data);
        setTelemetryData(data); // Update main data state

        // Update data history for charts
        if (data.imu) {
          // Log the IMU data being used for history update
          console.log('>>> TelemetryPanel Updating History with IMU (throttled):', data.imu);
          setDataHistory(prev => {
            const newHistory = { ...prev };
            // Log the actual values being pushed
            console.log('>>> Pushing values (throttled):', {ax: data.imu.accel?.x, ay: data.imu.accel?.y, az: data.imu.accel?.z, gx: data.imu.gyro?.x, gy: data.imu.gyro?.y, gz: data.imu.gyro?.z });
            newHistory.accelX = [...prev.accelX.slice(1), data.imu.accel?.x ?? 0]; 
            newHistory.accelY = [...prev.accelY.slice(1), data.imu.accel?.y ?? 0];
            newHistory.accelZ = [...prev.accelZ.slice(1), data.imu.accel?.z ?? 0];
            newHistory.gyroX = [...prev.gyroX.slice(1), data.imu.gyro?.x ?? 0];
            newHistory.gyroY = [...prev.gyroY.slice(1), data.imu.gyro?.y ?? 0];
            newHistory.gyroZ = [...prev.gyroZ.slice(1), data.imu.gyro?.z ?? 0];
            console.log('>>> New History State (throttled):', newHistory); // Log the new history state before returning
            return newHistory;
          });
        } else {
          console.warn('>>> TelemetryPanel: data.imu not found in message, skipping history update (throttled).');
        }
      } catch (err) {
         // Note: Errors inside the throttle might be less obvious
        console.error('Error during throttled state update:', err);
        setError('Error processing telemetry data');
      }
    }, updateInterval ?? 100), // Throttle based on prop, default 100ms
    [updateInterval] // Recreate throttle function if interval changes
  );

  // Direct handler for incoming messages - parses JSON and calls throttled update
  const handleTelemetryData = useCallback((message) => {
    // Log the raw incoming message data - check browser console!
    console.log('>>> TelemetryPanel RAW message.data:', message.data);
    try {
      const data = JSON.parse(message.data);
      // Call the throttled function to handle state updates
      throttledUpdateState(data);
    } catch (err) {
      console.error('Error parsing telemetry data:', err);
      setError('Error parsing telemetry data');
    } 
  }, [throttledUpdateState]); // Depends on the throttled function instance

  useEffect(() => {
    if (!ros) {
      setError('ROS connection not available');
      return;
    }

    let telemetryTopic = null;

    telemetryTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/robot/telemetry/all',
      messageType: 'std_msgs/String'
    });

    // Subscribe using the direct handler (which then calls the throttled one)
    telemetryTopic.subscribe(handleTelemetryData);
    console.log('TelemetryPanel mounted, subscribed to /robot/telemetry/all');

    return () => {
      if (telemetryTopic) {
        telemetryTopic.unsubscribe();
        console.log('TelemetryPanel unmounted, unsubscribed from /robot/telemetry/all');
      }
    };
  // Dependency: only re-subscribe if ros changes or the handler function itself changes
  }, [ros, handleTelemetryData]);
  
  // Draw charts when data history changes
  useEffect(() => {
    // Log when this effect runs
    console.log('--- Chart Drawing Effect RUNNING ---'); 
    try { // Add try block
      if (!chartCanvasRef.current || !dataHistory) {
        console.log('--- Chart Drawing Effect: Skipping (no canvas or dataHistory) ---');
        return;
      }
      
      console.log('--- Chart Drawing Effect: Proceeding to draw ---');
      const canvas = chartCanvasRef.current;
      const ctx = canvas.getContext('2d');
      const { width, height, padding, colors } = chartConfig;
      
      // Clear canvas
      ctx.clearRect(0, 0, width, height);
      
      // Draw background
      ctx.fillStyle = 'rgba(0, 0, 0, 0.1)';
      ctx.fillRect(0, 0, width, height);
      
      // Draw grid lines
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
      ctx.beginPath();
      
      // Horizontal center line
      ctx.moveTo(padding, height / 2);
      ctx.lineTo(width - padding, height / 2);
      
      // Vertical lines
      for (let i = 1; i < 5; i++) {
        const x = padding + ((width - 2 * padding) / 5) * i;
        ctx.moveTo(x, padding);
        ctx.lineTo(x, height - padding);
      }
      
      ctx.stroke();
      
      // Find min/max values for scaling
      const allValues = [
        ...dataHistory.accelX, 
        ...dataHistory.accelY, 
        ...dataHistory.accelZ,
        ...dataHistory.gyroX,
        ...dataHistory.gyroY,
        ...dataHistory.gyroZ,
      ];
      
      const maxValue = Math.max(Math.abs(Math.min(...allValues)), Math.max(...allValues), 0.1);
      const valueRange = maxValue * 2; // From -max to +max
      
      // Function to draw a data series
      const drawSeries = (data, color) => {
        const step = (width - 2 * padding) / (data.length - 1);
        
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        
        data.forEach((value, index) => {
          const x = padding + index * step;
          const y = height / 2 - (value / valueRange) * (height - 2 * padding);
          
          if (index === 0) {
            ctx.moveTo(x, y);
          } else {
            ctx.lineTo(x, y);
          }
        });
        
        ctx.stroke();
      };
      
      // Draw accelerometer data
      drawSeries(dataHistory.accelX, colors.accelX);
      drawSeries(dataHistory.accelY, colors.accelY);
      drawSeries(dataHistory.accelZ, colors.accelZ);
      
      // Draw gyroscope data (with dash line)
      ctx.setLineDash([5, 3]);
      drawSeries(dataHistory.gyroX, colors.gyroX);
      drawSeries(dataHistory.gyroY, colors.gyroY);
      drawSeries(dataHistory.gyroZ, colors.gyroZ);
      ctx.setLineDash([]);
      
      // Draw legend
      ctx.font = '10px Arial';
      ctx.textAlign = 'left';
      
      const legends = [
        { label: 'AccelX', color: colors.accelX },
        { label: 'AccelY', color: colors.accelY },
        { label: 'AccelZ', color: colors.accelZ },
        { label: 'GyroX', color: colors.gyroX, dashed: true },
        { label: 'GyroY', color: colors.gyroY, dashed: true },
        { label: 'GyroZ', color: colors.gyroZ, dashed: true }
      ];
      
      legends.forEach((legend, index) => {
        const x = padding + 10;
        const y = padding + 10 + index * 15;
        
        // Draw color box
        ctx.fillStyle = legend.color;
        ctx.fillRect(x, y - 8, 10, 10);
        
        // Draw dash for gyro
        if (legend.dashed) {
          ctx.strokeStyle = 'white';
          ctx.beginPath();
          ctx.moveTo(x, y - 3);
          ctx.lineTo(x + 10, y - 3);
          ctx.stroke();
        }
        
        // Draw label
        ctx.fillStyle = 'white';
        ctx.fillText(legend.label, x + 15, y);
      });
      
      console.log('--- Chart Drawing Effect COMPLETED ---'); // Log successful completion

    } catch (error) {
      // Catch and log any errors during drawing
      console.error('!!! ERROR in Chart Drawing Effect:', error);
    }
  }, [dataHistory, chartConfig]);

  const togglePanel = () => {
    setShowPanel(!showPanel);
  };

  const formatValue = (value) => {
    if (value === undefined || value === null) return 'N/A';
    if (typeof value === 'number') return value.toFixed(4);
    return value;
  };

  const renderOrientationDisplay = (orientation) => {
    if (!orientation) return null;
    
    const roll = orientation.roll * (180 / Math.PI);  // Convert to degrees
    const pitch = orientation.pitch * (180 / Math.PI);
    const yaw = orientation.yaw * (180 / Math.PI);
    
    return (
      <div className="orientation-display">
        <div className="orientation-box">
          <div 
            className="orientation-indicator"
            style={{
              transform: `rotateX(${pitch}deg) rotateY(${roll}deg) rotateZ(${yaw}deg)`
            }}
          >
            <div className="face front">F</div>
            <div className="face back">B</div>
            <div className="face right">R</div>
            <div className="face left">L</div>
            <div className="face top">T</div>
            <div className="face bottom">B</div>
          </div>
        </div>
        <div className="orientation-values">
          <div>Roll: {roll.toFixed(1)}°</div>
          <div>Pitch: {pitch.toFixed(1)}°</div>
          <div>Yaw: {yaw.toFixed(1)}°</div>
        </div>
      </div>
    );
  };

  // Log state right before rendering
  console.log('--- Rendering TelemetryPanel --- State:', { telemetryData, dataHistory, error, showPanel });

  return (
    <div className={`telemetry-panel ${showPanel ? '' : 'collapsed'}`}>
      <div className="panel-header">
        <h3>Sensor Telemetry</h3>
        <button className="toggle-button" onClick={togglePanel}>
          {showPanel ? '−' : '+'}
        </button>
      </div>
      
      {showPanel && (
        <div className="panel-content">
          {error && <div className="error-message">{error}</div>}
          
          {!telemetryData && !error && (
            <div className="loading">Waiting for telemetry data...</div>
          )}
          
          {telemetryData && (
            <>
              <div className="telemetry-section">
                <h4>IMU Data</h4>
                {telemetryData.imu && (
                  <div className="telemetry-grid">
                    <div className="telemetry-chart">
                      <canvas 
                        ref={chartCanvasRef} 
                        width={chartConfig.width} 
                        height={chartConfig.height}
                      />
                    </div>
                    
                    <div className="telemetry-values">
                      <div className="sensor-group">
                        <h5>Accelerometer</h5>
                        <div className="value-row">
                          <span>X:</span>
                          <span>{formatValue(telemetryData.imu.accel.x)} m/s²</span>
                        </div>
                        <div className="value-row">
                          <span>Y:</span>
                          <span>{formatValue(telemetryData.imu.accel.y)} m/s²</span>
                        </div>
                        <div className="value-row">
                          <span>Z:</span>
                          <span>{formatValue(telemetryData.imu.accel.z)} m/s²</span>
                        </div>
                      </div>
                      
                      <div className="sensor-group">
                        <h5>Gyroscope</h5>
                        <div className="value-row">
                          <span>X:</span>
                          <span>{formatValue(telemetryData.imu.gyro.x)} rad/s</span>
                        </div>
                        <div className="value-row">
                          <span>Y:</span>
                          <span>{formatValue(telemetryData.imu.gyro.y)} rad/s</span>
                        </div>
                        <div className="value-row">
                          <span>Z:</span>
                          <span>{formatValue(telemetryData.imu.gyro.z)} rad/s</span>
                        </div>
                      </div>
                    </div>
                    
                    {renderOrientationDisplay(telemetryData.imu.orientation)}
                  </div>
                )}
              </div>
              
              <div className="telemetry-section">
                <h4>Available Sensors</h4>
                <div className="sensors-list">
                  {Object.entries(telemetryData.devices).map(([name, device]) => (
                    <div key={name} className="sensor-item">
                      <div className="sensor-name">{name}</div>
                      <div className="sensor-description">{device.description}</div>
                    </div>
                  ))}
                </div>
              </div>
            </>
          )}
        </div>
      )}
    </div>
  );
};

export default TelemetryPanel; 