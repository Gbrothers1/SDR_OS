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

const TelemetryPanel = ({ ros, updateInterval, initialShowPanel = true }) => {
  const [telemetryData, setTelemetryData] = useState(null);
  const [error, setError] = useState(null);
  const [showPanel, setShowPanel] = useState(initialShowPanel);
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
        // Only update if we have IMU data
        if (data?.imu) {
          // Extract values
          const { accel, gyro } = data.imu;
          const values = {
            ax: accel.x,
            ay: accel.y,
            az: accel.z,
            gx: gyro.x,
            gy: gyro.y,
            gz: gyro.z
          };
          
          // Update history
          setDataHistory(prev => {
            const newHistory = { ...prev };
            Object.keys(values).forEach(key => {
              const historyKey = key === 'ax' ? 'accelX' :
                               key === 'ay' ? 'accelY' :
                               key === 'az' ? 'accelZ' :
                               key === 'gx' ? 'gyroX' :
                               key === 'gy' ? 'gyroY' :
                               'gyroZ';
              newHistory[historyKey] = [...prev[historyKey].slice(1), values[key]];
            });
            return newHistory;
          });
          
          // Update telemetry data
          setTelemetryData(data);
        }
      } catch (err) {
        console.error('Error in throttledUpdateState:', err);
        setError('Error processing telemetry data');
      }
    }, updateInterval || 100),
    [updateInterval]
  );

  // For message handler, do not create a new function on each render when showPanel changes
  // This prevents the cycle of subscribe/unsubscribe
  const messageHandlerRef = useRef(null);
  
  // Update the handler reference when throttledUpdateState changes
  useEffect(() => {
    messageHandlerRef.current = (message) => {
      try {
        // Skip processing if panel isn't visible, but don't recreate the handler
        if (!showPanel) return;
        
        const data = JSON.parse(message.data);
        throttledUpdateState(data);
      } catch (err) {
        console.error('Error parsing telemetry data:', err);
        setError('Error parsing telemetry data');
      }
    };
  }, [throttledUpdateState, showPanel]);

  // Create a stable subscription handler that uses the current ref value
  const stableMessageHandler = useCallback((message) => {
    if (messageHandlerRef.current) {
      messageHandlerRef.current(message);
    }
  }, []);

  // Subscribe to telemetry topic only once when component mounts
  useEffect(() => {
    if (!ros) {
      setError('ROS connection not available');
      return;
    }
    
    let telemetryTopic = null;
    
    try {
      telemetryTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/robot/telemetry/all',
        messageType: 'std_msgs/String'
      });
      
      // Using our stable handler
      telemetryTopic.subscribe(stableMessageHandler);
      console.log('TelemetryPanel mounted, subscribed to /robot/telemetry/all');
    } catch (err) {
      console.error('Error subscribing to telemetry topic:', err);
      setError('Failed to subscribe to telemetry topic');
    }
    
    // Only unsubscribe when component unmounts
    return () => {
      if (telemetryTopic) {
        try {
          telemetryTopic.unsubscribe();
          console.log('TelemetryPanel unmounted, unsubscribed from /robot/telemetry/all');
        } catch (err) {
          console.error('Error unsubscribing from telemetry topic:', err);
        }
      }
    };
  }, [ros, stableMessageHandler]);
  
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
  }, [dataHistory, chartConfig]);  // Remove showPanel from dependencies

  // Initialize showPanel based on prop
  useEffect(() => {
    setShowPanel(initialShowPanel);
  }, [initialShowPanel]);

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
    <div className={`telemetry-panel ${!showPanel ? 'collapsed' : ''}`}>
      <div className="telemetry-panel-border"></div>
      <div className="panel-header">
        <h3>Telemetry</h3>
        <button className="toggle-button" onClick={togglePanel}>
          <span>{showPanel ? "📊" : "📊"}</span>
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