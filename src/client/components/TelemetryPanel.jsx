import React, { useEffect, useState, useRef } from 'react';
import ROSLIB from 'roslib';
import '../styles/TelemetryPanel.css';

const TelemetryPanel = ({ ros }) => {
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

  useEffect(() => {
    if (!ros) {
      setError('ROS connection not available');
      return;
    }

    // Subscribe to telemetry data
    const telemetryTopic = new ROSLIB.Topic({
      ros: ros,
      name: '/robot/telemetry/all',
      messageType: 'std_msgs/String'
    });

    const handleTelemetryData = (message) => {
      try {
        const data = JSON.parse(message.data);
        setTelemetryData(data);
        
        // Update data history for charts
        if (data.imu) {
          setDataHistory(prev => {
            const newHistory = { ...prev };
            
            // Update accelerometer data
            newHistory.accelX = [...prev.accelX.slice(1), data.imu.accel.x];
            newHistory.accelY = [...prev.accelY.slice(1), data.imu.accel.y];
            newHistory.accelZ = [...prev.accelZ.slice(1), data.imu.accel.z];
            
            // Update gyroscope data
            newHistory.gyroX = [...prev.gyroX.slice(1), data.imu.gyro.x];
            newHistory.gyroY = [...prev.gyroY.slice(1), data.imu.gyro.y];
            newHistory.gyroZ = [...prev.gyroZ.slice(1), data.imu.gyro.z];
            
            return newHistory;
          });
        }
      } catch (err) {
        console.error('Error parsing telemetry data:', err);
        setError('Error parsing telemetry data');
      }
    };

    telemetryTopic.subscribe(handleTelemetryData);

    return () => {
      telemetryTopic.unsubscribe();
    };
  }, [ros]);
  
  // Draw charts when data history changes
  useEffect(() => {
    if (!chartCanvasRef.current || !dataHistory) return;
    
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