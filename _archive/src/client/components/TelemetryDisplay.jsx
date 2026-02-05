import React, { useState, useEffect, useCallback } from 'react';
import '../styles/TelemetryDisplay.css';

const TelemetryDisplay = ({ telemetryData }) => {
  const [stabilizedData, setStabilizedData] = useState({
    battery: { voltage: 0, current: 0, percentage: 0 },
    temperature: { cpu: 0, motor: 0 },
    position: { x: 0, y: 0, z: 0 },
    orientation: { roll: 0, pitch: 0, yaw: 0 },
    velocity: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
    jointStates: [],
    diagnostics: []
  });

  // Debounce function
  const debounce = useCallback((func, wait) => {
    let timeout;
    return (...args) => {
      clearTimeout(timeout);
      timeout = setTimeout(() => func(...args), wait);
    };
  }, []);

  // Stabilize data updates
  const updateStabilizedData = useCallback(
    debounce((newData) => {
      setStabilizedData(prevData => {
        // Only update if the change is significant
        const isSignificantChange = (old, new_) => {
          if (typeof old === 'number' && typeof new_ === 'number') {
            return Math.abs(old - new_) > 0.01;
          }
          return false;
        };

        const updateIfSignificant = (oldObj, newObj) => {
          const result = { ...oldObj };
          Object.keys(newObj).forEach(key => {
            if (typeof newObj[key] === 'object' && newObj[key] !== null) {
              result[key] = updateIfSignificant(oldObj[key] || {}, newObj[key]);
            } else if (isSignificantChange(oldObj[key], newObj[key])) {
              result[key] = newObj[key];
            } else {
              result[key] = oldObj[key];
            }
          });
          return result;
        };

        return updateIfSignificant(prevData, newData);
      });
    }, 100),
    []
  );

  useEffect(() => {
    if (telemetryData) {
      updateStabilizedData(telemetryData);
    }
  }, [telemetryData, updateStabilizedData]);

  return (
    <div className="telemetry-display">
      <div className="telemetry-section">
        <h3>Battery</h3>
        <div className="telemetry-item">
          <span>Voltage:</span>
          <span>{stabilizedData.battery.voltage.toFixed(2)}V</span>
        </div>
        <div className="telemetry-item">
          <span>Current:</span>
          <span>{stabilizedData.battery.current.toFixed(2)}A</span>
        </div>
        <div className="telemetry-item">
          <span>Percentage:</span>
          <span>{stabilizedData.battery.percentage.toFixed(1)}%</span>
        </div>
      </div>

      <div className="telemetry-section">
        <h3>Temperature</h3>
        <div className="telemetry-item">
          <span>CPU:</span>
          <span>{stabilizedData.temperature.cpu.toFixed(1)}°C</span>
        </div>
        <div className="telemetry-item">
          <span>Motor:</span>
          <span>{stabilizedData.temperature.motor.toFixed(1)}°C</span>
        </div>
      </div>

      <div className="telemetry-section">
        <h3>Position</h3>
        <div className="telemetry-item">
          <span>X:</span>
          <span>{stabilizedData.position.x.toFixed(2)}m</span>
        </div>
        <div className="telemetry-item">
          <span>Y:</span>
          <span>{stabilizedData.position.y.toFixed(2)}m</span>
        </div>
        <div className="telemetry-item">
          <span>Z:</span>
          <span>{stabilizedData.position.z.toFixed(2)}m</span>
        </div>
      </div>

      <div className="telemetry-section">
        <h3>Orientation</h3>
        <div className="telemetry-item">
          <span>Roll:</span>
          <span>{stabilizedData.orientation.roll.toFixed(1)}°</span>
        </div>
        <div className="telemetry-item">
          <span>Pitch:</span>
          <span>{stabilizedData.orientation.pitch.toFixed(1)}°</span>
        </div>
        <div className="telemetry-item">
          <span>Yaw:</span>
          <span>{stabilizedData.orientation.yaw.toFixed(1)}°</span>
        </div>
      </div>

      <div className="telemetry-section">
        <h3>Velocity</h3>
        <div className="telemetry-item">
          <span>Linear X:</span>
          <span>{stabilizedData.velocity.linear.x.toFixed(2)}m/s</span>
        </div>
        <div className="telemetry-item">
          <span>Linear Y:</span>
          <span>{stabilizedData.velocity.linear.y.toFixed(2)}m/s</span>
        </div>
        <div className="telemetry-item">
          <span>Linear Z:</span>
          <span>{stabilizedData.velocity.linear.z.toFixed(2)}m/s</span>
        </div>
        <div className="telemetry-item">
          <span>Angular X:</span>
          <span>{stabilizedData.velocity.angular.x.toFixed(2)}rad/s</span>
        </div>
        <div className="telemetry-item">
          <span>Angular Y:</span>
          <span>{stabilizedData.velocity.angular.y.toFixed(2)}rad/s</span>
        </div>
        <div className="telemetry-item">
          <span>Angular Z:</span>
          <span>{stabilizedData.velocity.angular.z.toFixed(2)}rad/s</span>
        </div>
      </div>

      {stabilizedData.jointStates.length > 0 && (
        <div className="telemetry-section">
          <h3>Joint States</h3>
          {stabilizedData.jointStates.map((joint, index) => (
            <div key={index} className="telemetry-item">
              <span>{joint.name}:</span>
              <span>{joint.position.toFixed(2)}rad</span>
            </div>
          ))}
        </div>
      )}

      {stabilizedData.diagnostics.length > 0 && (
        <div className="telemetry-section">
          <h3>Diagnostics</h3>
          {stabilizedData.diagnostics.map((diag, index) => (
            <div key={index} className="telemetry-item">
              <span>{diag.name}:</span>
              <span className={`status-${diag.status.toLowerCase()}`}>{diag.status}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default TelemetryDisplay; 