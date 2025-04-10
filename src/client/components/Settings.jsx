import React, { useState, useEffect } from 'react';
import '../styles/Settings.css';

const Settings = ({ isOpen, onClose, settings, onSettingsChange }) => {
  const [localSettings, setLocalSettings] = useState(settings);

  useEffect(() => {
    setLocalSettings(settings);
  }, [settings]);

  const handleChange = (section, key, value) => {
    const newSettings = {
      ...localSettings,
      [section]: {
        ...localSettings[section],
        [key]: value
      }
    };
    setLocalSettings(newSettings);
    onSettingsChange(newSettings);
  };

  const handleSave = () => {
    onSettingsChange(localSettings);
    onClose();
  };

  if (!isOpen) return null;

  return (
    <div className="settings-overlay">
      <div className="settings-modal">
        <div className="settings-header">
          <h2>Settings</h2>
          <button className="close-button" onClick={onClose}>×</button>
        </div>

        <div className="settings-content">
          {/* Network Settings */}
          <section className="settings-section">
            <h3>Network</h3>
            <div className="setting-item">
              <label>ROS Bridge IP Address:</label>
              <input
                type="text"
                value={localSettings.network.rosBridgeIp}
                onChange={(e) => handleChange('network', 'rosBridgeIp', e.target.value)}
                placeholder="localhost"
              />
            </div>
            <div className="setting-item">
              <label>ROS Bridge Port:</label>
              <input
                type="number"
                value={localSettings.network.rosBridgePort}
                onChange={(e) => handleChange('network', 'rosBridgePort', parseInt(e.target.value))}
                placeholder="9090"
              />
            </div>
          </section>

          {/* Visualization Settings */}
          <section className="settings-section">
            <h3>Visualization</h3>
            <div className="setting-item">
              <label>Background Color:</label>
              <input
                type="color"
                value={localSettings.visualization.backgroundColor}
                onChange={(e) => handleChange('visualization', 'backgroundColor', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>Grid Color:</label>
              <input
                type="color"
                value={localSettings.visualization.gridColor}
                onChange={(e) => handleChange('visualization', 'gridColor', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>Grid Size:</label>
              <input
                type="number"
                value={localSettings.visualization.gridSize}
                onChange={(e) => handleChange('visualization', 'gridSize', parseInt(e.target.value))}
                min="1"
                max="100"
              />
            </div>
            <div className="setting-item">
              <label>Grid Thickness:</label>
              <input
                type="number"
                value={localSettings.visualization.gridThickness}
                onChange={(e) => handleChange('visualization', 'gridThickness', parseInt(e.target.value))}
                min="1"
                max="10"
              />
            </div>
          </section>

          {/* Control Settings */}
          <section className="settings-section">
            <h3>Controls</h3>
            <div className="setting-item">
              <label>Camera Control Mode:</label>
              <select
                value={localSettings.controls.cameraControlMode}
                onChange={(e) => handleChange('controls', 'cameraControlMode', e.target.value)}
              >
                <option value="always">Always</option>
                <option value="toggle">Toggle (Hotkey)</option>
                <option value="never">Never</option>
              </select>
            </div>
            <div className="setting-item">
              <label>Camera Control Hotkey:</label>
              <input
                type="text"
                value={localSettings.controls.cameraControlHotkey}
                onChange={(e) => handleChange('controls', 'cameraControlHotkey', e.target.value)}
                placeholder="Alt"
                disabled={localSettings.controls.cameraControlMode !== 'toggle'}
              />
            </div>
          </section>

          {/* Topic Settings */}
          <section className="settings-section">
            <h3>Topics</h3>
            <div className="setting-item">
              <label>Command Velocity Topic:</label>
              <input
                type="text"
                value={localSettings.topics.cmdVel}
                onChange={(e) => handleChange('topics', 'cmdVel', e.target.value)}
                placeholder="/cmd_vel"
              />
            </div>
            <div className="setting-item">
              <label>Odometry Topic:</label>
              <input
                type="text"
                value={localSettings.topics.odom}
                onChange={(e) => handleChange('topics', 'odom', e.target.value)}
                placeholder="/odom"
              />
            </div>
            <div className="setting-item">
              <label>Joint States Topic:</label>
              <input
                type="text"
                value={localSettings.topics.jointStates}
                onChange={(e) => handleChange('topics', 'jointStates', e.target.value)}
                placeholder="/joint_states"
              />
            </div>
          </section>
        </div>

        <div className="settings-footer">
          <button className="save-button" onClick={handleSave}>Save Settings</button>
        </div>
      </div>
    </div>
  );
};

export default Settings; 