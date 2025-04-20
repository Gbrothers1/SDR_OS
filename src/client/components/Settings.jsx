import React, { useState, useEffect } from 'react';
import '../styles/Settings.css';

const Settings = ({ isOpen, onClose, onSave }) => {
  const [settings, setSettings] = useState({
    controller: {
      joystickSensitivity: 1.0,
      joystickDeadzone: 0.1,
      triggerSensitivity: 1.0,
      buttonDebounceTime: 50,
    },
    telemetry: {
      publishingRate: 10,
      updateInterval: 100,
    },
    logging: {
      logLevel: 'info',
      filterTopics: '',
      maxLogEntries: 1000,
    },
    visualization: {
      showGrid: true,
      gridSize: 1.0,
      gridColor: '#333333',
    },
    control: {
      enableKeyboard: true,
      enableJoystick: true,
      enableTouch: true,
    },
    topics: {
      cmdVel: '/cmd_vel',
      odom: '/odom',
      jointStates: '/joint_states',
      diagnostics: '/diagnostics',
    },
  });

  useEffect(() => {
    // Load saved settings from localStorage
    const savedSettings = localStorage.getItem('robotControllerSettings');
    if (savedSettings) {
      setSettings(JSON.parse(savedSettings));
    }
  }, []);

  const handleChange = (section, field, value) => {
    setSettings(prev => ({
      ...prev,
      [section]: {
        ...prev[section],
        [field]: value,
      },
    }));
  };

  const handleSave = () => {
    localStorage.setItem('robotControllerSettings', JSON.stringify(settings));
    onSave(settings);
    onClose();
  };

  if (!isOpen) return null;

  return (
    <div className="settings-overlay">
      <div className="settings-modal">
        <div className="settings-header">
          <h2>Settings</h2>
          <button className="close-button" onClick={onClose}>&times;</button>
        </div>
        <div className="settings-content">
          <div className="settings-section">
            <h3>Controller Settings</h3>
            <div className="setting-item">
              <label>Joystick Sensitivity</label>
              <input
                type="range"
                min="0.1"
                max="2.0"
                step="0.1"
                value={settings.controller.joystickSensitivity}
                onChange={(e) => handleChange('controller', 'joystickSensitivity', parseFloat(e.target.value))}
              />
              <span className="setting-value">{settings.controller.joystickSensitivity.toFixed(1)}</span>
            </div>
            <div className="setting-item">
              <label>Joystick Deadzone</label>
              <input
                type="range"
                min="0"
                max="0.5"
                step="0.05"
                value={settings.controller.joystickDeadzone}
                onChange={(e) => handleChange('controller', 'joystickDeadzone', parseFloat(e.target.value))}
              />
              <span className="setting-value">{settings.controller.joystickDeadzone.toFixed(2)}</span>
            </div>
            <div className="setting-item">
              <label>Trigger Sensitivity</label>
              <input
                type="range"
                min="0.1"
                max="2.0"
                step="0.1"
                value={settings.controller.triggerSensitivity}
                onChange={(e) => handleChange('controller', 'triggerSensitivity', parseFloat(e.target.value))}
              />
              <span className="setting-value">{settings.controller.triggerSensitivity.toFixed(1)}</span>
            </div>
            <div className="setting-item">
              <label>Button Debounce Time (ms)</label>
              <input
                type="number"
                min="0"
                max="500"
                step="10"
                value={settings.controller.buttonDebounceTime}
                onChange={(e) => handleChange('controller', 'buttonDebounceTime', parseInt(e.target.value))}
              />
            </div>
          </div>

          <div className="settings-section">
            <h3>Telemetry Settings</h3>
            <div className="setting-item">
              <label>Publishing Rate (Hz)</label>
              <input
                type="number"
                min="1"
                max="100"
                value={settings.telemetry.publishingRate}
                onChange={(e) => handleChange('telemetry', 'publishingRate', parseInt(e.target.value))}
              />
            </div>
            <div className="setting-item">
              <label>Update Interval (ms)</label>
              <input
                type="number"
                min="10"
                max="1000"
                step="10"
                value={settings.telemetry.updateInterval}
                onChange={(e) => handleChange('telemetry', 'updateInterval', parseInt(e.target.value))}
              />
            </div>
          </div>

          <div className="settings-section">
            <h3>Log Settings</h3>
            <div className="setting-item">
              <label>Log Level</label>
              <select
                value={settings.logging.logLevel}
                onChange={(e) => handleChange('logging', 'logLevel', e.target.value)}
              >
                <option value="debug">Debug</option>
                <option value="info">Info</option>
                <option value="warn">Warning</option>
                <option value="error">Error</option>
              </select>
            </div>
            <div className="setting-item">
              <label>Filter Topics</label>
              <input
                type="text"
                value={settings.logging.filterTopics}
                onChange={(e) => handleChange('logging', 'filterTopics', e.target.value)}
                placeholder="Comma-separated topics"
              />
            </div>
            <div className="setting-item">
              <label>Max Log Entries</label>
              <input
                type="number"
                min="100"
                max="10000"
                step="100"
                value={settings.logging.maxLogEntries}
                onChange={(e) => handleChange('logging', 'maxLogEntries', parseInt(e.target.value))}
              />
            </div>
          </div>

          <div className="settings-section">
            <h3>Visualization Settings</h3>
            <div className="setting-item">
              <label>Show Grid</label>
              <input
                type="checkbox"
                checked={settings.visualization.showGrid}
                onChange={(e) => handleChange('visualization', 'showGrid', e.target.checked)}
              />
            </div>
            <div className="setting-item">
              <label>Grid Size</label>
              <input
                type="number"
                min="0.1"
                max="5.0"
                step="0.1"
                value={settings.visualization.gridSize}
                onChange={(e) => handleChange('visualization', 'gridSize', parseFloat(e.target.value))}
              />
            </div>
            <div className="setting-item">
              <label>Grid Color</label>
              <input
                type="color"
                value={settings.visualization.gridColor}
                onChange={(e) => handleChange('visualization', 'gridColor', e.target.value)}
              />
            </div>
          </div>

          <div className="settings-section">
            <h3>Control Settings</h3>
            <div className="setting-item">
              <label>Enable Keyboard</label>
              <input
                type="checkbox"
                checked={settings.control.enableKeyboard}
                onChange={(e) => handleChange('control', 'enableKeyboard', e.target.checked)}
              />
            </div>
            <div className="setting-item">
              <label>Enable Joystick</label>
              <input
                type="checkbox"
                checked={settings.control.enableJoystick}
                onChange={(e) => handleChange('control', 'enableJoystick', e.target.checked)}
              />
            </div>
            <div className="setting-item">
              <label>Enable Touch</label>
              <input
                type="checkbox"
                checked={settings.control.enableTouch}
                onChange={(e) => handleChange('control', 'enableTouch', e.target.checked)}
              />
            </div>
          </div>

          <div className="settings-section">
            <h3>Topic Settings</h3>
            <div className="setting-item">
              <label>Command Velocity Topic</label>
              <input
                type="text"
                value={settings.topics.cmdVel}
                onChange={(e) => handleChange('topics', 'cmdVel', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>Odometry Topic</label>
              <input
                type="text"
                value={settings.topics.odom}
                onChange={(e) => handleChange('topics', 'odom', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>Joint States Topic</label>
              <input
                type="text"
                value={settings.topics.jointStates}
                onChange={(e) => handleChange('topics', 'jointStates', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>Diagnostics Topic</label>
              <input
                type="text"
                value={settings.topics.diagnostics}
                onChange={(e) => handleChange('topics', 'diagnostics', e.target.value)}
              />
            </div>
          </div>
        </div>
        <div className="settings-footer">
          <button className="save-button" onClick={handleSave}>Save Settings</button>
        </div>
      </div>
    </div>
  );
};

export default Settings; 