import React, { useState, useEffect } from 'react';
import '../styles/Settings.css';

const Settings = ({ isOpen, onClose, onSave, initialSettings }) => {
  const [settings, setSettings] = useState({
    rosbridgeUrl: localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090',
    socketUrl: localStorage.getItem('socketUrl') || 'http://localhost:3000',
    controller: {
      joystickSensitivity: 1.0,
      joystickDeadzone: 0.1,
      triggerSensitivity: 1.0,
      buttonDebounceTime: 50,
    },
    telemetry: {
      updateInterval: 100,
      showTelemetryPanel: true,
    },
    logging: {
      logLevel: 'info',
      filterTopics: '',
      maxLogEntries: 1000,
      subscribeRosout: true,
      subscribeDiagnostics: true,
    },
    visualization: {
      showGrid: true,
      gridSize: 1.0,
      gridColor: '#333333',
      tfThrottleRate: 10,
      showCameraFeed: false,
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
    const savedSettings = localStorage.getItem('robotControllerSettings');
    let loadedSettings = {};
    if (initialSettings) {
      loadedSettings = initialSettings;
    } else if (savedSettings) {
      loadedSettings = JSON.parse(savedSettings);
    }

    setSettings(prev => ({
      ...prev,
      ...loadedSettings,
      rosbridgeUrl: loadedSettings.rosbridgeUrl || localStorage.getItem('rosBridgeUrl') || prev.rosbridgeUrl,
      socketUrl: loadedSettings.socketUrl || localStorage.getItem('socketUrl') || prev.socketUrl,
      controller: { ...prev.controller, ...(loadedSettings.controller || {}) },
      telemetry: {
        ...prev.telemetry,
        ...(loadedSettings.telemetry || {}),
        updateInterval: loadedSettings.telemetry?.updateInterval !== undefined ? loadedSettings.telemetry.updateInterval : 100,
        showTelemetryPanel: loadedSettings.telemetry?.showTelemetryPanel !== undefined ? loadedSettings.telemetry.showTelemetryPanel : true,
      },
      logging: {
        ...prev.logging,
        ...(loadedSettings.logging || {}),
        subscribeRosout: loadedSettings.logging?.subscribeRosout !== undefined ? loadedSettings.logging.subscribeRosout : true,
        subscribeDiagnostics: loadedSettings.logging?.subscribeDiagnostics !== undefined ? loadedSettings.logging.subscribeDiagnostics : true,
      },
      visualization: {
        ...prev.visualization,
        ...(loadedSettings.visualization || {}),
        tfThrottleRate: loadedSettings.visualization?.tfThrottleRate !== undefined ? loadedSettings.visualization.tfThrottleRate : 10,
        showCameraFeed: loadedSettings.visualization?.showCameraFeed !== undefined ? loadedSettings.visualization.showCameraFeed : false,
      },
      control: { ...prev.control, ...(loadedSettings.control || {}) },
      topics: { ...prev.topics, ...(loadedSettings.topics || {}) },
    }));
  }, [initialSettings]);

  const handleChange = (section, field, value) => {
    if (section === 'connection') {
      setSettings(prev => ({
        ...prev,
        [field]: value,
      }));
    } else {
      setSettings(prev => ({
        ...prev,
        [section]: {
          ...prev[section],
          [field]: value,
        },
      }));
    }
  };

  const handleSave = () => {
    localStorage.setItem('rosBridgeUrl', settings.rosbridgeUrl);
    localStorage.setItem('socketUrl', settings.socketUrl);
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
            <h3>Connection Settings</h3>
            <div className="setting-item">
              <label>ROS Bridge URL</label>
              <input
                type="text"
                value={settings.rosbridgeUrl}
                onChange={(e) => handleChange('connection', 'rosbridgeUrl', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>Socket.IO URL</label>
              <input
                type="text"
                value={settings.socketUrl}
                onChange={(e) => handleChange('connection', 'socketUrl', e.target.value)}
              />
            </div>
          </div>

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
            <div className="setting-item">
              <label>Show Telemetry Panel</label>
              <input
                type="checkbox"
                checked={settings.telemetry.showTelemetryPanel}
                onChange={(e) => handleChange('telemetry', 'showTelemetryPanel', e.target.checked)}
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
            <div className="setting-item">
              <label>Subscribe to /rosout</label>
              <input
                type="checkbox"
                checked={settings.logging.subscribeRosout}
                onChange={(e) => handleChange('logging', 'subscribeRosout', e.target.checked)}
              />
            </div>
            <div className="setting-item">
              <label>Subscribe to /diagnostics</label>
              <input
                type="checkbox"
                checked={settings.logging.subscribeDiagnostics}
                onChange={(e) => handleChange('logging', 'subscribeDiagnostics', e.target.checked)}
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
            <div className="setting-item">
              <label>TF Throttle Rate (Hz)</label>
              <input
                type="number"
                min="1"
                max="60"
                step="1"
                value={settings.visualization.tfThrottleRate}
                onChange={(e) => handleChange('visualization', 'tfThrottleRate', parseInt(e.target.value) || 1)}
              />
            </div>
            <div className="setting-item">
              <label>Show Camera Feed</label>
              <input
                type="checkbox"
                checked={settings.visualization.showCameraFeed}
                onChange={(e) => handleChange('visualization', 'showCameraFeed', e.target.checked)}
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