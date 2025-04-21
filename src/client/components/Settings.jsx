import React, { useState, useEffect, useCallback } from 'react';
import '../styles/Settings.css';

const truncateVoiceName = (name) => {
  if (!name) return '';
  return name.length > 30 ? `${name.substring(0, 30)}...` : name;
};

const formatVoiceOption = (voice) => {
  if (!voice) return '';
  const truncatedName = truncateVoiceName(voice.name);
  return `${truncatedName} (${voice.lang})`;
};

const Settings = ({ isOpen, onClose, onSave, initialSettings }) => {
  const [settings, setSettings] = useState({
    telemetry: {
      showTelemetryPanel: true,
      updateInterval: 100,
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
      maxLinearSpeed: 1.0,
      maxAngularSpeed: 1.0,
      deadzone: 0.1,
    },
    topics: {
      cmd_vel: '/cmd_vel',
      odom: '/odom',
      imu: '/imu/data',
    },
    voice: {
      selectedVoice: '',
      volume: 1.0,
      rate: 1.0,
      pitch: 1.0,
    }
  });

  // Load settings on component mount or when initialSettings changes
  useEffect(() => {
    if (initialSettings) {
      // Deep merge settings ensuring telemetry and visualization nested objects are preserved
      setSettings(prevSettings => {
        const newSettings = { ...prevSettings };
        
        // Handle telemetry settings
        if (initialSettings.telemetry) {
          newSettings.telemetry = { 
            ...prevSettings.telemetry, 
            ...initialSettings.telemetry 
          };
        }
        
        // Handle visualization settings
        if (initialSettings.visualization) {
          newSettings.visualization = { 
            ...prevSettings.visualization, 
            ...initialSettings.visualization 
          };
        }
        
        // Handle control settings
        if (initialSettings.control) {
          newSettings.control = { 
            ...prevSettings.control, 
            ...initialSettings.control 
          };
        }
        
        // Handle topics settings
        if (initialSettings.topics) {
          newSettings.topics = { 
            ...prevSettings.topics, 
            ...initialSettings.topics 
          };
        }
        
        // Handle voice settings
        if (initialSettings.voice) {
          newSettings.voice = { 
            ...prevSettings.voice, 
            ...initialSettings.voice 
          };
        }
        
        return newSettings;
      });
    }
  }, [initialSettings, isOpen]);

  // Handle changes for nested settings
  const handleChange = useCallback((section, key, value) => {
    setSettings(prev => ({
      ...prev,
      [section]: {
        ...prev[section],
        [key]: value
      }
    }));
  }, []);

  // Handle save button click
  const handleSave = useCallback(() => {
    // Check if onSave is provided
    if (typeof onSave === 'function') {
      onSave(settings);
    }
    
    // Check if onClose is provided
    if (typeof onClose === 'function') {
      onClose();
    }
  }, [settings, onSave, onClose]);

  // Handle slider value updates
  const handleSliderChange = useCallback((section, key, value, min, max) => {
    const numValue = parseFloat(value);
    if (!isNaN(numValue) && numValue >= min && numValue <= max) {
      handleChange(section, key, numValue);
    }
  }, [handleChange]);

  // Update slider visual fill
  useEffect(() => {
    if (!isOpen) return;
    
    const sliders = document.querySelectorAll('input[type="range"]');
    sliders.forEach(slider => {
      const value = ((slider.value - slider.min) / (slider.max - slider.min)) * 100;
      slider.style.setProperty('--value-percent', `${value}%`);
    });
  }, [settings, isOpen]);

  const handleOverlayClick = useCallback((e) => {
    if (e.target.className === 'settings-overlay') {
      onClose();
    }
  }, [onClose]);

  if (!isOpen) return null;

  return (
    <div className="settings-overlay" onClick={handleOverlayClick}>
      <div className="settings-modal" onClick={e => e.stopPropagation()}>
        <div className="settings-modal-border"></div>
        <div className="settings-header">
          <h2>SETTINGS</h2>
          <button className="close-button" onClick={onClose}></button>
        </div>
        
        <div className="settings-content">
          {/* Telemetry Section */}
          <div className="settings-section">
            <h3>Telemetry</h3>
            <div className="setting-item">
              <label>Show Telemetry Panel</label>
              <input
                type="checkbox"
                checked={settings.telemetry.showTelemetryPanel}
                onChange={(e) => handleChange('telemetry', 'showTelemetryPanel', e.target.checked)}
              />
            </div>
            <div className="setting-item">
              <label>Update Interval (ms)</label>
              <input
                type="range"
                min="50"
                max="1000"
                value={settings.telemetry.updateInterval}
                onChange={(e) => handleSliderChange('telemetry', 'updateInterval', e.target.value, 50, 1000)}
              />
              <span className="setting-value">{settings.telemetry.updateInterval}ms</span>
            </div>
          </div>

          {/* Control Section */}
          <div className="settings-section">
            <h3>Control</h3>
            <div className="setting-item">
              <label>Enable Keyboard Control</label>
              <input
                type="checkbox"
                checked={settings.control.enableKeyboard}
                onChange={(e) => handleChange('control', 'enableKeyboard', e.target.checked)}
              />
            </div>
            <div className="setting-item">
              <label>Max Linear Speed (m/s)</label>
              <input
                type="range"
                min="0.1"
                max="2.0"
                step="0.1"
                value={settings.control.maxLinearSpeed}
                onChange={(e) => handleSliderChange('control', 'maxLinearSpeed', e.target.value, 0.1, 2.0)}
              />
              <span className="setting-value">{settings.control.maxLinearSpeed.toFixed(1)} m/s</span>
            </div>
            <div className="setting-item">
              <label>Max Angular Speed (rad/s)</label>
              <input
                type="range"
                min="0.1"
                max="3.0"
                step="0.1"
                value={settings.control.maxAngularSpeed}
                onChange={(e) => handleSliderChange('control', 'maxAngularSpeed', e.target.value, 0.1, 3.0)}
              />
              <span className="setting-value">{settings.control.maxAngularSpeed.toFixed(1)} rad/s</span>
            </div>
          </div>

          {/* Visualization Section */}
          <div className="settings-section">
            <h3>Visualization</h3>
            <div className="setting-item">
              <label>Show Grid</label>
              <input
                type="checkbox"
                checked={settings.visualization.showGrid}
                onChange={(e) => handleChange('visualization', 'showGrid', e.target.checked)}
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
            <div className="setting-item">
              <label>Grid Size</label>
              <input
                type="range"
                min="0.1"
                max="5.0"
                step="0.1"
                value={settings.visualization.gridSize}
                onChange={(e) => handleSliderChange('visualization', 'gridSize', e.target.value, 0.1, 5.0)}
              />
              <span className="setting-value">{settings.visualization.gridSize.toFixed(1)}m</span>
            </div>
          </div>

          {/* Topics Section */}
          <div className="settings-section">
            <h3>Topics</h3>
            <div className="setting-item">
              <label>Command Velocity</label>
              <input
                type="text"
                value={settings.topics.cmd_vel}
                onChange={(e) => handleChange('topics', 'cmd_vel', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>Odometry</label>
              <input
                type="text"
                value={settings.topics.odom}
                onChange={(e) => handleChange('topics', 'odom', e.target.value)}
              />
            </div>
            <div className="setting-item">
              <label>IMU Data</label>
              <input
                type="text"
                value={settings.topics.imu}
                onChange={(e) => handleChange('topics', 'imu', e.target.value)}
              />
            </div>
          </div>
        </div>

        <div className="settings-footer">
          <button className="save-button" onClick={handleSave}>
            Save Changes
          </button>
        </div>
      </div>
    </div>
  );
};

export default Settings; 