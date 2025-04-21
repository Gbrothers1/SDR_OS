import React, { useState, useEffect, useCallback } from 'react';
import '../styles/Settings.css';

// Default settings structure
const defaultSettings = {
  telemetry: { showTelemetryPanel: true, updateInterval: 100 },
  visualization: { showGrid: true, gridSize: 1.0, gridColor: '#333333', tfThrottleRate: 10, showCameraFeed: false },
  control: { enableKeyboard: true, maxLinearSpeed: 1.0, maxAngularSpeed: 1.0, deadzone: 0.1 },
  topics: { cmd_vel: '/cmd_vel', odom: '/odom', imu: '/imu/data' },
  voice: { selectedVoice: '', volume: 1.0, rate: 1.0, pitch: 1.0 }
};

// Helper for deep merging settings, preserving structure
const mergeSettings = (base, updates) => {
  const merged = { ...base };
  for (const section of Object.keys(defaultSettings)) {
    if (updates && updates[section]) {
      merged[section] = { ...base[section], ...updates[section] };
    }
  }
  // Handle legacy flat structure if necessary (example: connection settings)
  if (updates?.rosBridgeUrl) merged.connection = { ...(merged.connection || {}), rosBridgeUrl: updates.rosBridgeUrl };
  if (updates?.socketUrl) merged.connection = { ...(merged.connection || {}), socketUrl: updates.socketUrl };
  if (updates?.webVideoHost) merged.connection = { ...(merged.connection || {}), webVideoHost: updates.webVideoHost };

  return merged;
};

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
  const [settings, setSettings] = useState(defaultSettings);
  // Track which sections are expanded - start with all expanded
  const [expandedSections, setExpandedSections] = useState({
    telemetry: true,
    control: true,
    visualization: true,
    topics: true,
    connection: true
  });

  // Toggle section expansion
  const toggleSection = (section) => {
    setExpandedSections(prev => ({
      ...prev,
      [section]: !prev[section]
    }));
  };

  // Effect to load/merge settings when the modal opens or initialSettings change
  useEffect(() => {
    if (isOpen) {
      console.log("[Settings Effect] Modal opened. Loading initial settings:", initialSettings);
      // Start with defaults, then merge initialSettings passed as prop
      const merged = mergeSettings(defaultSettings, initialSettings);
      console.log("[Settings Effect] Merged settings:", merged);
      setSettings(merged);
    }
  }, [isOpen, initialSettings]);

  // Handle changes for nested settings
  const handleChange = useCallback((section, key, value) => {
    console.log(`[Settings handleChange] Section: ${section}, Key: ${key}, Value: ${value}`);
    setSettings(prev => {
      const newState = {
        ...prev,
        [section]: {
          ...prev[section],
          [key]: value
        }
      };
      console.log("[Settings handleChange] New state:", newState);
      return newState;
    });
  }, []);

  // Handle save button click
  const handleSave = useCallback(() => {
    console.log("[Settings handleSave] Saving settings:", settings);
    // Also save connection settings back to localStorage directly as they are not part of the main state object
    const connectionSettings = settings.connection || {};
    if (connectionSettings.rosBridgeUrl) localStorage.setItem('rosBridgeUrl', connectionSettings.rosBridgeUrl);
    if (connectionSettings.socketUrl) localStorage.setItem('socketUrl', connectionSettings.socketUrl);
    if (connectionSettings.webVideoHost) localStorage.setItem('webVideoHost', connectionSettings.webVideoHost);
    
    onSave(settings); // Pass the current state up
    onClose();
  }, [settings, onSave, onClose]);

  // Handle slider value updates
  const handleSliderChange = useCallback((section, key, value, min, max) => {
    const numValue = parseFloat(value);
    if (!isNaN(numValue) && numValue >= min && numValue <= max) {
      handleChange(section, key, numValue);
    } else {
      console.warn(`[Settings handleSliderChange] Invalid slider value: ${value}`);
    }
  }, [handleChange]);

  // Update slider visual fill after state updates
  useEffect(() => {
    if (!isOpen) return;
    
    // Use requestAnimationFrame for smoother updates after render
    const animationFrameId = requestAnimationFrame(() => {
      const sliders = document.querySelectorAll('.settings-content input[type="range"]');
      // console.log(`[Settings Slider Effect] Found ${sliders.length} sliders to update.`);
      sliders.forEach(slider => {
        const min = parseFloat(slider.min) || 0;
        const max = parseFloat(slider.max) || 100;
        const val = parseFloat(slider.value);
        
        if (!isNaN(val)) {
          const percentage = max === min ? 0 : ((val - min) / (max - min)) * 100;
          // console.log(`[Settings Slider Effect] Slider ${slider.id || 'no-id'}: value=${val}, min=${min}, max=${max}, percent=${percentage}%`);
          slider.style.setProperty('--value-percent', `${Math.max(0, Math.min(100, percentage))}%`);
        } else {
          // console.warn(`[Settings Slider Effect] Slider ${slider.id || 'no-id'} has invalid value: ${slider.value}`);
        }
      });
    });

    return () => cancelAnimationFrame(animationFrameId); // Cleanup

  }, [settings, isOpen]); // Re-run when settings or isOpen changes

  // Add explicit wheel event handler
  const handleWheel = useCallback((e) => {
    // Prevent the wheel event from propagating to parent elements
    e.stopPropagation();
    
    // Ensure the content area handles the scroll natively
    // This is a fallback in case normal scrolling doesn't work
    if (e.currentTarget) {
      const scrollAmount = e.deltaY;
      e.currentTarget.scrollTop += scrollAmount;
      console.log("Manual wheel handling: scrollTop =", e.currentTarget.scrollTop);
    }
  }, []);

  if (!isOpen) return null;

  // Helper to safely get nested values
  const getSetting = (section, key, defaultValue = '') => {
    return settings?.[section]?.[key] ?? defaultValue;
  };

  // Render a chevron indicator for section headings
  const renderChevron = (section) => {
    return (
      <span className={`section-chevron ${expandedSections[section] ? 'expanded' : 'collapsed'}`}>
        {expandedSections[section] ? '▼' : '►'}
      </span>
    );
  };

  return (
    <div className="settings-overlay" onClick={(e) => {
      // Close only if the overlay itself (not content) is clicked
      if (e.target === e.currentTarget) onClose();
    }}>
      <div className="settings-modal" onClick={e => e.stopPropagation()}> 
        <div className="settings-modal-border"></div>
        <div className="settings-header">
          <h2>SETTINGS</h2>
          <button className="close-button" onClick={onClose} aria-label="Close settings"></button>
        </div>
        
        <div 
          className="settings-content" 
          onWheel={handleWheel} 
          tabIndex="0"
        >
          <div className="settings-grid">
            {/* Telemetry Section */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('telemetry')} className="section-header">
                {renderChevron('telemetry')} Telemetry
              </h3>
              {expandedSections.telemetry && (
                <div className="section-content">
                  <div className="setting-item">
                    <label htmlFor="telemetry-show">Show Telemetry Panel</label>
                    <input
                      id="telemetry-show"
                      type="checkbox"
                      checked={getSetting('telemetry', 'showTelemetryPanel', true)}
                      onChange={(e) => handleChange('telemetry', 'showTelemetryPanel', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="telemetry-interval">Update Interval (ms)</label>
                    <input
                      id="telemetry-interval"
                      type="range"
                      min="50"
                      max="1000"
                      value={getSetting('telemetry', 'updateInterval', 100)}
                      onChange={(e) => handleSliderChange('telemetry', 'updateInterval', e.target.value, 50, 1000)}
                    />
                    <span className="setting-value">{getSetting('telemetry', 'updateInterval', 100)}ms</span>
                  </div>
                </div>
              )}
            </div>

            {/* Control Section */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('control')} className="section-header">
                {renderChevron('control')} Control
              </h3>
              {expandedSections.control && (
                <div className="section-content">
                  <div className="setting-item">
                    <label htmlFor="control-keyboard">Enable Keyboard Control</label>
                    <input
                      id="control-keyboard"
                      type="checkbox"
                      checked={getSetting('control', 'enableKeyboard', true)}
                      onChange={(e) => handleChange('control', 'enableKeyboard', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="control-linear-speed">Max Linear Speed (m/s)</label>
                    <input
                      id="control-linear-speed"
                      type="range"
                      min="0.1"
                      max="2.0"
                      step="0.1"
                      value={getSetting('control', 'maxLinearSpeed', 1.0)}
                      onChange={(e) => handleSliderChange('control', 'maxLinearSpeed', e.target.value, 0.1, 2.0)}
                    />
                    <span className="setting-value">{getSetting('control', 'maxLinearSpeed', 1.0).toFixed(1)} m/s</span>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="control-angular-speed">Max Angular Speed (rad/s)</label>
                    <input
                      id="control-angular-speed"
                      type="range"
                      min="0.1"
                      max="3.0"
                      step="0.1"
                      value={getSetting('control', 'maxAngularSpeed', 1.0)}
                      onChange={(e) => handleSliderChange('control', 'maxAngularSpeed', e.target.value, 0.1, 3.0)}
                    />
                    <span className="setting-value">{getSetting('control', 'maxAngularSpeed', 1.0).toFixed(1)} rad/s</span>
                  </div>
                </div>
              )}
            </div>

            {/* Visualization Section */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('visualization')} className="section-header">
                {renderChevron('visualization')} Visualization
              </h3>
              {expandedSections.visualization && (
                <div className="section-content">
                  <div className="setting-item">
                    <label htmlFor="viz-grid">Show Grid</label>
                    <input
                      id="viz-grid"
                      type="checkbox"
                      checked={getSetting('visualization', 'showGrid', true)}
                      onChange={(e) => handleChange('visualization', 'showGrid', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="viz-camera">Show Camera Feed</label>
                    <input
                      id="viz-camera"
                      type="checkbox"
                      checked={getSetting('visualization', 'showCameraFeed', false)}
                      onChange={(e) => handleChange('visualization', 'showCameraFeed', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="viz-grid-size">Grid Size</label>
                    <input
                      id="viz-grid-size"
                      type="range"
                      min="0.1"
                      max="5.0"
                      step="0.1"
                      value={getSetting('visualization', 'gridSize', 1.0)}
                      onChange={(e) => handleSliderChange('visualization', 'gridSize', e.target.value, 0.1, 5.0)}
                    />
                    <span className="setting-value">{getSetting('visualization', 'gridSize', 1.0).toFixed(1)}m</span>
                  </div>
                </div>
              )}
            </div>

            {/* Topics Section */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('topics')} className="section-header">
                {renderChevron('topics')} Topics
              </h3>
              {expandedSections.topics && (
                <div className="section-content">
                  <div className="setting-item">
                    <label htmlFor="topics-cmdvel">Command Velocity</label>
                    <input
                      id="topics-cmdvel"
                      type="text"
                      value={getSetting('topics', 'cmd_vel', '/cmd_vel')}
                      onChange={(e) => handleChange('topics', 'cmd_vel', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="topics-odom">Odometry</label>
                    <input
                      id="topics-odom"
                      type="text"
                      value={getSetting('topics', 'odom', '/odom')}
                      onChange={(e) => handleChange('topics', 'odom', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="topics-imu">IMU Data</label>
                    <input
                      id="topics-imu"
                      type="text"
                      value={getSetting('topics', 'imu', '/imu/data')}
                      onChange={(e) => handleChange('topics', 'imu', e.target.value)}
                    />
                  </div>
                </div>
              )}
            </div>
            
            {/* Connection Settings Section - Still reads/writes directly to localStorage */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('connection')} className="section-header">
                {renderChevron('connection')} Connection
              </h3>
              {expandedSections.connection && (
                <div className="section-content">
                  <div className="setting-item">
                    <label htmlFor="conn-ros">ROS Bridge URL</label>
                    <input
                      id="conn-ros"
                      type="text"
                      defaultValue={localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090'}
                      onChange={(e) => handleChange('connection', 'rosBridgeUrl', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="conn-socket">Socket.IO URL</label>
                    <input
                      id="conn-socket"
                      type="text"
                      defaultValue={localStorage.getItem('socketUrl') || 'http://localhost:3000'}
                      onChange={(e) => handleChange('connection', 'socketUrl', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="conn-video">Web Video Host</label>
                    <input
                      id="conn-video"
                      type="text"
                      defaultValue={localStorage.getItem('webVideoHost') || 'localhost'}
                      onChange={(e) => handleChange('connection', 'webVideoHost', e.target.value)}
                    />
                  </div>
                </div>
              )}
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