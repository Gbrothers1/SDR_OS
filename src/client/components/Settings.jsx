import React, { useState, useEffect, useCallback } from 'react';
import '../styles/Settings.css';

// Default settings structure
const defaultSettings = {
  ui: {
    showControllerOnStartup: true,
    controllerVisible: true,
    controllerMinimized: false,
    showTelemetryPanel: true,
    showLogsOnStartup: false,
    showCameraOnStartup: false
  },
  telemetry: { 
    updateInterval: 100 
  },
  visualization: { 
    showGrid: true, 
    gridSize: 1.0, 
    gridColor: '#333333', 
    tfThrottleRate: 10, 
    showRobotLabels: true,
    showImuIndicators: true,
    showAxisLabels: true,
    cameraPort: '8080',
    cameraQuality: 'medium',
    cameraRefreshRate: 30
  },
  control: { 
    enableKeyboard: true, 
    maxLinearSpeed: 1.0, 
    maxAngularSpeed: 1.0, 
    deadzone: 0.1,
    enableSoundEffects: true,
    enableHapticFeedback: true,
    joystickSensitivity: 1.0,
    triggerSensitivity: 1.0
  },
  topics: { 
    cmd_vel: '/cmd_vel', 
    odom: '/odom', 
    imu: '/imu/data',
    joint_states: '/joint_states',
    tf: '/tf',
    tf_static: '/tf_static',
    button_states: '/controller/button_states',
    joystick_state: '/controller/joystick_state',
    camera: '/webcam/image_raw'
  },
  voice: { 
    selectedVoice: '', 
    volume: 1.0, 
    rate: 1.0, 
    pitch: 1.0,
    defaultLanguage: 'en-US',
    enabled: true,
    announceStatus: true,
    announceErrors: true
  }
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
    ui: true,
    telemetry: true,
    control: true,
    visualization: true,
    topics: true,
    connection: true,
    voice: true
  });
  
  // State for available voices
  const [availableVoices, setAvailableVoices] = useState([]);
  
  // Load available voices
  useEffect(() => {
    const loadVoices = () => {
      if ('speechSynthesis' in window) {
        const voices = window.speechSynthesis.getVoices();
        setAvailableVoices(voices);
      }
    };
    
    // Load voices immediately if available
    loadVoices();
    
    // Also listen for voices to be loaded
    if (window.speechSynthesis && window.speechSynthesis.onvoiceschanged !== undefined) {
      window.speechSynthesis.onvoiceschanged = loadVoices;
    }
    
    return () => {
      if (window.speechSynthesis && window.speechSynthesis.onvoiceschanged !== undefined) {
        window.speechSynthesis.onvoiceschanged = null;
      }
    };
  }, [isOpen]);
  
  // Test the current voice
  const testVoice = useCallback(() => {
    if ('speechSynthesis' in window) {
      const selectedVoiceName = getSetting('voice', 'selectedVoice', '');
      const volume = parseFloat(getSetting('voice', 'volume', 1.0));
      const rate = parseFloat(getSetting('voice', 'rate', 1.0));
      const pitch = parseFloat(getSetting('voice', 'pitch', 1.0));
      
      // Find the voice by name
      let voice = null;
      if (selectedVoiceName) {
        voice = availableVoices.find(v => v.name === selectedVoiceName);
      }
      
      // If no voice selected, try to find a voice matching the default language
      if (!voice) {
        const defaultLang = getSetting('voice', 'defaultLanguage', 'en-US');
        voice = availableVoices.find(v => v.lang === defaultLang);
      }
      
      // Still no voice? Use the first available
      if (!voice && availableVoices.length > 0) {
        voice = availableVoices[0];
      }
      
      if (voice) {
        const utterance = new SpeechSynthesisUtterance('Testing text to speech settings');
        utterance.voice = voice;
        utterance.volume = volume;
        utterance.rate = rate;
        utterance.pitch = pitch;
        window.speechSynthesis.speak(utterance);
      }
    }
  }, [availableVoices, getSetting]);

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
            {/* UI Section */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('ui')} className="section-header">
                {renderChevron('ui')} User Interface
              </h3>
              {expandedSections.ui && (
                <div className="section-content">
                  <div className="setting-item">
                    <label htmlFor="ui-controller-startup">Show Controller on Next Startup</label>
                    <input
                      id="ui-controller-startup"
                      type="checkbox"
                      checked={getSetting('ui', 'showControllerOnStartup', true)}
                      onChange={(e) => handleChange('ui', 'showControllerOnStartup', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="ui-controller-visible">Controller Currently Visible</label>
                    <input
                      id="ui-controller-visible"
                      type="checkbox"
                      checked={getSetting('ui', 'controllerVisible', true)}
                      onChange={(e) => handleChange('ui', 'controllerVisible', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="ui-controller-minimized">Controller Currently Minimized</label>
                    <input
                      id="ui-controller-minimized"
                      type="checkbox"
                      checked={getSetting('ui', 'controllerMinimized', false)}
                      onChange={(e) => handleChange('ui', 'controllerMinimized', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="ui-telemetry">Show Telemetry Panel</label>
                    <input
                      id="ui-telemetry"
                      type="checkbox"
                      checked={getSetting('ui', 'showTelemetryPanel', true)}
                      onChange={(e) => handleChange('ui', 'showTelemetryPanel', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="ui-logs-startup">Show Logs on Startup</label>
                    <input
                      id="ui-logs-startup"
                      type="checkbox"
                      checked={getSetting('ui', 'showLogsOnStartup', false)}
                      onChange={(e) => handleChange('ui', 'showLogsOnStartup', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="ui-camera-startup">Show Camera on Startup</label>
                    <input
                      id="ui-camera-startup"
                      type="checkbox"
                      checked={getSetting('ui', 'showCameraOnStartup', false)}
                      onChange={(e) => handleChange('ui', 'showCameraOnStartup', e.target.checked)}
                    />
                  </div>
                </div>
              )}
            </div>

            {/* Telemetry Section */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('telemetry')} className="section-header">
                {renderChevron('telemetry')} Telemetry
              </h3>
              {expandedSections.telemetry && (
                <div className="section-content">
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
                  <div className="setting-item">
                    <label htmlFor="control-deadzone">Joystick Deadzone</label>
                    <input
                      id="control-deadzone"
                      type="range"
                      min="0.0"
                      max="0.5"
                      step="0.05"
                      value={getSetting('control', 'deadzone', 0.1)}
                      onChange={(e) => handleSliderChange('control', 'deadzone', e.target.value, 0.0, 0.5)}
                    />
                    <span className="setting-value">{getSetting('control', 'deadzone', 0.1).toFixed(2)}</span>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="control-sound">Enable Sound Effects</label>
                    <input
                      id="control-sound"
                      type="checkbox"
                      checked={getSetting('control', 'enableSoundEffects', true)}
                      onChange={(e) => handleChange('control', 'enableSoundEffects', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="control-haptic">Enable Haptic Feedback</label>
                    <input
                      id="control-haptic"
                      type="checkbox"
                      checked={getSetting('control', 'enableHapticFeedback', true)}
                      onChange={(e) => handleChange('control', 'enableHapticFeedback', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="control-joystick-sensitivity">Joystick Sensitivity</label>
                    <input
                      id="control-joystick-sensitivity"
                      type="range"
                      min="0.5"
                      max="2.0"
                      step="0.1"
                      value={getSetting('control', 'joystickSensitivity', 1.0)}
                      onChange={(e) => handleSliderChange('control', 'joystickSensitivity', e.target.value, 0.5, 2.0)}
                    />
                    <span className="setting-value">{getSetting('control', 'joystickSensitivity', 1.0).toFixed(1)}x</span>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="control-trigger-sensitivity">Trigger Sensitivity</label>
                    <input
                      id="control-trigger-sensitivity"
                      type="range"
                      min="0.5"
                      max="2.0"
                      step="0.1"
                      value={getSetting('control', 'triggerSensitivity', 1.0)}
                      onChange={(e) => handleSliderChange('control', 'triggerSensitivity', e.target.value, 0.5, 2.0)}
                    />
                    <span className="setting-value">{getSetting('control', 'triggerSensitivity', 1.0).toFixed(1)}x</span>
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
                  <div className="setting-item">
                    <label htmlFor="viz-robot-labels">Show Robot Labels</label>
                    <input
                      id="viz-robot-labels"
                      type="checkbox"
                      checked={getSetting('visualization', 'showRobotLabels', true)}
                      onChange={(e) => handleChange('visualization', 'showRobotLabels', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="viz-imu-indicators">Show IMU Indicators</label>
                    <input
                      id="viz-imu-indicators"
                      type="checkbox"
                      checked={getSetting('visualization', 'showImuIndicators', true)}
                      onChange={(e) => handleChange('visualization', 'showImuIndicators', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="viz-axis-labels">Show Axis Labels</label>
                    <input
                      id="viz-axis-labels"
                      type="checkbox"
                      checked={getSetting('visualization', 'showAxisLabels', true)}
                      onChange={(e) => handleChange('visualization', 'showAxisLabels', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="viz-camera-port">Camera Port</label>
                    <input
                      id="viz-camera-port"
                      type="text"
                      value={getSetting('visualization', 'cameraPort', '8080')}
                      onChange={(e) => handleChange('visualization', 'cameraPort', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="viz-camera-quality">Camera Quality</label>
                    <select
                      id="viz-camera-quality"
                      value={getSetting('visualization', 'cameraQuality', 'medium')}
                      onChange={(e) => handleChange('visualization', 'cameraQuality', e.target.value)}
                    >
                      <option value="low">Low</option>
                      <option value="medium">Medium</option>
                      <option value="high">High</option>
                    </select>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="viz-camera-refresh">Camera Refresh Rate (fps)</label>
                    <input
                      id="viz-camera-refresh"
                      type="range"
                      min="5"
                      max="60"
                      step="5"
                      value={getSetting('visualization', 'cameraRefreshRate', 30)}
                      onChange={(e) => handleSliderChange('visualization', 'cameraRefreshRate', e.target.value, 5, 60)}
                    />
                    <span className="setting-value">{getSetting('visualization', 'cameraRefreshRate', 30)} fps</span>
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
                  <div className="setting-item">
                    <label htmlFor="topics-joint-states">Joint States</label>
                    <input
                      id="topics-joint-states"
                      type="text"
                      value={getSetting('topics', 'joint_states', '/joint_states')}
                      onChange={(e) => handleChange('topics', 'joint_states', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="topics-tf">TF</label>
                    <input
                      id="topics-tf"
                      type="text"
                      value={getSetting('topics', 'tf', '/tf')}
                      onChange={(e) => handleChange('topics', 'tf', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="topics-tf-static">TF Static</label>
                    <input
                      id="topics-tf-static"
                      type="text"
                      value={getSetting('topics', 'tf_static', '/tf_static')}
                      onChange={(e) => handleChange('topics', 'tf_static', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="topics-button-states">Button States</label>
                    <input
                      id="topics-button-states"
                      type="text"
                      value={getSetting('topics', 'button_states', '/controller/button_states')}
                      onChange={(e) => handleChange('topics', 'button_states', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="topics-joystick-state">Joystick State</label>
                    <input
                      id="topics-joystick-state"
                      type="text"
                      value={getSetting('topics', 'joystick_state', '/controller/joystick_state')}
                      onChange={(e) => handleChange('topics', 'joystick_state', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="topics-camera">Camera</label>
                    <input
                      id="topics-camera"
                      type="text"
                      value={getSetting('topics', 'camera', '/webcam/image_raw')}
                      onChange={(e) => handleChange('topics', 'camera', e.target.value)}
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

            {/* Voice Settings Section */}
            <div className="settings-section">
              <h3 onClick={() => toggleSection('voice')} className="section-header">
                {renderChevron('voice')} Voice
              </h3>
              {expandedSections.voice && (
                <div className="section-content">
                  <div className="setting-item">
                    <label htmlFor="voice-selected">Selected Voice</label>
                    <div className="voice-select-wrapper">
                      <select
                        id="voice-selected"
                        className="voice-select"
                        value={getSetting('voice', 'selectedVoice', '')}
                        onChange={(e) => handleChange('voice', 'selectedVoice', e.target.value)}
                      >
                        <option value="">Auto-select by language</option>
                        
                        {availableVoices.length > 0 && (
                          <>
                            {/* US English Voices */}
                            <optgroup label="American English (en-US)">
                              {availableVoices
                                .filter(voice => voice.lang === 'en-US')
                                .map((voice, index) => (
                                  <option key={`en-us-${index}`} value={voice.name}>
                                    {formatVoiceOption(voice)}
                                  </option>
                                ))
                              }
                            </optgroup>
                            
                            {/* Other English Voices */}
                            <optgroup label="Other English Variants">
                              {availableVoices
                                .filter(voice => voice.lang.startsWith('en-') && voice.lang !== 'en-US')
                                .map((voice, index) => (
                                  <option key={`en-other-${index}`} value={voice.name}>
                                    {formatVoiceOption(voice)}
                                  </option>
                                ))
                              }
                            </optgroup>
                            
                            {/* All Other Voices */}
                            <optgroup label="Other Languages">
                              {availableVoices
                                .filter(voice => !voice.lang.startsWith('en-'))
                                .map((voice, index) => (
                                  <option key={`other-${index}`} value={voice.name}>
                                    {formatVoiceOption(voice)}
                                  </option>
                                ))
                              }
                            </optgroup>
                          </>
                        )}
                      </select>
                    </div>
                    <button className="test-voice-button" onClick={testVoice}>
                      Test
                    </button>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="voice-volume">Volume</label>
                    <input
                      id="voice-volume"
                      type="range"
                      min="0.0"
                      max="1.0"
                      step="0.01"
                      value={getSetting('voice', 'volume', 1.0)}
                      onChange={(e) => handleSliderChange('voice', 'volume', e.target.value, 0.0, 1.0)}
                    />
                    <span className="setting-value">{getSetting('voice', 'volume', 1.0).toFixed(2)}</span>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="voice-rate">Rate</label>
                    <input
                      id="voice-rate"
                      type="range"
                      min="0.5"
                      max="2.0"
                      step="0.01"
                      value={getSetting('voice', 'rate', 1.0)}
                      onChange={(e) => handleSliderChange('voice', 'rate', e.target.value, 0.5, 2.0)}
                    />
                    <span className="setting-value">{getSetting('voice', 'rate', 1.0).toFixed(2)}x</span>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="voice-pitch">Pitch</label>
                    <input
                      id="voice-pitch"
                      type="range"
                      min="0.5"
                      max="2.0"
                      step="0.01"
                      value={getSetting('voice', 'pitch', 1.0)}
                      onChange={(e) => handleSliderChange('voice', 'pitch', e.target.value, 0.5, 2.0)}
                    />
                    <span className="setting-value">{getSetting('voice', 'pitch', 1.0).toFixed(2)}x</span>
                  </div>
                  <div className="setting-item">
                    <label htmlFor="voice-default-language">Default Language</label>
                    <input
                      id="voice-default-language"
                      type="text"
                      value={getSetting('voice', 'defaultLanguage', 'en-US')}
                      onChange={(e) => handleChange('voice', 'defaultLanguage', e.target.value)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="voice-enabled">Enabled</label>
                    <input
                      id="voice-enabled"
                      type="checkbox"
                      checked={getSetting('voice', 'enabled', true)}
                      onChange={(e) => handleChange('voice', 'enabled', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="voice-announce-status">Announce Status</label>
                    <input
                      id="voice-announce-status"
                      type="checkbox"
                      checked={getSetting('voice', 'announceStatus', true)}
                      onChange={(e) => handleChange('voice', 'announceStatus', e.target.checked)}
                    />
                  </div>
                  <div className="setting-item">
                    <label htmlFor="voice-announce-errors">Announce Errors</label>
                    <input
                      id="voice-announce-errors"
                      type="checkbox"
                      checked={getSetting('voice', 'announceErrors', true)}
                      onChange={(e) => handleChange('voice', 'announceErrors', e.target.checked)}
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