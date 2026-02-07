import React, { createContext, useContext, useState, useCallback } from 'react';

// Default settings structure
const defaultSettings = {
  ui: {
    // These settings control the controller overlay's visibility:
    showControllerOnStartup: true,  // Whether to show controller when app first loads (affects next startup)
    controllerVisible: true,        // Current visibility state (can be toggled at runtime)
    controllerMinimized: false,     // Whether the controller is minimized (hidden with CSS but still in DOM)
    
    // Other UI settings:
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
    triggerSensitivity: 1.0,
    gamepadPreset: 'balanced',
    gamepadJoystickHz: 30,
    gamepadButtonHz: 10,
    gamepadUiHz: 60,
    gamepadAxisEpsilon: 0.01,
    gamepadUseVolatile: true,
    gamepadDisableCompression: true
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
    camera: '/webcam/image_raw',
    telemetry_all: '/robot/telemetry/all'
  },
  connection: {
    rosBridgeUrl: `ws://${window.location.hostname}:9090`,
    socketUrl: window.location.origin,
    webVideoHost: window.location.hostname
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
  },
  genesis: {
    enabled: true,
    streamBackend: 'websocket',
    bridgeWsUrl: '',
    webrtcStunUrls: ['stun:stun.l.google.com:19302'],
    webrtcAutoUpgrade: false,
    showStats: false,
    defaultMode: 'Real Robot',
    activeRobot: '',
    activeEnvIdx: 0,
    streamFps: 60,
    jpegQuality: 80,
    cameraRes: '1280x720',
    trainingMetricsRate: 5,
    showSimOverlay: true,
    autoConnectBridge: true,
    showPip: true
  }
};

const mergeSettings = (base, updates) => {
  const merged = { ...base };
  for (const section of Object.keys(base)) {
    if (updates && updates[section]) {
      merged[section] = { ...base[section], ...updates[section] };
    }
  }

  // Legacy flat connection keys
  if (updates?.rosBridgeUrl) merged.connection = { ...(merged.connection || {}), rosBridgeUrl: updates.rosBridgeUrl };
  if (updates?.socketUrl) merged.connection = { ...(merged.connection || {}), socketUrl: updates.socketUrl };
  if (updates?.webVideoHost) merged.connection = { ...(merged.connection || {}), webVideoHost: updates.webVideoHost };

  return merged;
};

const applyLegacyConnection = (settings) => {
  const legacy = {};
  const rosBridgeUrl = localStorage.getItem('rosBridgeUrl');
  const socketUrl = localStorage.getItem('socketUrl');
  const webVideoHost = localStorage.getItem('webVideoHost');

  if (rosBridgeUrl) legacy.rosBridgeUrl = rosBridgeUrl;
  if (socketUrl) legacy.socketUrl = socketUrl;
  if (webVideoHost) legacy.webVideoHost = webVideoHost;

  if (Object.keys(legacy).length > 0) {
    return {
      ...settings,
      connection: {
        ...settings.connection,
        ...legacy
      }
    };
  }

  return settings;
};

const SettingsContext = createContext();

export const SettingsProvider = ({ children }) => {
  const [settings, setSettings] = useState(() => {
    // Try to load settings from localStorage
    const savedSettings = localStorage.getItem('robotControllerSettings');
    if (savedSettings) {
      try {
        const parsed = JSON.parse(savedSettings);
        const merged = mergeSettings(defaultSettings, parsed);
        return applyLegacyConnection(merged);
      } catch (error) {
        console.error('Failed to parse saved settings:', error);
        return defaultSettings;
      }
    }
    return applyLegacyConnection(defaultSettings);
  });

  // Save settings to localStorage whenever they change
  React.useEffect(() => {
    localStorage.setItem('robotControllerSettings', JSON.stringify(settings));
  }, [settings]);

  const getSetting = useCallback((section, key, defaultValue = undefined) => {
    return settings?.[section]?.[key] ?? defaultValue;
  }, [settings]);

  const updateSettings = useCallback((newSettings) => {
    setSettings(prev => {
      const updated = { ...prev };
      for (const [section, values] of Object.entries(newSettings)) {
        if (updated[section]) {
          updated[section] = { ...updated[section], ...values };
        } else {
          updated[section] = values;
        }
      }
      return updated;
    });
  }, []);

  const resetSettings = useCallback(() => {
    setSettings(defaultSettings);
  }, []);

  const value = {
    settings,
    getSetting,
    updateSettings,
    resetSettings,
  };

  return (
    <SettingsContext.Provider value={value}>
      {children}
    </SettingsContext.Provider>
  );
};

export const useSettings = () => {
  const context = useContext(SettingsContext);
  if (!context) {
    throw new Error('useSettings must be used within a SettingsProvider');
  }
  return context;
};

export default SettingsContext; 
