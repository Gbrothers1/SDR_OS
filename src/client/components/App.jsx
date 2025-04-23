import React, { useEffect, useState, useCallback } from 'react';
import RobotViewer from './RobotViewer';
import ControlOverlay from './ControlOverlay';
import LogViewer from './LogViewer';
import SettingsIcon from './SettingsIcon';
import CameraIcon from './CameraIcon';
import SettingsModal from './SettingsModal';
import TelemetryPanel from './TelemetryPanel';
import SplashScreen from './SplashScreen';
import CameraViewer from './CameraViewer';
import { SettingsProvider, useSettings } from '../contexts/SettingsContext';
import ROSLIB from 'roslib';
import io from 'socket.io-client';
import '../styles/App.css';

const AppContent = () => {
  const [isLoading, setIsLoading] = useState(true);
  const [ros, setRos] = useState(null);
  const [socket, setSocket] = useState(null);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState(null);
  const [isSettingsOpen, setIsSettingsOpen] = useState(false);
  const [isLogViewerVisible, setIsLogViewerVisible] = useState(false);
  const [displaySettings, setDisplaySettings] = useState({
    showLogViewer: false,
    showTelemetryPanel: true,
    showCameraFeed: false,
  });
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });
  
  // Track if this client has a gamepad connected
  const [hasGamepad, setHasGamepad] = useState(false);

  // Store the full settings object in state to easily pass down parts
  const [appSettings, setAppSettings] = useState(null);

  const [isCameraVisible, setIsCameraVisible] = useState(false);

  const { getSetting, updateSettings } = useSettings();
  const [isControllerVisible, setIsControllerVisible] = useState(
    getSetting('ui', 'showControllerOnStartup', true)
  );
  
  // Monitor settings changes for controller visibility
  useEffect(() => {
    // Function to sync controller visibility with settings
    const syncControllerVisibility = () => {
      // For runtime control we use controllerVisible, not the startup value
      const currentSetting = getSetting('ui', 'controllerVisible', true);
      console.log('AppContent: Syncing controller visibility from settings:', currentSetting);
      setIsControllerVisible(currentSetting);
    };
    
    // Call immediately and whenever getSetting might change (dependency)
    syncControllerVisibility();
    
    // This will execute whenever the settings context changes the getSetting function
  }, [getSetting]);

  const initializeConnections = async () => {
    try {
      const rosBridgeUrl = localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090';
      console.log('Attempting ROS connection to:', rosBridgeUrl);

      const newRos = new ROSLIB.Ros({
        url: rosBridgeUrl
      });

      let connectionResolved = false;
      
      newRos.on('connection', () => {
        console.log('Connected to ROS bridge');
        setConnected(true);
        setError(null);
        connectionResolved = true;
      });

      newRos.on('error', (error) => {
        console.error('ROS connection error:', error);
        if (!connectionResolved) {
          connectionResolved = true;
          throw new Error('Failed to connect to ROS bridge server');
        }
      });

      newRos.on('close', () => {
        console.log('ROS connection closed');
        setConnected(false);
      });

      await Promise.race([
        new Promise(resolve => setTimeout(() => {
          if (!connectionResolved) {
            connectionResolved = true;
            throw new Error('ROS connection timeout');
          }
        }, 5000)),
        new Promise(resolve => {
          const checkConnection = setInterval(() => {
            if (connectionResolved) {
              clearInterval(checkConnection);
              resolve();
            }
          }, 100);
        })
      ]);

      setRos(newRos);

      const socketUrl = localStorage.getItem('socketUrl') || 'http://localhost:3000';
      console.log('Connecting to Socket.IO:', socketUrl);
      const newSocket = io(socketUrl);
      setSocket(newSocket);
      
      // Listen for gamepad connection events globally
      window.addEventListener('gamepadconnected', () => {
        console.log('Gamepad connected to this client');
        setHasGamepad(true);
      });
      
      window.addEventListener('gamepaddisconnected', () => {
        console.log('Gamepad disconnected from this client');
        setHasGamepad(false);
      });

      return true;
    } catch (error) {
      console.error('Connection initialization error:', error);
      setError(`Failed to initialize: ${error.message}`);
      return false;
    }
  };

  const toggleLogViewer = () => {
    setIsLogViewerVisible(prev => !prev);
    setDisplaySettings(prev => ({
      ...prev,
      showLogViewer: !prev.showLogViewer
    }));
  };

  const toggleCamera = () => {
    // Toggle camera visibility
    const newCameraVisible = !isCameraVisible;
    setIsCameraVisible(newCameraVisible);
    
    // Update display settings for camera
    setDisplaySettings(prev => ({
      ...prev,
      showCameraFeed: newCameraVisible
    }));
    
    // Store the settings in localStorage
    const updatedSettings = {
      ...displaySettings,
      showCameraFeed: newCameraVisible
    };
    localStorage.setItem('displaySettings', JSON.stringify(updatedSettings));
  };

  const handleSplashComplete = async () => {
    console.log('Splash screen complete, initializing connections...');
    const success = await initializeConnections();
    if (success) {
      console.log('Connections initialized successfully');
      setIsLoading(false);
    } else {
      console.error('Failed to initialize connections');
      setError('Failed to initialize system connections');
    }
  };

  // Memoize handleControlChange to stabilize the prop passed to ControlOverlay
  const handleControlChange = useCallback((newState) => {
    // Update local control state
    setControlState(newState);
  }, []); // No dependencies, function doesn't rely on changing state/props

  const handleSettingsSave = (newSettings) => {
    console.log("Saving settings:", newSettings);
    
    // Update display settings state (including camera)
    const newDisplaySettings = {
      showLogViewer: newSettings.ui?.showLogsOnStartup ?? false,
      showTelemetryPanel: newSettings.ui?.showTelemetryPanel ?? true,
      showCameraFeed: newSettings.ui?.showCameraOnStartup ?? false,
    };
    
    setDisplaySettings(newDisplaySettings);
    
    // Handle controller visibility settings
    const uiSettings = newSettings.ui || {};
    
    // Handle showControllerOnStartup specifically
    if (uiSettings.showControllerOnStartup !== undefined) {
      console.log('Updating showControllerOnStartup setting:', uiSettings.showControllerOnStartup);
      
      // If only showControllerOnStartup was set (typically from the settings screen),
      // we don't want to immediately change visibility - that should only happen on next startup
      // However, we need to make sure controllerVisible is set properly for the next save
      if (uiSettings.controllerVisible === undefined) {
        // Keep the same value for controllerVisible that we already have
        uiSettings.controllerVisible = getSetting('ui', 'controllerVisible', true);
      }
    }
    
    // If controllerVisible was changed, update the isControllerVisible state
    if (uiSettings.controllerVisible !== undefined) {
      console.log('Setting isControllerVisible from settings save:', uiSettings.controllerVisible);
      setIsControllerVisible(uiSettings.controllerVisible);
    }
    
    // Update full settings state
    setAppSettings(newSettings);
    
    // Save display settings (for initial load)
    localStorage.setItem('displaySettings', JSON.stringify(newDisplaySettings));
    
    // Save the full settings object
    localStorage.setItem('robotControllerSettings', JSON.stringify(newSettings));
  };

  const toggleSettings = useCallback(() => {
    setIsSettingsOpen(prev => !prev);
    console.log("Settings toggled:", !isSettingsOpen);
  }, [isSettingsOpen]);

  const closeSettings = useCallback(() => {
    setIsSettingsOpen(false);
    console.log("Settings closed");
  }, []);

  const toggleController = useCallback(() => {
    const newValue = !isControllerVisible;
    console.log('App: Toggling controller visibility to:', newValue);
    
    // Update local state
    setIsControllerVisible(newValue);
    
    // Update settings context
    updateSettings({
      ui: {
        controllerVisible: newValue
      }
    });
  }, [isControllerVisible, updateSettings]);

  useEffect(() => {
    // Load full settings on initial mount
    const savedFullSettings = localStorage.getItem('robotControllerSettings');
    if (savedFullSettings) {
      try {
        const parsedSettings = JSON.parse(savedFullSettings);
        console.log('Loaded settings from localStorage:', parsedSettings);
        
        // Handle potential legacy settings format
        if (!parsedSettings.ui) {
          // Migrate legacy settings to new format
          parsedSettings.ui = {
            showControllerOnStartup: parsedSettings.control?.showControllerOnStartup ?? true,
            controllerVisible: parsedSettings.control?.showControllerOnStartup ?? true,
            controllerMinimized: false, // Default to not minimized
            showTelemetryPanel: parsedSettings.telemetry?.showTelemetryPanel ?? true,
            showLogsOnStartup: false,
            showCameraOnStartup: parsedSettings.visualization?.showCameraFeed ?? false
          };
          
          // Remove legacy properties
          if (parsedSettings.control) {
            delete parsedSettings.control.showControllerOnStartup;
          }
          if (parsedSettings.telemetry) {
            delete parsedSettings.telemetry.showTelemetryPanel;
          }
          if (parsedSettings.visualization) {
            delete parsedSettings.visualization.showCameraFeed;
          }
          
          console.log('Migrated legacy settings to new format:', parsedSettings);
          
          // Save back the migrated settings
          localStorage.setItem('robotControllerSettings', JSON.stringify(parsedSettings));
        } else {
          // Make sure all UI properties exist
          if (!parsedSettings.ui.hasOwnProperty('controllerMinimized')) {
            parsedSettings.ui.controllerMinimized = false;
          }
          
          // Make sure controllerVisible has a value - this is separate from showControllerOnStartup
          if (parsedSettings.ui.controllerVisible === undefined) {
            // On fresh reload, controllerVisible should be initialized from showControllerOnStartup
            parsedSettings.ui.controllerVisible = parsedSettings.ui.showControllerOnStartup;
          }
        }
        
        setAppSettings(parsedSettings);
        
        // Initialize UI state from the settings
        // On a fresh load, controllerVisible is initialized from showControllerOnStartup
        if (parsedSettings.ui) {
          console.log('Setting UI state from loaded settings:', parsedSettings.ui);
          
          // On initial app load, we respect the showControllerOnStartup setting
          // This ensures the controller appears (or not) based on startup preference
          const shouldShowOnStartup = parsedSettings.ui.showControllerOnStartup;
          console.log('Controller should show on startup:', shouldShowOnStartup);
          
          // Initialize the controllerVisible setting based on the startup setting
          parsedSettings.ui.controllerVisible = shouldShowOnStartup;
          
          // Also set the local React state
          setIsControllerVisible(shouldShowOnStartup);
          setIsLogViewerVisible(parsedSettings.ui.showLogsOnStartup);
          setIsCameraVisible(parsedSettings.ui.showCameraOnStartup);
          
          setDisplaySettings(prev => ({
            ...prev,
            showLogViewer: parsedSettings.ui.showLogsOnStartup,
            showTelemetryPanel: parsedSettings.ui.showTelemetryPanel,
            showCameraFeed: parsedSettings.ui.showCameraOnStartup
          }));
          
          // Make sure updated settings with initialized controllerVisible are saved
          localStorage.setItem('robotControllerSettings', JSON.stringify(parsedSettings));
        }
      } catch (e) {
        console.error('Error loading full settings:', e);
      }
    }
    
    // Legacy display settings
    const savedDisplaySettings = localStorage.getItem('displaySettings');
    if (savedDisplaySettings) {
      try {
        const loadedDisplay = JSON.parse(savedDisplaySettings);
        // Only use these if we didn't already set from robotControllerSettings
        if (!savedFullSettings || !JSON.parse(savedFullSettings).ui) {
          setDisplaySettings(prev => ({ 
            ...prev, // Start with defaults
            ...loadedDisplay // Override with loaded values
          }));
          setIsLogViewerVisible(loadedDisplay.showLogViewer);
          setIsCameraVisible(loadedDisplay.showCameraFeed || false);
        }
      } catch (e) {
        console.error('Error loading display settings:', e);
      }
    }
    
    // Check if a gamepad is already connected
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    if (gamepads.some(gamepad => gamepad !== null)) {
      setHasGamepad(true);
    }
    
    return () => {
      window.removeEventListener('gamepadconnected', () => setHasGamepad(true));
      window.removeEventListener('gamepaddisconnected', () => setHasGamepad(false));
    };
  }, []);

  // Store display settings (but throttle the updates)
  useEffect(() => {
    const debounceTimer = setTimeout(() => {
      localStorage.setItem('displaySettings', JSON.stringify(displaySettings));
    }, 1000); // Throttle to once every second
    
    return () => clearTimeout(debounceTimer);
  }, [displaySettings]);

  // Initialize display settings from the UI settings during component mount
  useEffect(() => {
    if (appSettings?.ui) {
      setIsControllerVisible(appSettings.ui.controllerVisible);
      setIsLogViewerVisible(appSettings.ui.showLogsOnStartup);
      setIsCameraVisible(appSettings.ui.showCameraOnStartup);
      
      setDisplaySettings(prev => ({
        ...prev,
        showLogViewer: appSettings.ui.showLogsOnStartup,
        showTelemetryPanel: appSettings.ui.showTelemetryPanel,
        showCameraFeed: appSettings.ui.showCameraOnStartup
      }));
    }
  }, [appSettings]);

  if (isLoading) {
    return <SplashScreen onComplete={handleSplashComplete} />;
  }

  return (
    <div className="app">
      {error && (
        <div className="error-banner">
          {error}
          <button onClick={() => window.location.reload()}>
            Retry Connection
          </button>
        </div>
      )}
      
      <div className="app-container">
        <div className="viewer-container">
          <RobotViewer 
            ros={ros} 
            tfThrottleRate={appSettings?.visualization?.tfThrottleRate ?? 10}
            settings={appSettings}
          />
          {isControllerVisible && (
            <ControlOverlay 
              ros={ros} 
              socket={socket} 
              controlState={controlState}
              onControlChange={handleControlChange}
            />
          )}
          <TelemetryPanel 
            ros={ros} 
            // Pass updateInterval from settings state
            updateInterval={appSettings?.telemetry?.updateInterval ?? 100} // Default 100ms
            initialShowPanel={displaySettings.showTelemetryPanel}
          />
          {displaySettings.showCameraFeed && 
            <CameraViewer 
              ros={ros} 
              topic={'/webcam/image_raw'}
              host={appSettings?.webVideoHost || localStorage.getItem('webVideoHost') || 'localhost'}
            />
          }
          
          <div className="toggle-buttons">
            <button className="toggle-log" onClick={toggleLogViewer}>
              {isLogViewerVisible ? 'Hide Logs' : 'Show Logs'}
            </button>
          </div>
        </div>
        
        {isLogViewerVisible && (
          <LogViewer 
            ros={ros} 
            // Pass subscription settings from state (or default)
            subscribeRosout={appSettings?.logging?.subscribeRosout ?? true}
            subscribeDiagnostics={appSettings?.logging?.subscribeDiagnostics ?? true}
          />
        )}
      </div>
      
      <div className="toolbar">
        <SettingsIcon onClick={toggleSettings} />
        <CameraIcon onClick={toggleCamera} isActive={isCameraVisible} />
      </div>
      
      <SettingsModal
        isOpen={isSettingsOpen}
        onClose={closeSettings}
        onSave={handleSettingsSave}
        initialSettings={appSettings}
      />
    </div>
  );
};

const App = () => {
  return (
    <SettingsProvider>
      <AppContent />
    </SettingsProvider>
  );
};

export default App; 