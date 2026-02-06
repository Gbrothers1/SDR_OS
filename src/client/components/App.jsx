import React, { useEffect, useState, useCallback, useRef } from 'react';
import SplashScreen from './SplashScreen';
import ViewerLayer from './ViewerLayer';
import TrustStrip from './TrustStrip';
import EdgePanel from './EdgePanel';
import CommandBar from './CommandBar';
import ControlOverlay from './ControlOverlay';
import TelemetryConsole from './TelemetryConsole';
import PolicyBrowserPanel from './PolicyBrowserPanel';
import TrainingPanel from './TrainingPanel';
import LogViewer from './LogViewer';
import SettingsModal from './SettingsModal';
import { SettingsProvider, useSettings } from '../contexts/SettingsContext';
import { GenesisProvider, useGenesis } from '../contexts/GenesisContext';
import { PhaseProvider } from '../contexts/PhaseContext';
import { TrainingPanelProvider, useTrainingPanel } from '../contexts/TrainingPanelContext';
import { SessionProvider } from '../contexts/SessionContext';
import ROSLIB from 'roslib';
import io from 'socket.io-client';
import '../styles/App.css';

const CockpitContent = ({ socket }) => {
  const [isLoading, setIsLoading] = useState(true);
  const [ros, setRos] = useState(null);
  const [rosConnected, setRosConnected] = useState(false);
  const [rosConnecting, setRosConnecting] = useState(false);
  const [error, setError] = useState(null);
  const [appSettings, setAppSettings] = useState(null);
  const rosReconnectTimerRef = useRef(null);
  const { genesisConnected, genesisMode: currentGenesisMode } = useGenesis();

  // ── Test mode: bypass ROS/Genesis connections for UI development ──
  const [testMode, setTestMode] = useState(() => {
    return localStorage.getItem('sdr_test_mode') === 'true';
  });

  const enableTestMode = useCallback(() => {
    setTestMode(true);
    localStorage.setItem('sdr_test_mode', 'true');
    setError(null);
    // Stop ROS reconnect loop
    if (rosReconnectTimerRef.current) {
      clearTimeout(rosReconnectTimerRef.current);
      rosReconnectTimerRef.current = null;
    }
  }, []);

  const disableTestMode = useCallback(() => {
    setTestMode(false);
    localStorage.removeItem('sdr_test_mode');
    // Restart ROS connection
    connectRos();
  }, []);

  // Derive whether policy browser should replace telemetry in right panel
  const showPolicyPanel = genesisConnected &&
    (currentGenesisMode === 'eval' || currentGenesisMode === 'policy');

  // Edge panel state
  const [leftOpen, setLeftOpen] = useState(false);
  const [leftPinned, setLeftPinned] = useState(false);
  const [rightOpen, setRightOpen] = useState(false);
  const [rightPinned, setRightPinned] = useState(false);
  const [bottomOpen, setBottomOpen] = useState(false);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [leftExpanded, setLeftExpanded] = useState(false);
  const [rightExpanded, setRightExpanded] = useState(false);

  // Control state for ControlOverlay
  const [controlState, setControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });

  const { getSetting, updateSettings, settings } = useSettings();
  const handleControlChange = useCallback((newState) => {
    setControlState(newState);
  }, []);

  // Initialize connections with auto-reconnect for ROS
  const connectRos = useCallback((attempt = 0) => {
    // Skip ROS connection in test mode
    if (localStorage.getItem('sdr_test_mode') === 'true') {
      setRosConnecting(false);
      return;
    }
    const rosBridgeUrl = getSetting('connection', 'rosBridgeUrl', `ws://${window.location.hostname}:9090`);
    setRosConnecting(true);
    const newRos = new ROSLIB.Ros({ url: rosBridgeUrl });
    let resolved = false;

    const clearTimer = () => {
      if (rosReconnectTimerRef.current) {
        clearTimeout(rosReconnectTimerRef.current);
        rosReconnectTimerRef.current = null;
      }
    };

    const scheduleReconnect = () => {
      clearTimer();
      const delay = Math.min(5000 + attempt * 1000, 15000); // backoff up to 15s
      rosReconnectTimerRef.current = setTimeout(() => {
        rosReconnectTimerRef.current = null;
        connectRos(attempt + 1);
      }, delay);
    };

    newRos.on('connection', () => {
      resolved = true;
      setRosConnected(true);
      setError(null);
      setRos(newRos);
      setRosConnecting(false);
      clearTimer();
    });

    newRos.on('error', (err) => {
      if (!resolved) resolved = true;
      setRosConnecting(false);
      scheduleReconnect();
      setError(prev => prev || `ROS bridge error: ${err?.message || 'unknown'}`);
    });

    newRos.on('close', () => {
      setRosConnected(false);
      setRosConnecting(false);
      scheduleReconnect();
    });
  }, [getSetting]);

  const handleSplashComplete = async () => {
    connectRos();
    // Load settings
    setAppSettings(settings);
    setIsLoading(false);
  };

  // Cleanup timers and sockets on unmount
  useEffect(() => {
    return () => {
      if (rosReconnectTimerRef.current) {
        clearTimeout(rosReconnectTimerRef.current);
      }
      if (ros && typeof ros.close === 'function') {
        ros.close();
      }
    };
  }, [ros]);

  // Keyboard shortcuts for panels
  useEffect(() => {
    const handleKey = (e) => {
      if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

      // Bottom panel: ` or /
      if (e.key === '`' || e.key === '/') {
        e.preventDefault();
        setBottomOpen(prev => !prev);
      }
      
      // Left panel: Ctrl+1
      if (e.ctrlKey && e.key === '1') {
        e.preventDefault();
        setLeftOpen(prev => !prev);
      }
      
      // Right panel: Ctrl+2
      if (e.ctrlKey && e.key === '2') {
        e.preventDefault();
        setRightOpen(prev => !prev);
      }
    };
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, []);

  // Gamepad panel triggers: L1=leftPanel, R1=rightPanel
  useEffect(() => {
    let animFrame;
    const prevButtons = { 4: false, 5: false, 9: false }; // L1=4, R1=5, Start=9

    const poll = () => {
      // Safari-safe: navigator.getGamepads() may return null before user presses a button
      let gamepads;
      try {
        gamepads = navigator.getGamepads ? navigator.getGamepads() : null;
      } catch (e) {
        gamepads = null;
      }
      
      if (gamepads) {
        const gp = gamepads[0];
        if (gp) {
          const startPressed = gp.buttons[9]?.pressed || false;

          // Panel toggle chord: Start + L1/R1
          if (startPressed) {
            // L1 (button 4)
            if (gp.buttons[4]?.pressed && !prevButtons[4]) {
              setLeftOpen(prev => !prev);
            }

            // R1 (button 5)
            if (gp.buttons[5]?.pressed && !prevButtons[5]) {
              setRightOpen(prev => !prev);
            }
          }

          prevButtons[4] = gp.buttons[4]?.pressed || false;
          prevButtons[5] = gp.buttons[5]?.pressed || false;
          prevButtons[9] = startPressed;
        }
      }
      animFrame = requestAnimationFrame(poll);
    };
    animFrame = requestAnimationFrame(poll);
    return () => cancelAnimationFrame(animFrame);
  }, []);

  // Stage cell click handler — opens relevant edge panel
  const handleStageClick = useCallback((stage) => {
    setRightOpen(true);
  }, []);

  const handleSettingsSave = useCallback((newSettings) => {
    setAppSettings(newSettings);
    updateSettings(newSettings);
    localStorage.setItem('robotControllerSettings', JSON.stringify(newSettings));
  }, [updateSettings]);

  if (isLoading) {
    return <SplashScreen onComplete={handleSplashComplete} />;
  }

  // In test mode, suppress the error banner
  const showError = error && !genesisConnected && !testMode;

  return (
    <PhaseProvider ros={ros} rosConnected={rosConnected}>
      <div className="app">
        {/* Error banner with test mode option */}
        {showError && (
          <div className="app__error-banner">
            {error}
            <button onClick={() => window.location.reload()}>Retry</button>
            <button className="app__test-mode-btn" onClick={enableTestMode}>
              Test Mode
            </button>
          </div>
        )}

        {/* Test mode indicator */}
        {testMode && (
          <div className="app__test-badge" onClick={disableTestMode} title="Click to exit test mode and reconnect">
            TEST MODE
          </div>
        )}

        {/* Layer 1: Fullscreen viewer */}
        <ViewerLayer ros={ros} appSettings={appSettings} testMode={testMode} />

        {/* Layer 2: Trust strip */}
        <TrustStrip onStageClick={handleStageClick} testMode={testMode} />

        {/* Layer 3: Edge panels */}

        {/* Left: Controls */}
        <EdgePanel
          side="left"
          isOpen={leftOpen}
          onOpen={() => setLeftOpen(true)}
          onClose={() => { if (!leftPinned) setLeftOpen(false); }}
          pinned={leftPinned}
          onTogglePin={() => setLeftPinned(prev => !prev)}
          title="Controls"
        >
          <ControlOverlay
            ros={ros}
            socket={socket}
            controlState={controlState}
            onControlChange={handleControlChange}
            onExpandChange={setLeftExpanded}
          />
        </EdgePanel>

        {/* Overflow panel — slides out from behind the left edge panel */}
        <div
          className={`overflow-panel overflow-panel--left${leftOpen && leftExpanded ? ' overflow-panel--open' : ''}`}
          id="overflow-panel-left"
        />

        {/* Right: Details / Policy Browser */}
        <EdgePanel
          side="right"
          isOpen={rightOpen}
          onOpen={() => setRightOpen(true)}
          onClose={() => { if (!rightPinned) setRightOpen(false); }}
          pinned={rightPinned}
          onTogglePin={() => setRightPinned(prev => !prev)}
          title={showPolicyPanel ? 'Policy' : 'Info'}
        >
          {showPolicyPanel ? (
            <PolicyBrowserPanel onExpandChange={setRightExpanded} />
          ) : (
            <TelemetryConsole
              ros={ros}
              rosConnected={rosConnected}
              appSettings={appSettings}
            />
          )}
        </EdgePanel>

        {/* Overflow panel — slides out from behind the right edge panel */}
        <div
          className={`overflow-panel overflow-panel--right${rightOpen && rightExpanded ? ' overflow-panel--open' : ''}`}
          id="overflow-panel-right"
        />

        {/* Bottom: Command bar */}
        <EdgePanel
          side="bottom"
          isOpen={bottomOpen}
          onClose={() => setBottomOpen(false)}
        >
          <CommandBar
            onClose={() => setBottomOpen(false)}
            onOpenSettings={() => { setBottomOpen(false); setSettingsOpen(true); }}
          />
        </EdgePanel>

        {/* Settings modal (keeps existing component) */}
        <SettingsModal
          isOpen={settingsOpen}
          onClose={() => setSettingsOpen(false)}
          onSave={handleSettingsSave}
          initialSettings={appSettings}
        />

        {/* Training Panel Overlay */}
        <TrainingPanel />
      </div>
    </PhaseProvider>
  );
};

const AppWithGenesis = () => {
  const [socket, setSocket] = useState(null);
  const { getSetting } = useSettings();
  const socketUrl = getSetting('connection', 'socketUrl', window.location.origin);

  useEffect(() => {
    const newSocket = io(socketUrl);
    setSocket(newSocket);
    return () => { if (newSocket) newSocket.disconnect(); };
  }, [socketUrl]);

  return (
    <GenesisProvider socket={socket}>
      <SessionProvider>
        <TrainingPanelProvider>
          <CockpitContent socket={socket} />
        </TrainingPanelProvider>
      </SessionProvider>
    </GenesisProvider>
  );
};

const App = () => (
  <SettingsProvider>
    <AppWithGenesis />
  </SettingsProvider>
);

export default App;
