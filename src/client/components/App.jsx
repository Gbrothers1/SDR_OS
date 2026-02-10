import React, { useEffect, useState, useCallback, useRef } from 'react';
import SplashScreen from './SplashScreen';
import ViewerLayer from './ViewerLayer';
import TrustStrip from './TrustStrip';
import EdgePanel from './EdgePanel';
import CommandBar from './CommandBar';
import ControlOverlay from './ControlOverlay';
import TelemetryConsole from './TelemetryConsole';
import PolicyBrowserPanel from './PolicyBrowserPanel';
import BlendPanel from './BlendPanel';
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

class ErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null };
  }
  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }
  componentDidCatch(error, info) {
    console.error('[ErrorBoundary]', error, info.componentStack);
  }
  render() {
    if (this.state.hasError) {
      return (
        <div style={{
          position: 'fixed', inset: 0,
          background: '#0f1117', color: '#e2e4ea',
          fontFamily: 'monospace', padding: 40,
          display: 'flex', flexDirection: 'column', gap: 16,
        }}>
          <h2 style={{ color: '#ef5350', margin: 0 }}>SDR_OS — Render Error</h2>
          <pre style={{ whiteSpace: 'pre-wrap', fontSize: 13, opacity: 0.8 }}>
            {String(this.state.error)}
          </pre>
          <button
            onClick={() => window.location.reload()}
            style={{
              alignSelf: 'flex-start',
              padding: '8px 24px', cursor: 'pointer',
              background: 'transparent', border: '1px solid #5b8def',
              color: '#5b8def', borderRadius: 4, fontFamily: 'monospace',
            }}
          >
            Reload
          </button>
        </div>
      );
    }
    return this.props.children;
  }
}

const CockpitContent = ({ socket }) => {
  const [isLoading, setIsLoading] = useState(true);
  const [ros, setRos] = useState(null);
  const [rosConnected, setRosConnected] = useState(false);
  const [rosConnecting, setRosConnecting] = useState(false);
  const [error, setError] = useState(null);
  const [appSettings, setAppSettings] = useState(null);
  const rosReconnectTimerRef = useRef(null);
  const { genesisConnected, genesisMode: currentGenesisMode, sendGamepadAxes, sendCommand, sendVelocityCommand, currentFrame, mediaStream, streamCodec } = useGenesis();

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

  // Local mode tracking for test mode (no Genesis to store mode server-side)
  const [testModeView, setTestModeView] = useState('teleop');

  // ── Viewer source selection (shared between ViewerLayer and TrustStrip) ──
  // null = auto (genesis > ros > idle), 'genesis' or 'ros' = user override
  const [viewerOverride, setViewerOverride] = useState(null);

  // Compute what's actually showing
  const hasGenesisStream = genesisConnected && (currentFrame || mediaStream || streamCodec === 'h264');
  const hasRos = rosConnected;
  let effectiveViewer = 'idle';
  if (viewerOverride) {
    effectiveViewer = viewerOverride;
  } else if (hasGenesisStream) {
    effectiveViewer = 'genesis';
  } else if (hasRos) {
    effectiveViewer = 'ros';
  } else if (testMode) {
    effectiveViewer = 'ros';
  }

  // Can toggle when both sources available, or in test mode (has 3D demo + sim placeholder)
  const canToggleViewer = (hasGenesisStream && hasRos) || testMode;

  const handleViewerToggle = useCallback(() => {
    setViewerOverride(prev => {
      if (prev === 'genesis') return 'ros';
      if (prev === 'ros') return 'genesis';
      // Auto mode: flip from whatever's currently effective
      if (hasGenesisStream) return 'ros';
      return 'genesis';
    });
  }, [hasGenesisStream]);

  // Derive which panel to show in the right edge panel
  const activeMode = testMode ? testModeView : currentGenesisMode;
  const isGenesisOrTest = genesisConnected || testMode;
  const showPolicyPanel = isGenesisOrTest &&
    (activeMode === 'eval' || activeMode === 'policy');
  const showBlendPanel = isGenesisOrTest &&
    (activeMode === 'hil_blend' || activeMode === 'blend' || activeMode === 'online_finetune');

  // Edge panel state
  const [leftOpen, setLeftOpen] = useState(false);
  const [leftPinned, setLeftPinned] = useState(false);
  const [rightOpen, setRightOpen] = useState(false);
  const [rightPinned, setRightPinned] = useState(false);
  const [bottomOpen, setBottomOpen] = useState(false);
  const [settingsOpen, setSettingsOpen] = useState(false);
  const [leftExpanded, setLeftExpanded] = useState(false);
  const [rightExpanded, setRightExpanded] = useState(false);

  // Handle mode change from TrustStrip — open right panel for all modes
  const handleModeChange = useCallback((mode) => {
    if (testMode) setTestModeView(mode);
    setRightOpen(true);
  }, [testMode]);

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
        <ViewerLayer ros={ros} appSettings={appSettings} testMode={testMode} viewerOverride={viewerOverride} onViewerChange={setViewerOverride} />

        {/* Layer 2: Trust strip */}
        <TrustStrip
          onStageClick={handleStageClick}
          testMode={testMode}
          onModeChange={handleModeChange}
          effectiveViewer={effectiveViewer}
          canToggleViewer={canToggleViewer}
          onViewerToggle={handleViewerToggle}
        />

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
            sendGamepadAxes={sendGamepadAxes}
            sendCommand={sendCommand}
            sendVelocityCommand={sendVelocityCommand}
          />
        </EdgePanel>

        {/* Overflow panel — slides out from behind the left edge panel */}
        <div
          className={`overflow-panel overflow-panel--left${leftOpen && leftExpanded ? ' overflow-panel--open' : ''}`}
          id="overflow-panel-left"
        />

        {/* Right: Telemetry / Policy Browser / Blend Panel */}
        <EdgePanel
          side="right"
          isOpen={rightOpen}
          onOpen={() => setRightOpen(true)}
          onClose={() => { if (!rightPinned) setRightOpen(false); }}
          pinned={rightPinned}
          onTogglePin={() => setRightPinned(prev => !prev)}
          title={showPolicyPanel ? 'Policy' : showBlendPanel ? 'Blend' : 'Telemetry'}
        >
          {showPolicyPanel ? (
            <PolicyBrowserPanel onExpandChange={setRightExpanded} />
          ) : showBlendPanel ? (
            <BlendPanel onExpandChange={setRightExpanded} />
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
    console.log('[socket] connecting', socketUrl);
    newSocket.on('connect', () => console.log('[socket] connected', newSocket.id));
    newSocket.on('connect_error', (err) => console.warn('[socket] connect_error', err?.message || err));
    newSocket.on('disconnect', (reason) => console.warn('[socket] disconnected', reason));
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
  <ErrorBoundary>
    <SettingsProvider>
      <AppWithGenesis />
    </SettingsProvider>
  </ErrorBoundary>
);

export default App;
