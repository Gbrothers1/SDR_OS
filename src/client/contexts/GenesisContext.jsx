import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from 'react';
import { useSettings } from './SettingsContext';
import { detectH264Support, H264Decoder, MsgType, parseVideoHeader } from '../utils/H264Decoder';

const GenesisContext = createContext();

export const useGenesis = () => {
  const context = useContext(GenesisContext);
  if (!context) {
    throw new Error('useGenesis must be used within a GenesisProvider');
  }
  return context;
};

export const GenesisProvider = ({ children, socket }) => {
  const { getSetting, settings } = useSettings();
  const streamBackend = getSetting('genesis', 'streamBackend', 'websocket');
  const wsProto = typeof window !== 'undefined' && window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const autoWsUrl = typeof window !== 'undefined' ? `${wsProto}//${window.location.host}/stream/ws` : '';
  const bridgeWsUrl = getSetting('genesis', 'bridgeWsUrl', '') || autoWsUrl;
  const webrtcStunUrls = getSetting('genesis', 'webrtcStunUrls', ['stun:stun.l.google.com:19302']) || [];
  const webrtcAutoUpgrade = getSetting('genesis', 'webrtcAutoUpgrade', false);

  // WebSocket connection to Genesis bridge
  const [bridgeWs, setBridgeWs] = useState(null);
  const [currentFrame, setCurrentFrame] = useState(null);
  const [trainingMetrics, setTrainingMetrics] = useState(null);
  const [genesisConnected, setGenesisConnected] = useState(false);
  const [webrtcConnected, setWebrtcConnected] = useState(false);
  const [mediaStream, setMediaStream] = useState(null);
  const [genesisMode, setGenesisMode] = useState('teleop_record');
  const [robotList, setRobotList] = useState([]);
  const [currentRobot, setCurrentRobot] = useState(null);  // Currently loaded robot
  const [envInfo, setEnvInfo] = useState(null);
  
  // Script status tracking
  const [scriptStatus, setScriptStatus] = useState('idle');
  const [currentScriptName, setCurrentScriptName] = useState(null);
  const [scriptError, setScriptError] = useState(null);
  
  // Frame statistics
  const [frameStats, setFrameStats] = useState(null);
  
  // Memory estimates (VRAM/DRAM)
  const [memoryEstimate, setMemoryEstimate] = useState(null);

  // Policy browser state
  const [policyList, setPolicyList] = useState([]);
  const [policyLoadStatus, setPolicyLoadStatus] = useState(null);
  const [policyLoadError, setPolicyLoadError] = useState(null);

  // Structured telemetry from genesis-forge
  const [obsBreakdown, setObsBreakdown] = useState(null);
  const [rewardBreakdown, setRewardBreakdown] = useState(null);
  const [velocityCommand, setVelocityCommand] = useState(null);
  
  // H.264 codec state
  const [streamCodec, setStreamCodec] = useState('jpeg');
  const [h264Codec, setH264Codec] = useState(null);
  const [h264Support, setH264Support] = useState(null);
  const h264DecoderRef = useRef(null);

  // Stream stats tracking
  const [streamStats, setStreamStats] = useState({
    frameId: 0, frameSeq: 0, lastKeyframeTime: null,
    wsBytesPerSec: 0, mode: 'ws',
  });
  const wsBytesAccRef = useRef(0);
  const wsStatsIntervalRef = useRef(null);
  const lastFrameSeqRef = useRef(-1);

  // Signaling session state
  const signalingSessionRef = useRef(null);
  const bridgeWsRef = useRef(null);

  // DataChannel refs
  const controlDcRef = useRef(null);
  const commandsDcRef = useRef(null);

  // Track Blob URLs for cleanup
  const blobUrlsRef = useRef(new Set());
  const webrtcPcRef = useRef(null);
  const webrtcReconnectTimerRef = useRef(null);

  const currentFrameRef = useRef(null);

  // Detect H.264 support on mount
  useEffect(() => {
    detectH264Support().then(result => {
      setH264Support(result);
      if (result.supported) {
        console.log('WebCodecs H.264 supported:', result.config.codec);
      } else {
        console.log('WebCodecs H.264 not supported, will use JPEG fallback');
      }
    });
  }, []);

  // Track if we've sent negotiate message for current connection
  const negotiateSentRef = useRef(false);

  // Connect to transport server WebSocket (always — WS carries video, telemetry, and signaling)
  useEffect(() => {
    if (!bridgeWsUrl) return;

    let ws = null;
    let reconnectTimer = null;
    let awaitingKeyframe = true;

    // WS bandwidth tracking interval
    const statsInterval = setInterval(() => {
      setStreamStats(prev => ({
        ...prev,
        wsBytesPerSec: wsBytesAccRef.current,
      }));
      wsBytesAccRef.current = 0;
    }, 1000);
    wsStatsIntervalRef.current = statsInterval;

    const handleVideoFrame = (buffer) => {
      const view = new DataView(buffer);
      const { frameId, frameSeq, isKeyframe, payload } = parseVideoHeader(view);

      // Detect sequence discontinuity → reset decoder
      if (lastFrameSeqRef.current >= 0 && frameSeq > lastFrameSeqRef.current + 1) {
        console.warn(`Frame seq gap: ${lastFrameSeqRef.current} → ${frameSeq}`);
        if (h264DecoderRef.current) h264DecoderRef.current.reset();
        awaitingKeyframe = true;
      }
      lastFrameSeqRef.current = frameSeq;

      // Keyframe gating
      if (awaitingKeyframe) {
        if (!isKeyframe) return;
        awaitingKeyframe = false;
      }

      // Update stats
      setStreamStats(prev => ({
        ...prev,
        frameId,
        frameSeq,
        lastKeyframeTime: isKeyframe ? Date.now() : prev.lastKeyframeTime,
      }));

      // Feed decoder (only in WS render mode)
      if (h264DecoderRef.current && streamBackend === 'websocket') {
        h264DecoderRef.current.decode(payload, isKeyframe);
      }
    };

    const handleTelemetry = (buffer) => {
      // 0x02 + u16 LE subject_len + subject bytes + payload
      const view = new DataView(buffer);
      const subjectLen = view.getUint16(1, true);
      const subjectBytes = new Uint8Array(buffer, 3, subjectLen);
      const subject = new TextDecoder().decode(subjectBytes);
      const payload = new Uint8Array(buffer, 3 + subjectLen);

      try {
        const data = JSON.parse(new TextDecoder().decode(payload));
        if (subject === 'training.metrics' || subject.includes('metrics')) {
          setTrainingMetrics(data);
          if (data.obs_breakdown) setObsBreakdown(data.obs_breakdown);
          if (data.reward_breakdown) setRewardBreakdown(data.reward_breakdown);
          if (data.velocity_command) setVelocityCommand(data.velocity_command);
        } else if (subject === 'env.info') {
          setEnvInfo(data);
        } else if (subject === 'frame.stats') {
          setFrameStats(data);
        }
      } catch {
        // Non-JSON telemetry payload — ignore
      }
    };

    const handleSignaling = (buffer) => {
      try {
        const json = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 1)));
        handleSignalingMessage(json);
      } catch (e) {
        console.error('Failed to parse signaling message:', e);
      }
    };

    const connect = () => {
      try {
        console.log('Connecting to transport server:', bridgeWsUrl);
        ws = new WebSocket(bridgeWsUrl);
        ws.binaryType = 'arraybuffer';

        ws.onopen = () => {
          console.log('Transport server WebSocket connected');
          setBridgeWs(ws);
          bridgeWsRef.current = ws;
          setGenesisConnected(true);
          awaitingKeyframe = true;
          lastFrameSeqRef.current = -1;
          negotiateSentRef.current = false;
        };

        ws.onmessage = (event) => {
          if (event.data instanceof ArrayBuffer) {
            const buffer = event.data;
            if (buffer.byteLength < 1) return;
            wsBytesAccRef.current += buffer.byteLength;

            const msgType = new DataView(buffer).getUint8(0);
            switch (msgType) {
              case MsgType.VIDEO:     handleVideoFrame(buffer); break;
              case MsgType.TELEMETRY: handleTelemetry(buffer);  break;
              case MsgType.SIGNALING: handleSignaling(buffer);  break;
              default: console.warn('Unknown WS message type:', msgType);
            }
          } else if (typeof event.data === 'string') {
            // Fallback: JSON text messages (Socket.io compat)
            try {
              const msg = JSON.parse(event.data);
              if (msg.type === 'training_metrics') setTrainingMetrics(msg);
              else if (msg.type === 'env_info') setEnvInfo(msg);
              else if (msg.type === 'negotiated') {
                setStreamCodec(msg.stream_codec);
                setH264Codec(msg.h264_codec);
              }
            } catch (e) {
              console.error('Error parsing text message:', e);
            }
          }
        };

        ws.onerror = () => {
          setGenesisConnected(false);
        };

        ws.onclose = () => {
          setBridgeWs(null);
          bridgeWsRef.current = null;
          setGenesisConnected(false);
          reconnectTimer = setTimeout(() => connect(), 3000);
        };
      } catch (error) {
        console.error('Error connecting to transport server:', error);
      }
    };

    connect();

    return () => {
      clearInterval(statsInterval);
      wsStatsIntervalRef.current = null;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      if (ws) ws.close();
      bridgeWsRef.current = null;
    };
  }, [bridgeWsUrl, streamBackend]);

  // Helper: send a 0x03 signaling message over WebSocket
  const sendSignaling = useCallback((payload) => {
    const ws = bridgeWsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    const json = JSON.stringify(payload);
    const jsonBytes = new TextEncoder().encode(json);
    const msg = new Uint8Array(1 + jsonBytes.length);
    msg[0] = MsgType.SIGNALING;
    msg.set(jsonBytes, 1);
    ws.send(msg.buffer);
  }, []);

  // Handle incoming 0x03 signaling messages
  const handleSignalingMessage = useCallback((msg) => {
    const pc = webrtcPcRef.current;
    const sessionId = signalingSessionRef.current;

    switch (msg.type) {
      case 'answer':
        if (msg.session_id !== sessionId) return;
        if (pc) {
          pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: msg.sdp }))
            .catch(e => console.error('Failed to set remote description:', e));
        }
        break;
      case 'ice':
        if (msg.session_id !== sessionId) return;
        if (pc) {
          pc.addIceCandidate(new RTCIceCandidate({
            candidate: msg.candidate,
            sdpMid: msg.sdpMid,
            sdpMLineIndex: msg.sdpMLineIndex,
          })).catch(e => console.warn('Error adding remote ICE candidate:', e));
        }
        break;
      case 'close':
        if (msg.session_id !== sessionId) return;
        teardownWebRTC();
        break;
      default:
        console.warn('Unknown signaling message type:', msg.type);
    }
  }, []);

  // Teardown WebRTC session
  const teardownWebRTC = useCallback(() => {
    controlDcRef.current = null;
    commandsDcRef.current = null;
    if (webrtcPcRef.current) {
      webrtcPcRef.current.close();
      webrtcPcRef.current = null;
    }
    signalingSessionRef.current = null;
    setMediaStream(null);
    setWebrtcConnected(false);
  }, []);

  // Initiate WebRTC upgrade over WS signaling (0x03)
  const initiateWebRTC = useCallback(async () => {
    const ws = bridgeWsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;

    // Tear down existing session
    teardownWebRTC();

    const sessionId = crypto.randomUUID();
    signalingSessionRef.current = sessionId;

    const iceServers = Array.isArray(webrtcStunUrls) && webrtcStunUrls.length > 0
      ? [{ urls: webrtcStunUrls }]
      : [{ urls: 'stun:stun.l.google.com:19302' }];

    try {
      const pc = new RTCPeerConnection({ iceServers });
      webrtcPcRef.current = pc;

      // Create DataChannels BEFORE offer
      const controlDc = pc.createDataChannel('control', { ordered: false, maxRetransmits: 0 });
      const commandsDc = pc.createDataChannel('commands', { ordered: true });

      controlDc.binaryType = 'arraybuffer';
      controlDc.onopen = () => { controlDcRef.current = controlDc; };
      controlDc.onclose = () => { controlDcRef.current = null; };

      commandsDc.onopen = () => { commandsDcRef.current = commandsDc; };
      commandsDc.onclose = () => { commandsDcRef.current = null; };

      // Trickle ICE: send candidates as they arrive
      pc.onicecandidate = (e) => {
        if (e.candidate) {
          sendSignaling({
            type: 'ice',
            session_id: sessionId,
            candidate: e.candidate.candidate,
            sdpMid: e.candidate.sdpMid,
            sdpMLineIndex: e.candidate.sdpMLineIndex,
          });
        }
      };

      pc.ontrack = (e) => {
        if (e.streams && e.streams[0]) {
          setMediaStream(e.streams[0]);
          setWebrtcConnected(true);
          setStreamStats(prev => ({ ...prev, mode: 'rtc' }));
        }
      };

      pc.onconnectionstatechange = () => {
        if (pc.connectionState === 'connected') {
          setWebrtcConnected(true);
        } else if (pc.connectionState === 'failed' || pc.connectionState === 'disconnected') {
          console.warn('WebRTC connection', pc.connectionState, '— falling back to WS');
          teardownWebRTC();
          setStreamStats(prev => ({ ...prev, mode: 'ws' }));
          // WS stays alive, browser auto-falls back to WS rendering
        }
      };

      // Create and send offer
      const offer = await pc.createOffer();
      await pc.setLocalDescription(offer);

      sendSignaling({
        type: 'offer',
        session_id: sessionId,
        sdp: pc.localDescription.sdp,
      });

    } catch (err) {
      console.error('WebRTC initiation error:', err);
      teardownWebRTC();
    }
  }, [webrtcStunUrls, sendSignaling, teardownWebRTC]);

  // Auto-upgrade to WebRTC when setting enabled and WS is connected
  useEffect(() => {
    if (streamBackend === 'webrtc' && genesisConnected && bridgeWsRef.current) {
      initiateWebRTC();
    } else if (streamBackend !== 'webrtc') {
      if (webrtcPcRef.current) {
        sendSignaling({ type: 'close', session_id: signalingSessionRef.current });
        teardownWebRTC();
      }
    }
    return () => {
      if (webrtcReconnectTimerRef.current) {
        clearTimeout(webrtcReconnectTimerRef.current);
        webrtcReconnectTimerRef.current = null;
      }
    };
  }, [streamBackend, genesisConnected, initiateWebRTC, sendSignaling, teardownWebRTC]);
  
  // Listen to Socket.io events for Genesis status
  useEffect(() => {
    if (!socket) return;
    
    const handleGenesisStatus = (data) => {
      setGenesisConnected(data.connected);
    };
    
    const handleScriptStatus = (data) => {
      setScriptStatus(data.status || 'idle');
      setCurrentScriptName(data.script_name || null);
      setScriptError(data.error || null);
    };
    
    const handleFrameStats = (data) => {
      setFrameStats(data);
    };
    
    const handleEnvInfo = (data) => {
      setEnvInfo(data);
    };
    
    const handleRobotList = (data) => {
      console.log('Received robot list:', data);
      setRobotList(data.robots || []);
    };
    
    const handleRobotLoaded = (data) => {
      console.log('Robot loaded:', data);
      setCurrentRobot({
        name: data.robot_name,
        label: data.label || data.robot_name
      });
      // Update memory estimate if provided
      if (data.memory_estimate) {
        setMemoryEstimate(data.memory_estimate);
      }
    };
    
    const handleMemoryEstimate = (data) => {
      console.log('Memory estimate:', data);
      setMemoryEstimate(data);
    };
    
    const handleRobotUnloaded = (data) => {
      console.log('Robot unloaded:', data);
      if (data.success) {
        setCurrentRobot(null);
      }
    };
    
    const handleTrainingMetrics = (data) => {
      setTrainingMetrics(data);
      // Extract structured telemetry if bundled in metrics
      if (data.obs_breakdown) setObsBreakdown(data.obs_breakdown);
      if (data.reward_breakdown) setRewardBreakdown(data.reward_breakdown);
      if (data.velocity_command) setVelocityCommand(data.velocity_command);
    };

    const handleObsBreakdown = (data) => setObsBreakdown(data);
    const handleRewardBreakdown = (data) => setRewardBreakdown(data);
    const handleVelocityCommand = (data) => setVelocityCommand(data);

    const handlePolicyList = (data) => {
      setPolicyList(data.policies || []);
    };
    const handlePolicyLoadStatus = (data) => {
      setPolicyLoadStatus(data.status || null);
      setPolicyLoadError(data.error || null);
    };

    socket.on('genesis_policy_list', handlePolicyList);
    socket.on('genesis_policy_load_status', handlePolicyLoadStatus);
    socket.on('genesis_obs_breakdown', handleObsBreakdown);
    socket.on('genesis_reward_breakdown', handleRewardBreakdown);
    socket.on('genesis_velocity_command', handleVelocityCommand);
    socket.on('genesis_status', handleGenesisStatus);
    socket.on('genesis_script_status', handleScriptStatus);
    socket.on('genesis_frame_stats', handleFrameStats);
    socket.on('genesis_env_info', handleEnvInfo);
    socket.on('genesis_robot_list', handleRobotList);
    socket.on('genesis_robot_loaded', handleRobotLoaded);
    socket.on('genesis_robot_unloaded', handleRobotUnloaded);
    socket.on('genesis_training_metrics', handleTrainingMetrics);
    socket.on('genesis_memory_estimate', handleMemoryEstimate);
    
    // Request robot list on connect
    socket.emit('genesis_scan_robots');
    
    return () => {
      socket.off('genesis_status', handleGenesisStatus);
      socket.off('genesis_script_status', handleScriptStatus);
      socket.off('genesis_frame_stats', handleFrameStats);
      socket.off('genesis_env_info', handleEnvInfo);
      socket.off('genesis_robot_list', handleRobotList);
      socket.off('genesis_robot_loaded', handleRobotLoaded);
      socket.off('genesis_robot_unloaded', handleRobotUnloaded);
      socket.off('genesis_training_metrics', handleTrainingMetrics);
      socket.off('genesis_memory_estimate', handleMemoryEstimate);
      socket.off('genesis_policy_list', handlePolicyList);
      socket.off('genesis_policy_load_status', handlePolicyLoadStatus);
      socket.off('genesis_obs_breakdown', handleObsBreakdown);
      socket.off('genesis_reward_breakdown', handleRewardBreakdown);
      socket.off('genesis_velocity_command', handleVelocityCommand);
    };
  }, [socket]);
  
  // Emit genesis viewer settings to bridge when they change
  const genesisJpegQuality = getSetting('genesis', 'jpegQuality', 80);
  const genesisStreamFps = getSetting('genesis', 'streamFps', 60);
  const genesisCameraRes = getSetting('genesis', 'cameraRes', '1280x720');
  const genesisMetricsRate = getSetting('genesis', 'trainingMetricsRate', 5);

  useEffect(() => {
    if (!socket) return;
    socket.emit('genesis_settings', {
      jpegQuality: genesisJpegQuality,
      streamFps: genesisStreamFps,
      cameraRes: genesisCameraRes,
      trainingMetricsRate: genesisMetricsRate,
    });
  }, [socket, genesisJpegQuality, genesisStreamFps, genesisCameraRes, genesisMetricsRate]);

  // Action: Load a robot
  const loadRobot = useCallback((robotName) => {
    if (socket) {
      console.log('Loading robot:', robotName);
      socket.emit('genesis_load_robot', { robot_name: robotName });
    }
  }, [socket]);
  
  // Action: Unload the current robot
  const unloadRobot = useCallback(() => {
    if (socket) {
      console.log('Unloading robot');
      socket.emit('genesis_unload_robot', {});
    }
  }, [socket]);
  
  // Action: Reset environment
  const resetEnv = useCallback(() => {
    if (socket) {
      console.log('Resetting Genesis environment');
      socket.emit('genesis_reset', {});
    }
  }, [socket]);
  
  // Action: Pause/Resume simulation
  const pauseSim = useCallback((paused) => {
    if (socket) {
      console.log('Genesis pause:', paused);
      socket.emit('genesis_pause', { paused });
    }
  }, [socket]);
  
  // Action: Set Genesis mode
  const setMode = useCallback((mode) => {
    if (socket) {
      console.log('Setting Genesis mode:', mode);
      setGenesisMode(mode);
      socket.emit('genesis_set_mode', { mode });
    }
  }, [socket]);
  
  // Action: Select environment index
  const selectEnv = useCallback((envIdx) => {
    if (socket) {
      console.log('Selecting env index:', envIdx);
      socket.emit('genesis_select_env', { env_idx: envIdx });
    }
  }, [socket]);
  
  // Action: Set camera position
  const setCameraPos = useCallback((position, lookat) => {
    if (socket) {
      socket.emit('genesis_camera', { position, lookat });
    }
  }, [socket]);
  
  // Action: Set target alpha manually
  const setAlpha = useCallback((alpha) => {
    if (socket) {
      socket.emit('genesis_set_alpha', { alpha });
    }
  }, [socket]);

  // Action: Set command source (gamepad or ros)
  const setCommandSource = useCallback((source) => {
    if (socket) {
      socket.emit('genesis_set_command_source', { source });
    }
  }, [socket]);

  // Action: List available policy checkpoints
  const listPolicies = useCallback(() => {
    if (socket) {
      socket.emit('genesis_list_policies');
    }
  }, [socket]);

  // Action: Load a specific policy checkpoint
  const loadPolicy = useCallback((checkpointDir, modelFile = null) => {
    if (socket) {
      setPolicyLoadStatus('loading');
      setPolicyLoadError(null);
      const payload = { checkpoint_dir: checkpointDir };
      if (modelFile) payload.model_file = modelFile;
      socket.emit('genesis_load_policy', payload);
    }
  }, [socket]);

  // Action: Set simulation timestep (dt)
  const setDt = useCallback((newDt) => {
    if (socket) {
      socket.emit('genesis_settings', { dt: newDt });
    }
  }, [socket]);

  // Action: Emergency stop
  const estop = useCallback(() => {
    if (socket) {
      socket.emit('genesis_estop', {});
      // Also pause locally
      pauseSim(true);
    }
  }, [socket, pauseSim]);
  
  // DataChannel: send gamepad axes (binary, unordered)
  const sendGamepadAxes = useCallback((axes, buttonBitmask = 0) => {
    const dc = controlDcRef.current;
    if (!dc || dc.readyState !== 'open') return;
    const axisCount = axes.length;
    const buf = new ArrayBuffer(1 + axisCount * 4 + 4);
    const view = new DataView(buf);
    view.setUint8(0, axisCount);
    for (let i = 0; i < axisCount; i++) {
      view.setFloat32(1 + i * 4, axes[i], true);
    }
    view.setUint32(1 + axisCount * 4, buttonBitmask, true);
    dc.send(buf);
  }, []);

  // DataChannel: send discrete command (reliable, ordered)
  const sendCommand = useCallback((cmd) => {
    const dc = commandsDcRef.current;
    if (!dc || dc.readyState !== 'open') return;
    dc.send(JSON.stringify(cmd));
  }, []);

  // Extract safety/blend state from training metrics
  const blendAlpha = trainingMetrics?.blend_alpha ?? 0.0;
  const deadmanActive = trainingMetrics?.deadman_active ?? false;
  const safetyFlags = trainingMetrics?.safety_flags ?? {};
  const actorTag = trainingMetrics?.actor_tag ?? 'teleop';
  const isPaused = trainingMetrics?.paused ?? false;

  // Command source and policy state (forge path)
  const commandSource = trainingMetrics?.command_source ?? 'gamepad';
  const deadmanActiveForge = trainingMetrics?.deadman_active_forge ?? false;
  const policyLoaded = trainingMetrics?.policy_loaded ?? false;
  const policyCheckpoint = trainingMetrics?.policy_checkpoint ?? null;

  const value = {
    // State
    streamBackend,
    currentFrame,
    mediaStream,
    trainingMetrics,
    genesisConnected: genesisConnected || webrtcConnected,
    webrtcConnected,
    genesisMode,
    robotList,
    currentRobot,  // Currently loaded robot info
    envInfo,
    bridgeConnected: bridgeWs !== null || webrtcConnected,
    socket, // Expose socket for components that need it

    // H.264 codec state
    streamCodec,
    h264Codec,
    h264Support,
    h264DecoderRef,

    // Stream stats (for StreamStats component)
    streamStats,

    // Script status
    scriptStatus,
    currentScriptName,
    scriptError,
    frameStats,
    
    // Memory estimates
    memoryEstimate,
    
    // Derived state
    blendAlpha,
    deadmanActive,
    safetyFlags,
    actorTag,
    isPaused,

    // Structured telemetry
    obsBreakdown,
    rewardBreakdown,
    velocityCommand,

    // Command source and policy
    commandSource,
    deadmanActiveForge,
    policyLoaded,
    policyCheckpoint,

    // Policy browser
    policyList,
    policyLoadStatus,
    policyLoadError,

    // Actions
    loadRobot,
    unloadRobot,
    reset: resetEnv,
    pause: pauseSim,
    isPaused,
    setMode,
    selectEnv,
    setCameraPos,
    setAlpha,
    setCommandSource,
    estop,
    listPolicies,
    loadPolicy,
    setDt,
    initiateWebRTC,
    webrtcPcRef,
    sendGamepadAxes,
    sendCommand,
  };
  
  return (
    <GenesisContext.Provider value={value}>
      {children}
    </GenesisContext.Provider>
  );
};
