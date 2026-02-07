import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from 'react';
import { useSettings } from './SettingsContext';
import { detectH264Support, H264Decoder, MsgType, CodecType, parseVideoHeader } from '../utils/H264Decoder';

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

  // Safety state (driven by telemetry.safety.state from sim)
  const [safetyState, setSafetyState] = useState({ state_id: 0, mode: 'ARMED', reason: 'ok', since_ms: 0 });
  const [videoHealthy, setVideoHealthy] = useState(true);
  const lastFrameTimeRef = useRef(Date.now());
  const lastSafetyStateIdRef = useRef(0);

  // Command sequence counter for 0x04 messages
  const cmdSeqRef = useRef(0);

  // Track previous JPEG frame URL for cleanup
  const prevFrameUrlRef = useRef(null);

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
      const { frameId, frameSeq, isKeyframe, codec, payload } = parseVideoHeader(view);

      // Update video health tracking
      lastFrameTimeRef.current = Date.now();
      if (!videoHealthy) setVideoHealthy(true);

      // Detect sequence discontinuity → reset decoder
      if (lastFrameSeqRef.current >= 0 && frameSeq > lastFrameSeqRef.current + 1) {
        console.warn(`Frame seq gap: ${lastFrameSeqRef.current} → ${frameSeq}`);
        if (h264DecoderRef.current) h264DecoderRef.current.reset();
        awaitingKeyframe = true;
      }
      lastFrameSeqRef.current = frameSeq;

      // Keyframe gating (for H.264)
      if (awaitingKeyframe && codec === CodecType.H264) {
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

      // Codec-aware frame routing
      if (codec === CodecType.H264) {
        if (h264DecoderRef.current && streamBackend !== 'webrtc') {
          h264DecoderRef.current.decode(payload, isKeyframe);
        }
        if (streamCodec !== 'h264') setStreamCodec('h264');
      } else if (codec === CodecType.JPEG) {
        // JPEG: create object URL for img tag rendering
        if (prevFrameUrlRef.current) {
          URL.revokeObjectURL(prevFrameUrlRef.current);
        }
        const blob = new Blob([payload], { type: 'image/jpeg' });
        const url = URL.createObjectURL(blob);
        prevFrameUrlRef.current = url;
        setCurrentFrame(url);
        if (streamCodec !== 'jpeg') setStreamCodec('jpeg');
      }
    };

    const handleTelemetry = (buffer) => {
      // 0x02 + u16 LE subject_len + subject bytes + payload
      const view = new DataView(buffer);
      const subjectLen = view.getUint16(1, true);
      const subjectBytes = new Uint8Array(buffer, 3, subjectLen);
      const subject = new TextDecoder().decode(subjectBytes);
      const telPayload = new Uint8Array(buffer, 3 + subjectLen);

      try {
        const data = JSON.parse(new TextDecoder().decode(telPayload));

        // Route by NATS subject
        if (subject === 'telemetry.training.metrics') {
          setTrainingMetrics(data);
          if (data.velocity_command) setVelocityCommand(data.velocity_command);
        } else if (subject === 'telemetry.reward.breakdown') {
          setRewardBreakdown(data);
        } else if (subject === 'telemetry.obs.breakdown') {
          setObsBreakdown(data);
        } else if (subject === 'telemetry.velocity.command') {
          setVelocityCommand(data);
        } else if (subject === 'telemetry.safety.state') {
          // Canonical safety state from sim — ignore out-of-order
          if (data.state_id > lastSafetyStateIdRef.current) {
            lastSafetyStateIdRef.current = data.state_id;
            setSafetyState(data);
          }
        } else if (subject === 'telemetry.policy.list') {
          setPolicyList(data.policies || []);
        } else if (subject === 'telemetry.command.ack') {
          // Command acknowledgment — could be used for UI feedback
          if (data.action === 'load_policy') {
            setPolicyLoadStatus(data.status);
          } else if (data.action === 'list_policies' && data.status !== 'ok') {
            console.warn('list_policies failed:', data);
          }
        } else if (subject === 'telemetry.frame.stats') {
          setFrameStats(data);
        } else if (subject === 'telemetry.safety.video_gate') {
          // Transport video gate state — informational
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

    // Video health check (Layer 1: 500ms)
    const videoHealthInterval = setInterval(() => {
      const age = Date.now() - lastFrameTimeRef.current;
      if (age > 500) {
        setVideoHealthy(false);
      }
    }, 200);

    connect();

    return () => {
      clearInterval(statsInterval);
      clearInterval(videoHealthInterval);
      wsStatsIntervalRef.current = null;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      if (ws) ws.close();
      bridgeWsRef.current = null;
      if (prevFrameUrlRef.current) {
        URL.revokeObjectURL(prevFrameUrlRef.current);
        prevFrameUrlRef.current = null;
      }
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

  // Helper: send a 0x04 command message over WebSocket
  const sendWsCommand = useCallback((action, data = {}, ttlMs = null) => {
    const ws = bridgeWsRef.current;
    if (!ws || ws.readyState !== WebSocket.OPEN) return;

    cmdSeqRef.current += 1;
    const cmd = { action, cmd_seq: cmdSeqRef.current, data };
    if (ttlMs !== null) cmd.ttl_ms = ttlMs;

    const jsonStr = JSON.stringify(cmd);
    const jsonBytes = new TextEncoder().encode(jsonStr);
    const msg = new Uint8Array(1 + jsonBytes.length);
    msg[0] = MsgType.COMMAND;
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
  
  // Send genesis viewer settings via 0x04 when they change
  const genesisJpegQuality = getSetting('genesis', 'jpegQuality', 80);
  const genesisStreamFps = getSetting('genesis', 'streamFps', 60);
  const genesisCameraRes = getSetting('genesis', 'cameraRes', '1280x720');
  const genesisH264Bitrate = getSetting('genesis', 'h264Bitrate', 3);
  const genesisH264Preset = getSetting('genesis', 'h264Preset', 'llhp');

  useEffect(() => {
    if (!genesisConnected) return;
    sendWsCommand('settings', {
      jpeg_quality: genesisJpegQuality,
      stream_fps: genesisStreamFps,
      camera_res: genesisCameraRes,
      h264_bitrate: genesisH264Bitrate,
      h264_preset: genesisH264Preset,
    });
  }, [genesisConnected, genesisJpegQuality, genesisStreamFps, genesisCameraRes, genesisH264Bitrate, genesisH264Preset, sendWsCommand]);

  // Action: Load a robot
  const loadRobot = useCallback((robotName) => {
    sendWsCommand('load_robot', { robot_name: robotName });
  }, [sendWsCommand]);

  // Action: Unload the current robot
  const unloadRobot = useCallback(() => {
    sendWsCommand('unload_robot');
  }, [sendWsCommand]);

  // Action: Reset environment
  const resetEnv = useCallback(() => {
    sendWsCommand('reset');
  }, [sendWsCommand]);

  // Action: Pause/Resume simulation
  const pauseSim = useCallback((paused) => {
    sendWsCommand('pause', { paused });
  }, [sendWsCommand]);

  // Action: Set Genesis mode
  const setMode = useCallback((mode) => {
    setGenesisMode(mode);
    sendWsCommand('set_mode', { mode });
  }, [sendWsCommand]);

  // Action: Select environment index
  const selectEnv = useCallback((envIdx) => {
    sendWsCommand('select_env', { env_idx: envIdx });
  }, [sendWsCommand]);

  // Action: Set camera position
  const setCameraPos = useCallback((position, lookat) => {
    sendWsCommand('camera', { position, lookat });
  }, [sendWsCommand]);

  // Action: Set target alpha manually
  const setAlpha = useCallback((alpha) => {
    sendWsCommand('set_alpha', { alpha });
  }, [sendWsCommand]);

  // Action: Set command source (gamepad or ros)
  const setCommandSource = useCallback((source) => {
    sendWsCommand('set_command_source', { source });
  }, [sendWsCommand]);

  // Action: List available policy checkpoints
  const listPolicies = useCallback(() => {
    sendWsCommand('list_policies');
  }, [sendWsCommand]);

  // Action: Load a specific policy checkpoint
  const loadPolicy = useCallback((checkpointDir, modelFile = null) => {
    setPolicyLoadStatus('loading');
    setPolicyLoadError(null);
    const payload = { checkpoint_dir: checkpointDir };
    if (modelFile) payload.model_file = modelFile;
    sendWsCommand('load_policy', payload);
  }, [sendWsCommand]);

  // Action: Set simulation timestep (dt)
  const setDt = useCallback((newDt) => {
    sendWsCommand('settings', { dt: newDt });
  }, [sendWsCommand]);

  // Action: Emergency stop
  const estop = useCallback(() => {
    sendWsCommand('estop', { reason: 'operator' });
  }, [sendWsCommand]);

  // Action: Clear E-STOP (re-arm)
  const estopClear = useCallback(() => {
    sendWsCommand('estop_clear');
  }, [sendWsCommand]);

  // Action: Send velocity command with TTL
  const sendVelocityCommand = useCallback((linearX, linearY, angularZ) => {
    // Layer 1: don't send commands when video is lost
    if (!videoHealthy) return;
    sendWsCommand('set_cmd_vel', { linear_x: linearX, linear_y: linearY, angular_z: angularZ }, 200);
  }, [sendWsCommand, videoHealthy]);
  
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
    currentRobot,
    envInfo,
    bridgeConnected: bridgeWs !== null || webrtcConnected,
    socket,

    // H.264 codec state
    streamCodec,
    h264Codec,
    h264Support,
    h264DecoderRef,

    // Stream stats (for StreamStats component)
    streamStats,

    // Safety state (from canonical telemetry.safety.state)
    safetyState,
    videoHealthy,

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
    estopClear,
    listPolicies,
    loadPolicy,
    setDt,
    initiateWebRTC,
    webrtcPcRef,
    sendGamepadAxes,
    sendCommand,
    sendWsCommand,
    sendVelocityCommand,
  };
  
  return (
    <GenesisContext.Provider value={value}>
      {children}
    </GenesisContext.Provider>
  );
};
