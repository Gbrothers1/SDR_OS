import React, { createContext, useContext, useState, useEffect, useCallback, useRef } from 'react';
import { useSettings } from './SettingsContext';
import { detectH264Support, H264Decoder, parseFrameHeader, FrameType } from '../utils/H264Decoder';

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
  const bridgeWsUrl = getSetting('genesis', 'bridgeWsUrl', `ws://${window.location.hostname}:9091`);
  const webrtcSignalingUrl = getSetting('genesis', 'webrtcSignalingUrl', `http://${window.location.hostname}:9092`);
  const webrtcStunUrls = getSetting('genesis', 'webrtcStunUrls', ['stun:stun.l.google.com:19302']) || [];

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

  // Connect to Genesis bridge WebSocket (only when streamBackend === 'websocket')
  useEffect(() => {
    if (streamBackend !== 'websocket') {
      if (bridgeWs) {
        bridgeWs.close();
        setBridgeWs(null);
      }
      setGenesisConnected(false);
      setCurrentFrame(null);
      currentFrameRef.current = null;
      blobUrlsRef.current.forEach(url => URL.revokeObjectURL(url));
      blobUrlsRef.current.clear();
      return;
    }
    let ws = null;
    let reconnectTimer = null;
    
    const connect = () => {
      try {
        console.log('Connecting to Genesis bridge (WebSocket):', bridgeWsUrl);
        ws = new WebSocket(bridgeWsUrl);
        
        ws.onopen = () => {
          console.log('Genesis bridge WebSocket connected');
          setBridgeWs(ws);
          setGenesisConnected(true);
          negotiateSentRef.current = false; // Reset for new connection
        };
        
        ws.onmessage = (event) => {
          if (event.data instanceof Blob) {
            // Convert Blob to ArrayBuffer for parsing
            event.data.arrayBuffer().then(buffer => {
              const { type, frameId, flags, payload } = parseFrameHeader(buffer);

              if (type === FrameType.H264 && h264DecoderRef.current) {
                // H.264 path
                h264DecoderRef.current.decode(payload, frameId, flags);
              } else if (type === FrameType.JPEG || type === 0) {
                // JPEG path (type 0 for backwards compatibility with old protocol)
                if (currentFrameRef.current && blobUrlsRef.current.has(currentFrameRef.current)) {
                  URL.revokeObjectURL(currentFrameRef.current);
                  blobUrlsRef.current.delete(currentFrameRef.current);
                }
                const blob = new Blob([payload], { type: 'image/jpeg' });
                const blobUrl = URL.createObjectURL(blob);
                blobUrlsRef.current.add(blobUrl);
                currentFrameRef.current = blobUrl;
                // Trim runaway growth if cleanup ever lags
                if (blobUrlsRef.current.size > 30) {
                  const oldest = blobUrlsRef.current.values().next().value;
                  URL.revokeObjectURL(oldest);
                  blobUrlsRef.current.delete(oldest);
                }
                setCurrentFrame(blobUrl);
              }
            }).catch(err => {
              console.error('Failed to read frame buffer:', err);
            });
            return;
          }
          try {
            const msg = JSON.parse(event.data);
            if (msg.type === 'training_metrics') {
              setTrainingMetrics(msg);
            } else if (msg.type === 'env_info') {
              setEnvInfo(msg);
            } else if (msg.type === 'negotiated') {
              setStreamCodec(msg.stream_codec);
              setH264Codec(msg.h264_codec);
              console.log('Stream codec negotiated:', msg.stream_codec);
            }
          } catch (e) {
            console.error('Error parsing Genesis message:', e);
          }
        };
        
        ws.onerror = () => {
          setGenesisConnected(false);
        };
        
        ws.onclose = () => {
          setBridgeWs(null);
          setGenesisConnected(false);
          reconnectTimer = setTimeout(() => connect(), 3000);
        };
      } catch (error) {
        console.error('Error connecting to Genesis bridge:', error);
      }
    };
    
    connect();
    
    return () => {
      if (reconnectTimer) clearTimeout(reconnectTimer);
      if (ws) ws.close();
      blobUrlsRef.current.forEach(url => URL.revokeObjectURL(url));
      blobUrlsRef.current.clear();
    };
  }, [streamBackend, bridgeWsUrl]);

  // Send codec negotiation after both WebSocket is connected AND h264Support is determined
  useEffect(() => {
    if (!bridgeWs || bridgeWs.readyState !== WebSocket.OPEN) {
      return;
    }
    if (h264Support === null) {
      return; // Still detecting H.264 support
    }
    if (negotiateSentRef.current) {
      return; // Already sent for this connection
    }

    // Send negotiate message now that we know our capabilities
    if (h264Support.supported) {
      bridgeWs.send(JSON.stringify({
        type: 'negotiate',
        supports_h264: true,
        h264_codec: h264Support.config.codec
      }));
    } else {
      bridgeWs.send(JSON.stringify({
        type: 'negotiate',
        supports_h264: false
      }));
    }
    negotiateSentRef.current = true;
    console.log('Sent codec negotiation:', h264Support.supported ? h264Support.config.codec : 'jpeg-only');
  }, [bridgeWs, h264Support]);

  // Connect via WebRTC (only when streamBackend === 'webrtc')
  useEffect(() => {
    if (streamBackend !== 'webrtc') {
      if (webrtcPcRef.current) {
        webrtcPcRef.current.close();
        webrtcPcRef.current = null;
      }
      setMediaStream(null);
      setWebrtcConnected(false);
      if (webrtcReconnectTimerRef.current) {
        clearTimeout(webrtcReconnectTimerRef.current);
        webrtcReconnectTimerRef.current = null;
      }
      return;
    }
    let cancelled = false;
    const iceServers = Array.isArray(webrtcStunUrls) && webrtcStunUrls.length > 0
      ? [{ urls: webrtcStunUrls }]
      : [{ urls: 'stun:stun.l.google.com:19302' }];
    
    const connectWebRTC = async () => {
      try {
        const pc = new RTCPeerConnection({ iceServers });
        webrtcPcRef.current = pc;
        const localCandidates = [];
        pc.onicecandidate = (e) => {
          if (e.candidate) localCandidates.push(e.candidate);
        };
        // Register track/state handlers BEFORE offer/answer exchange
        pc.ontrack = (e) => {
          if (cancelled) return;
          if (e.streams && e.streams[0]) {
            setMediaStream(e.streams[0]);
            setWebrtcConnected(true);
          }
        };
        pc.onconnectionstatechange = () => {
          if (cancelled) return;
          if (pc.connectionState === 'connected') {
            setWebrtcConnected(true);
          } else if (pc.connectionState === 'failed' || pc.connectionState === 'closed' || pc.connectionState === 'disconnected') {
            setWebrtcConnected(false);
            setMediaStream(null);
            webrtcReconnectTimerRef.current = setTimeout(connectWebRTC, 3000);
          }
        };
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        await new Promise((resolve) => {
          if (pc.iceGatheringState === 'complete') return resolve();
          pc.onicegatheringstatechange = () => {
            if (pc.iceGatheringState === 'complete') resolve();
          };
          setTimeout(resolve, 2000);
        });
        const offerPayload = {
          type: pc.localDescription.type,
          sdp: pc.localDescription.sdp,
          candidates: localCandidates.map(c => ({ candidate: c.candidate, mid: c.sdpMid, sdpMLineIndex: c.sdpMLineIndex })),
        };
        const baseUrl = webrtcSignalingUrl.replace(/\/$/, '');
        const res = await fetch(`${baseUrl}/offer`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(offerPayload),
        });
        if (!res.ok || cancelled) {
          throw new Error(`WebRTC offer failed: ${res.status}`);
        }
        const answerData = await res.json();
        await pc.setRemoteDescription(new RTCSessionDescription({ type: answerData.type, sdp: answerData.sdp }));
        const remoteCandidates = answerData.candidates || [];
        for (const c of remoteCandidates) {
          try {
            await pc.addIceCandidate(new RTCIceCandidate(c));
          } catch (err) {
            console.warn('Error adding remote ICE candidate:', err);
          }
        }
      } catch (err) {
        console.error('WebRTC connection error:', err);
        setWebrtcConnected(false);
        setMediaStream(null);
        if (!cancelled) {
          webrtcReconnectTimerRef.current = setTimeout(connectWebRTC, 3000);
        }
      }
    };
    connectWebRTC();
    return () => {
      cancelled = true;
      if (webrtcReconnectTimerRef.current) {
        clearTimeout(webrtcReconnectTimerRef.current);
        webrtcReconnectTimerRef.current = null;
      }
      if (webrtcPcRef.current) {
        webrtcPcRef.current.close();
        webrtcPcRef.current = null;
      }
      setMediaStream(null);
      setWebrtcConnected(false);
    };
  }, [streamBackend, webrtcSignalingUrl, webrtcStunUrls]);
  
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

  // Action: Emergency stop
  const estop = useCallback(() => {
    if (socket) {
      socket.emit('genesis_estop', {});
      // Also pause locally
      pauseSim(true);
    }
  }, [socket, pauseSim]);
  
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
  };
  
  return (
    <GenesisContext.Provider value={value}>
      {children}
    </GenesisContext.Provider>
  );
};
