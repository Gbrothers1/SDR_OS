import React, { useState, useEffect, useRef, useCallback } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import { useSettings } from '../contexts/SettingsContext';
import { H264Decoder } from '../utils/H264Decoder';
import StreamStats from './StreamStats';
import '../styles/SimViewer.css';

const SimViewer = () => {
  const {
    streamBackend,
    currentFrame,
    mediaStream,
    envInfo,
    genesisConnected,
    bridgeConnected,
    trainingMetrics,
    setCameraPos,
    socket,
    streamCodec,
    h264Codec,
    h264Support,
    h264DecoderRef,
    streamStats,
    webrtcConnected,
    webrtcPcRef,
  } = useGenesis();
  const { getSetting } = useSettings();
  const showStats = getSetting('genesis', 'showStats', false);
  
  const [fps, setFps] = useState(0);
  const frameCountRef = useRef(0);
  const lastFpsUpdateRef = useRef(Date.now());
  const videoRef = useRef(null);
  const h264CanvasRef = useRef(null);
  const [statsVisible, setStatsVisible] = useState(showStats);
  const [rtcStats, setRtcStats] = useState(null);
  const prevStreamBackendRef = useRef(streamBackend);
  
  // Camera state for orbit controls
  const containerRef = useRef(null);
  const [isDragging, setIsDragging] = useState(false);
  const [dragMode, setDragMode] = useState(null); // 'orbit', 'pan', 'zoom'
  const lastMousePos = useRef({ x: 0, y: 0 });
  
  // Camera parameters (spherical coordinates)
  const cameraState = useRef({
    // Spherical coordinates for orbit
    radius: 3.0,
    theta: Math.PI / 4,  // Azimuth angle (horizontal)
    phi: Math.PI / 4,    // Polar angle (vertical)
    // Target point (what the camera looks at)
    target: { x: 0, y: 0, z: 0.5 },
    // Camera up vector
    up: { x: 0, y: 0, z: 1 }
  });

  const defaultCameraState = useRef({
    radius: 3.0, theta: Math.PI / 4, phi: Math.PI / 4,
    target: { x: 0, y: 0, z: 0.5 }, up: { x: 0, y: 0, z: 1 }
  });
  const [containMode, setContainMode] = useState(false);
  const touchStartRef = useRef([]);
  const initialPinchDistance = useRef(null);
  const initialPinchRadius = useRef(null);

  // Convert spherical to cartesian coordinates
  const getCartesianPosition = useCallback(() => {
    const { radius, theta, phi, target } = cameraState.current;
    return {
      x: target.x + radius * Math.sin(phi) * Math.cos(theta),
      y: target.y + radius * Math.sin(phi) * Math.sin(theta),
      z: target.z + radius * Math.cos(phi)
    };
  }, []);
  
  // Send camera update to bridge
  const updateCamera = useCallback(() => {
    if (!socket) return;
    
    const pos = getCartesianPosition();
    const { target } = cameraState.current;
    
    socket.emit('genesis_camera', {
      position: [pos.x, pos.y, pos.z],
      lookat: [target.x, target.y, target.z]
    });
  }, [socket, getCartesianPosition]);

  // Reset camera to default position
  const resetCamera = useCallback(() => {
    const d = defaultCameraState.current;
    Object.assign(cameraState.current, {
      radius: d.radius, theta: d.theta, phi: d.phi,
      target: { ...d.target }
    });
    updateCamera();
  }, [updateCamera]);

  // Mouse event handlers
  const handleMouseDown = useCallback((e) => {
    e.preventDefault();
    setIsDragging(true);
    lastMousePos.current = { x: e.clientX, y: e.clientY };
    
    // Determine drag mode based on button
    if (e.button === 0) {
      // Left click: orbit
      setDragMode('orbit');
    } else if (e.button === 1 || (e.button === 0 && e.shiftKey)) {
      // Middle click or shift+left: pan
      setDragMode('pan');
    } else if (e.button === 2 || (e.button === 0 && e.ctrlKey)) {
      // Right click or ctrl+left: zoom
      setDragMode('zoom');
    }
  }, []);
  
  const handleMouseMove = useCallback((e) => {
    if (!isDragging || !dragMode) return;
    
    const deltaX = e.clientX - lastMousePos.current.x;
    const deltaY = e.clientY - lastMousePos.current.y;
    lastMousePos.current = { x: e.clientX, y: e.clientY };
    
    const sensitivity = 0.005;
    
    if (dragMode === 'orbit') {
      // Orbit: rotate around target
      cameraState.current.theta -= deltaX * sensitivity * 2;
      cameraState.current.phi -= deltaY * sensitivity * 2;
      
      // Clamp phi to prevent flipping
      cameraState.current.phi = Math.max(0.1, Math.min(Math.PI - 0.1, cameraState.current.phi));
      
      updateCamera();
    } else if (dragMode === 'pan') {
      // Pan: move target point
      const { theta, phi, radius } = cameraState.current;
      const panSpeed = radius * sensitivity * 0.5;
      
      // Calculate right and up vectors in camera space
      const rightX = Math.sin(theta + Math.PI / 2);
      const rightY = Math.cos(theta + Math.PI / 2);
      const upX = Math.cos(phi) * Math.cos(theta);
      const upY = Math.cos(phi) * Math.sin(theta);
      const upZ = Math.sin(phi);
      
      cameraState.current.target.x -= (deltaX * rightX + deltaY * upX) * panSpeed;
      cameraState.current.target.y -= (deltaX * rightY + deltaY * upY) * panSpeed;
      cameraState.current.target.z += deltaY * upZ * panSpeed;
      
      updateCamera();
    } else if (dragMode === 'zoom') {
      // Zoom: change radius
      cameraState.current.radius *= 1 + deltaY * sensitivity * 0.5;
      cameraState.current.radius = Math.max(0.5, Math.min(50, cameraState.current.radius));
      
      updateCamera();
    }
  }, [isDragging, dragMode, updateCamera]);
  
  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
    setDragMode(null);
  }, []);
  
  const handleWheel = useCallback((e) => {
    e.preventDefault();
    
    // Zoom with scroll wheel
    const zoomSpeed = 0.001;
    cameraState.current.radius *= 1 + e.deltaY * zoomSpeed;
    cameraState.current.radius = Math.max(0.5, Math.min(50, cameraState.current.radius));
    
    updateCamera();
  }, [updateCamera]);
  
  // Prevent context menu on right click
  const handleContextMenu = useCallback((e) => {
    e.preventDefault();
  }, []);

  // Keyboard handler
  const handleKeyDown = useCallback((e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
    switch (e.key.toLowerCase()) {
      case 'z': resetCamera(); break;
      case 'c': setContainMode(prev => !prev); break;
      case 's': setStatsVisible(prev => !prev); break;
      case 'f':
        if (containerRef.current) {
          document.fullscreenElement ? document.exitFullscreen() : containerRef.current.requestFullscreen();
        }
        break;
    }
  }, [resetCamera]);

  // Touch handlers
  const handleTouchStart = useCallback((e) => {
    const touches = Array.from(e.touches);
    touchStartRef.current = touches.map(t => ({ x: t.clientX, y: t.clientY }));
    if (touches.length === 1) {
      setIsDragging(true); setDragMode('orbit');
      lastMousePos.current = { x: touches[0].clientX, y: touches[0].clientY };
    } else if (touches.length === 2) {
      setIsDragging(true);
      const dx = touches[1].clientX - touches[0].clientX;
      const dy = touches[1].clientY - touches[0].clientY;
      initialPinchDistance.current = Math.hypot(dx, dy);
      initialPinchRadius.current = cameraState.current.radius;
      setDragMode('pan');
      lastMousePos.current = {
        x: (touches[0].clientX + touches[1].clientX) / 2,
        y: (touches[0].clientY + touches[1].clientY) / 2
      };
    }
  }, []);

  const handleTouchMove = useCallback((e) => {
    e.preventDefault();
    const touches = Array.from(e.touches);
    if (touches.length === 2 && initialPinchDistance.current != null) {
      const dx = touches[1].clientX - touches[0].clientX;
      const dy = touches[1].clientY - touches[0].clientY;
      const scale = initialPinchDistance.current / Math.hypot(dx, dy);
      cameraState.current.radius = Math.max(0.5, Math.min(50, initialPinchRadius.current * scale));
      updateCamera();
    } else if (touches.length === 1 && isDragging) {
      const deltaX = touches[0].clientX - lastMousePos.current.x;
      const deltaY = touches[0].clientY - lastMousePos.current.y;
      lastMousePos.current = { x: touches[0].clientX, y: touches[0].clientY };
      cameraState.current.theta -= deltaX * 0.01;
      cameraState.current.phi = Math.max(0.1, Math.min(Math.PI - 0.1, cameraState.current.phi - deltaY * 0.01));
      updateCamera();
    }
  }, [isDragging, updateCamera]);

  const handleTouchEnd = useCallback((e) => {
    if (e.touches.length === 0) {
      setIsDragging(false); setDragMode(null);
      initialPinchDistance.current = null;
    }
  }, []);

  // Add/remove event listeners
  useEffect(() => {
    const container = containerRef.current;
    if (!container) return;
    
    // Add mouse listeners
    container.addEventListener('mousedown', handleMouseDown);
    container.addEventListener('wheel', handleWheel, { passive: false });
    container.addEventListener('contextmenu', handleContextMenu);

    // Add touch listeners
    container.addEventListener('touchstart', handleTouchStart, { passive: false });
    container.addEventListener('touchmove', handleTouchMove, { passive: false });
    container.addEventListener('touchend', handleTouchEnd);

    // Add global listeners for mouse move/up (to handle dragging outside container)
    window.addEventListener('mousemove', handleMouseMove);
    window.addEventListener('mouseup', handleMouseUp);
    window.addEventListener('keydown', handleKeyDown);

    return () => {
      container.removeEventListener('mousedown', handleMouseDown);
      container.removeEventListener('wheel', handleWheel);
      container.removeEventListener('contextmenu', handleContextMenu);
      container.removeEventListener('touchstart', handleTouchStart);
      container.removeEventListener('touchmove', handleTouchMove);
      container.removeEventListener('touchend', handleTouchEnd);
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
      window.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleMouseDown, handleMouseMove, handleMouseUp, handleWheel, handleContextMenu, handleTouchStart, handleTouchMove, handleTouchEnd, handleKeyDown]);
  
  // Attach mediaStream to video element when using WebRTC
  useEffect(() => {
    const video = videoRef.current;
    if (!video) return;
    if (streamBackend === 'webrtc' && mediaStream) {
      video.srcObject = mediaStream;
    } else {
      video.srcObject = null;
    }
    return () => {
      video.srcObject = null;
    };
  }, [streamBackend, mediaStream]);

  // Initialize H264Decoder when codec is h264
  useEffect(() => {
    if (streamCodec === 'h264' && h264CanvasRef.current && h264Support?.supported) {
      const decoder = new H264Decoder(h264CanvasRef.current);
      decoder.init(h264Codec || h264Support.config.codec);
      h264DecoderRef.current = decoder;

      return () => {
        decoder.close();
        h264DecoderRef.current = null;
      };
    }
  }, [streamCodec, h264Codec, h264Support, h264DecoderRef]);

  // Reset H264 decoder on render mode switch (WS ↔ RTC)
  useEffect(() => {
    if (prevStreamBackendRef.current !== streamBackend) {
      if (h264DecoderRef.current) {
        h264DecoderRef.current.reset();
      }
      prevStreamBackendRef.current = streamBackend;
    }
  }, [streamBackend, h264DecoderRef]);

  // Poll WebRTC stats when connected
  useEffect(() => {
    if (!webrtcConnected || !webrtcPcRef?.current) {
      setRtcStats(null);
      return;
    }
    const interval = setInterval(async () => {
      try {
        const stats = await webrtcPcRef.current.getStats();
        let kbps = 0, rtt = 0;
        stats.forEach(report => {
          if (report.type === 'inbound-rtp' && report.kind === 'video') {
            kbps = Math.round((report.bytesReceived || 0) / 1024);
          }
          if (report.type === 'candidate-pair' && report.currentRoundTripTime != null) {
            rtt = Math.round(report.currentRoundTripTime * 1000);
          }
        });
        setRtcStats({ kbps, rtt });
      } catch {
        // PC may have been closed
      }
    }, 500);
    return () => clearInterval(interval);
  }, [webrtcConnected, webrtcPcRef]);

  // Calculate FPS (WebSocket: from frame updates; WebRTC: from metrics or video)
  useEffect(() => {
    if (streamBackend === 'webrtc') {
      if (trainingMetrics?.fps != null) {
        setFps(Math.round(trainingMetrics.fps));
      }
      return;
    }
    if (!currentFrame) return;
    
    frameCountRef.current++;
    
    const now = Date.now();
    const elapsed = now - lastFpsUpdateRef.current;
    
    if (elapsed >= 1000) {
      setFps(Math.round((frameCountRef.current / elapsed) * 1000));
      frameCountRef.current = 0;
      lastFpsUpdateRef.current = now;
    }
  }, [streamBackend, currentFrame, trainingMetrics?.fps]);
  
  const hasVideo = streamCodec === 'h264'
    ? (h264Support?.supported && h264DecoderRef?.current)
    : (streamBackend === 'websocket' ? !!currentFrame : !!mediaStream);

  // Connection status indicator
  const getStatusText = () => {
    if (!bridgeConnected) {
      return 'Bridge Disconnected';
    }
    if (!genesisConnected) {
      return 'Waiting for Genesis...';
    }
    if (streamCodec === 'h264') {
      return 'Connected (H.264)';
    }
    return streamBackend === 'webrtc' ? 'Connected (WebRTC)' : 'Connected';
  };
  
  const getStatusClass = () => {
    if (!bridgeConnected || !genesisConnected) {
      return 'status-disconnected';
    }
    return 'status-connected';
  };
  
  return (
    <div className={`sim-viewer${containMode ? ' contain-mode' : ''}`} ref={containerRef}>
      {/* Frame display */}
      <div className="sim-viewer-frame">
        {streamCodec === 'h264' && h264Support?.supported ? (
          <canvas
            ref={h264CanvasRef}
            className="sim-frame-image sim-viewer__canvas"
            style={{
              width: '100%',
              height: '100%',
              objectFit: containMode ? 'contain' : 'cover',
            }}
          />
        ) : streamBackend === 'websocket' && currentFrame ? (
          <img
            src={currentFrame}
            alt="Genesis simulation"
            className="sim-frame-image"
            draggable={false}
            style={{ objectFit: containMode ? 'contain' : 'cover' }}
          />
        ) : streamBackend === 'webrtc' && mediaStream ? (
          <video
            ref={videoRef}
            className="sim-frame-image"
            autoPlay
            playsInline
            muted
            style={{ width: '100%', height: '100%', objectFit: 'contain' }}
          />
        ) : (
          <div className="sim-no-frame">
            <div className="sim-no-frame-message">
              {!bridgeConnected && (
                <>
                  <div className="sim-icon">⚠️</div>
                  <div>Genesis Bridge not connected</div>
                  <div className="sim-hint">
                    {streamBackend === 'webrtc'
                      ? `WebRTC: connect to ${webrtcSignalingUrl}`
                      : 'Check bridge server at ws://localhost:9091'}
                  </div>
                </>
              )}
              {bridgeConnected && !genesisConnected && (
                <>
                  <div className="sim-icon">⏳</div>
                  <div>Waiting for Genesis...</div>
                  <div className="sim-hint">Bridge connected, waiting for simulation</div>
                </>
              )}
              {bridgeConnected && genesisConnected && !hasVideo && (
                <>
                  <div className="sim-icon">📡</div>
                  <div>Waiting for frames...</div>
                </>
              )}
            </div>
          </div>
        )}
      </div>
      
      {/* Overlay info */}
      <div className="sim-viewer-overlay">
        {/* Top left: Connection status */}
        <div className={`sim-status ${getStatusClass()}`}>
          <span className="sim-status-dot"></span>
          {getStatusText()}
        </div>
        
        {/* Top right: FPS and env info */}
        {hasVideo && (
          <div className="sim-info">
            <div className="sim-info-item">
              <span className="sim-info-label">FPS:</span>
              <span className="sim-info-value">{fps}</span>
            </div>
            {envInfo && (
              <>
                <div className="sim-info-item">
                  <span className="sim-info-label">Robot:</span>
                  <span className="sim-info-value">{envInfo.robot_name || 'Unknown'}</span>
                </div>
                <div className="sim-info-item">
                  <span className="sim-info-label">Envs:</span>
                  <span className="sim-info-value">{envInfo.num_envs || 0}</span>
                </div>
                {envInfo.mode && (
                  <div className="sim-info-item">
                    <span className="sim-info-label">Mode:</span>
                    <span className="sim-info-value">{envInfo.mode}</span>
                  </div>
                )}
              </>
            )}
            {trainingMetrics && (
              <div className="sim-info-item">
                <span className="sim-info-label">Reward:</span>
                <span className="sim-info-value">
                  {trainingMetrics.total_reward?.toFixed(2) || '0.00'}
                </span>
              </div>
            )}
          </div>
        )}
      </div>
      
      {/* Camera controls hint */}
      {hasVideo && (
        <div className="sim-camera-hint">
          <span>Drag: orbit</span>
          <span>Shift+drag: pan</span>
          <span>Scroll: zoom</span>
          <span>Z: reset</span>
          <span>S: stats</span>
        </div>
      )}

      {/* Stream stats strip */}
      {statsVisible && hasVideo && (
        <StreamStats
          fps={fps}
          frameId={streamStats?.frameId ?? 0}
          lastKeyframeTime={streamStats?.lastKeyframeTime ?? null}
          wsBytesPerSec={streamStats?.wsBytesPerSec ?? 0}
          rtcStats={rtcStats}
          mode={streamStats?.mode ?? 'ws'}
          decoderQueueSize={h264DecoderRef?.current?.getStats()?.queueSize ?? 0}
          maxQueueSize={3}
        />
      )}
    </div>
  );
};

export default SimViewer;
