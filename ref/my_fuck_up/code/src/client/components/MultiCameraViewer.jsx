import React, { useState, useEffect, useRef } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import { useSettings } from '../contexts/SettingsContext';
import '../styles/MultiCameraViewer.css';

const MultiCameraViewer = () => {
  const { socket } = useGenesis();
  const { getSetting } = useSettings();
  const [cameras, setCameras] = useState([]);
  const [activeCamera, setActiveCamera] = useState(null);
  const [cameraSources, setCameraSources] = useState({});
  const [layout, setLayout] = useState('main'); // 'main', 'grid', 'pip'
  
  // Track Blob URLs for cleanup
  const blobUrlsRef = useRef(new Map());
  
  // WebSocket connections for each camera
  const wsConnectionsRef = useRef(new Map());
  
  useEffect(() => {
    if (!socket) return;
    
    // Request camera list
    socket.emit('genesis_get_camera_list');
    
    // Listen for camera list
    const handleCameraList = (data) => {
      console.log('Received camera list:', data);
      setCameras(data.cameras || []);
      
      // Set first camera as active if none selected
      if (data.cameras && data.cameras.length > 0 && !activeCamera) {
        setActiveCamera(data.cameras[0].name);
      }
    };
    
    socket.on('genesis_camera_list', handleCameraList);
    
    return () => {
      socket.off('genesis_camera_list', handleCameraList);
    };
  }, [socket, activeCamera]);
  
  useEffect(() => {
    // Connect to WebSocket streams for each camera
    cameras.forEach(camera => {
      const cameraName = camera.name;
      
      // Skip if already connected
      if (wsConnectionsRef.current.has(cameraName)) {
        return;
      }
      
      // Create WebSocket connection
      // Assuming genesis bridge provides per-camera streams
      const baseUrl = getSetting('genesis', 'bridgeWsUrl', `ws://${window.location.hostname}:9091`);
      const wsUrl = `${baseUrl}/camera/${cameraName}`;
      const ws = new WebSocket(wsUrl);
      
      ws.onopen = () => {
        console.log(`Camera ${cameraName} WebSocket connected`);
      };
      
      ws.onmessage = (event) => {
        if (event.data instanceof Blob) {
          // Clean up old Blob URL
          const oldUrl = blobUrlsRef.current.get(cameraName);
          if (oldUrl) {
            URL.revokeObjectURL(oldUrl);
          }
          
          // Create new Blob URL
          const url = URL.createObjectURL(event.data);
          blobUrlsRef.current.set(cameraName, url);
          
          // Update state
          setCameraSources(prev => ({
            ...prev,
            [cameraName]: url
          }));
        }
      };
      
      ws.onerror = (error) => {
        console.error(`Camera ${cameraName} WebSocket error:`, error);
      };
      
      ws.onclose = () => {
        console.log(`Camera ${cameraName} WebSocket closed`);
        wsConnectionsRef.current.delete(cameraName);
      };
      
      wsConnectionsRef.current.set(cameraName, ws);
    });
    
    // Cleanup on unmount or camera list change
    return () => {
      wsConnectionsRef.current.forEach((ws, cameraName) => {
        ws.close();
      });
      wsConnectionsRef.current.clear();
      
      // Revoke all Blob URLs
      blobUrlsRef.current.forEach(url => URL.revokeObjectURL(url));
      blobUrlsRef.current.clear();
    };
  }, [cameras]);
  
  const handleCameraSelect = (cameraName) => {
    setActiveCamera(cameraName);
  };
  
  const handleLayoutChange = (newLayout) => {
    setLayout(newLayout);
  };
  
  if (cameras.length === 0) {
    return (
      <div className="multi-camera-viewer no-cameras">
        <div className="no-cameras-message">
          <span className="icon">📷</span>
          <p>No cameras available</p>
          <p className="hint">Load a robot with cameras configured</p>
        </div>
      </div>
    );
  }
  
  return (
    <div className={`multi-camera-viewer layout-${layout}`}>
      {/* Layout controls */}
      <div className="camera-controls">
        <div className="layout-buttons">
          <button
            className={`layout-btn ${layout === 'main' ? 'active' : ''}`}
            onClick={() => handleLayoutChange('main')}
            title="Main view with thumbnails"
          >
            <span>▢</span>
          </button>
          <button
            className={`layout-btn ${layout === 'grid' ? 'active' : ''}`}
            onClick={() => handleLayoutChange('grid')}
            title="Grid layout"
          >
            <span>▦</span>
          </button>
          <button
            className={`layout-btn ${layout === 'pip' ? 'active' : ''}`}
            onClick={() => handleLayoutChange('pip')}
            title="Picture-in-picture"
          >
            <span>⊡</span>
          </button>
        </div>
      </div>
      
      {/* Main layout */}
      {layout === 'main' && (
        <>
          <div className="main-camera-view">
            {activeCamera && cameraSources[activeCamera] ? (
              <img
                src={cameraSources[activeCamera]}
                alt={`Camera ${activeCamera}`}
                className="camera-image"
              />
            ) : (
              <div className="camera-placeholder">
                <span>Waiting for {activeCamera} stream...</span>
              </div>
            )}
            <div className="camera-label">
              {cameras.find(c => c.name === activeCamera)?.label || activeCamera}
            </div>
          </div>
          
          <div className="camera-thumbnails">
            {cameras.map(camera => (
              <CameraThumbnail
                key={camera.name}
                camera={camera}
                source={cameraSources[camera.name]}
                isActive={activeCamera === camera.name}
                onClick={() => handleCameraSelect(camera.name)}
              />
            ))}
          </div>
        </>
      )}
      
      {/* Grid layout */}
      {layout === 'grid' && (
        <div className="camera-grid">
          {cameras.map(camera => (
            <div key={camera.name} className="grid-camera-view">
              {cameraSources[camera.name] ? (
                <img
                  src={cameraSources[camera.name]}
                  alt={camera.label || camera.name}
                  className="camera-image"
                />
              ) : (
                <div className="camera-placeholder">
                  <span>Loading...</span>
                </div>
              )}
              <div className="camera-label">{camera.label || camera.name}</div>
            </div>
          ))}
        </div>
      )}
      
      {/* Picture-in-picture layout */}
      {layout === 'pip' && (
        <>
          <div className="main-camera-view">
            {activeCamera && cameraSources[activeCamera] ? (
              <img
                src={cameraSources[activeCamera]}
                alt={`Camera ${activeCamera}`}
                className="camera-image"
              />
            ) : (
              <div className="camera-placeholder">
                <span>Waiting for stream...</span>
              </div>
            )}
          </div>
          
          <div className="pip-cameras">
            {cameras
              .filter(c => c.name !== activeCamera)
              .map(camera => (
                <div
                  key={camera.name}
                  className="pip-camera"
                  onClick={() => handleCameraSelect(camera.name)}
                >
                  {cameraSources[camera.name] ? (
                    <img
                      src={cameraSources[camera.name]}
                      alt={camera.label || camera.name}
                      className="camera-image"
                    />
                  ) : (
                    <div className="camera-placeholder-small">
                      <span>...</span>
                    </div>
                  )}
                  <div className="pip-label">{camera.label || camera.name}</div>
                </div>
              ))}
          </div>
        </>
      )}
    </div>
  );
};

const CameraThumbnail = ({ camera, source, isActive, onClick }) => {
  return (
    <div
      className={`camera-thumbnail ${isActive ? 'active' : ''}`}
      onClick={onClick}
    >
      {source ? (
        <img src={source} alt={camera.label || camera.name} />
      ) : (
        <div className="thumbnail-placeholder">
          <span>...</span>
        </div>
      )}
      <div className="thumbnail-label">{camera.label || camera.name}</div>
    </div>
  );
};

export default MultiCameraViewer;
