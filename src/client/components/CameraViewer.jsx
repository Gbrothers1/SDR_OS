import React, { useState, useEffect, useRef } from 'react';
import '../styles/CameraViewer.css';

const CameraViewer = ({ ros, topic, host = localStorage.getItem('webVideoHost') || 'localhost' }) => {
  const [error, setError] = useState(null);
  const [isDragging, setIsDragging] = useState(false);
  const [isResizing, setIsResizing] = useState(false);
  const [position, setPosition] = useState({ x: 20, y: 20 });
  const [size, setSize] = useState({ width: 640, height: 480 });
  const [dragOffset, setDragOffset] = useState({ x: 0, y: 0 });
  const [resizeStart, setResizeStart] = useState({ x: 0, y: 0, width: 0, height: 0 });
  const [isMinimized, setIsMinimized] = useState(false);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [selectedCamera, setSelectedCamera] = useState(topic);
  const [currentHost, setCurrentHost] = useState(host);
  const [streamUrl, setStreamUrl] = useState('');
  const [isLoading, setIsLoading] = useState(true);
  
  const availableCameras = [
    { id: '/webcam/image_raw', name: 'Front Camera' },
  ];

  const viewerRef = useRef(null);
  const iframeRef = useRef(null);

  // Function to construct the stream URL
  const constructStreamUrl = () => {
    const hostname = currentHost;
    const port = '8080';
    const baseUrl = `http://${hostname}:${port}`;
    // Use stream_viewer endpoint for iframe
    const url = `${baseUrl}/stream_viewer?topic=${selectedCamera}`;
    console.log(`CameraViewer: Constructed stream URL: ${url}`);
    return url;
  };

  // Handle fullscreen changes
  useEffect(() => {
    const handleFullscreenChange = () => {
      setIsFullscreen(document.fullscreenElement === iframeRef.current);
    };

    document.addEventListener('fullscreenchange', handleFullscreenChange);
    return () => {
      document.removeEventListener('fullscreenchange', handleFullscreenChange);
    };
  }, []);

  const toggleFullscreen = async () => {
    try {
      if (!isFullscreen) {
        await iframeRef.current.requestFullscreen();
      } else {
        await document.exitFullscreen();
      }
    } catch (err) {
      console.error('Error toggling fullscreen:', err);
    }
  };

  // Set up the stream
  useEffect(() => {
    if (isMinimized) return; // Do nothing if minimized
    setIsLoading(true);
    setError(null);
    const url = constructStreamUrl();
    setStreamUrl(url);
    console.log(`CameraViewer: Setting up stream for ${selectedCamera} on host ${currentHost}`);

    return () => {
      console.log(`CameraViewer: Cleaning up stream for ${selectedCamera}`);
    };
  }, [selectedCamera, currentHost, isMinimized]);

  // Handle iframe load errors
  const handleIframeError = () => {
    console.error(`CameraViewer: Error loading iframe from ${streamUrl}`);
    setError('Failed to load camera stream. Please check if the camera topic is being published.');
    setIsLoading(false);
  };

  // Handle iframe load success
  const handleIframeLoad = () => {
    console.log('CameraViewer: Iframe loaded successfully');
    setError(null);
    setIsLoading(false);
  };

  const handleMouseDown = (e) => {
    if (e.target.classList.contains('camera-header')) {
      setIsDragging(true);
      const rect = viewerRef.current.getBoundingClientRect();
      setDragOffset({
        x: e.clientX - rect.left,
        y: e.clientY - rect.top
      });
    }
  };

  const handleResizeStart = (e) => {
    e.preventDefault();
    e.stopPropagation();
    setIsResizing(true);
    const rect = viewerRef.current.getBoundingClientRect();
    setResizeStart({
      x: e.clientX,
      y: e.clientY,
      width: rect.width,
      height: rect.height
    });
  };

  const handleMouseMove = (e) => {
    if (isDragging) {
      setPosition({
        x: e.clientX - dragOffset.x,
        y: e.clientY - dragOffset.y
      });
    } else if (isResizing) {
      const deltaX = e.clientX - resizeStart.x;
      const deltaY = e.clientY - resizeStart.y;
      
      // Calculate new size with minimum constraints
      const newWidth = Math.max(320, resizeStart.width + deltaX);
      const newHeight = Math.max(240, resizeStart.height + deltaY);
      
      setSize({
        width: newWidth,
        height: newHeight
      });
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
    setIsResizing(false);
  };

  useEffect(() => {
    if (isDragging || isResizing) {
      window.addEventListener('mousemove', handleMouseMove);
      window.addEventListener('mouseup', handleMouseUp);
    }
    return () => {
      window.removeEventListener('mousemove', handleMouseMove);
      window.removeEventListener('mouseup', handleMouseUp);
    };
  }, [isDragging, isResizing, dragOffset, resizeStart]);

  const toggleMinimize = () => {
    const newState = !isMinimized;
    setIsMinimized(newState);
    if (newState) {
      // If minimizing, clear stream to unsubscribe
      setStreamUrl('');
    } else {
      // Restoring viewer, rebuild URL
      const url = constructStreamUrl();
      setStreamUrl(url);
      setIsLoading(true);
    }
  };

  const handleCameraChange = (e) => {
    setSelectedCamera(e.target.value);
  };

  return (
    <div
      className={`camera-viewer ${isMinimized ? 'minimized' : ''}`}
      ref={viewerRef}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
        cursor: isDragging ? 'grabbing' : 'default',
        width: `${size.width}px`,
        height: isMinimized ? 'auto' : `${size.height + 40}px`, // Add header height
      }}
      onMouseDown={handleMouseDown}
    >
      <div className="camera-header">
        <span>Camera Feed</span>
        <div className="camera-controls">
          <select
            value={selectedCamera}
            onChange={handleCameraChange}
            onClick={(e) => e.stopPropagation()}
          >
            {availableCameras.map(camera => (
              <option key={camera.id} value={camera.id}>
                {camera.name}
              </option>
            ))}
          </select>
          <button 
            onClick={toggleFullscreen} 
            className="fullscreen-button"
            title={isFullscreen ? "Exit Fullscreen" : "Fullscreen"}
          >
            {isFullscreen ? '⤓' : '⤢'}
          </button>
          <button 
            onClick={toggleMinimize} 
            className="minimize-button"
            title={isMinimized ? "Restore" : "Minimize"}
          >
            {isMinimized ? '□' : '−'}
          </button>
        </div>
      </div>
      {!isMinimized && (
        <div className="camera-content" style={{ height: `${size.height}px` }}>
          {error && <div className="camera-error">{error}</div>}
          {isLoading && !error && <div className="camera-loading">Loading camera stream...</div>}
          {streamUrl && (
            <iframe
              ref={iframeRef}
              src={streamUrl}
              title="Camera Feed"
              onError={handleIframeError}
              onLoad={handleIframeLoad}
              frameBorder="0"
              scrolling="no"
              allow="autoplay; fullscreen"
              style={{
                width: '100%',
                height: '100%',
                border: 'none',
                display: isLoading ? 'none' : 'block'
              }}
            />
          )}
          <div 
            className="resize-handle"
            onMouseDown={handleResizeStart}
          />
        </div>
      )}
    </div>
  );
};

export default CameraViewer; 