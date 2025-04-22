import React, { useState, useEffect, useRef, memo } from 'react';
import '../styles/CameraViewer.css';
import ROSLIB from 'roslib';

// Memoize the component to prevent unnecessary renders
const CameraViewer = memo(({ ros, topic, host = localStorage.getItem('webVideoHost') || 'localhost' }) => {
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
  const [lastActivityTime, setLastActivityTime] = useState(Date.now());
  
  const availableCameras = [
    { id: '/webcam/image_raw', name: 'Front Camera' },
  ];

  const viewerRef = useRef(null);
  const iframeRef = useRef(null);
  const subscribedRef = useRef(false);

  // Function to construct the stream URL
  const constructStreamUrl = () => {
    const hostname = currentHost;
    const port = '8080';
    const baseUrl = `http://${hostname}:${port}`;
    // Use stream_viewer endpoint for iframe
    const url = `${baseUrl}/stream_viewer?topic=${selectedCamera}`;
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
      // Update activity timestamp
      setLastActivityTime(Date.now());
      
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
    // Don't do anything if minimized
    if (isMinimized) {
      setStreamUrl('');
      return;
    }
    
    setIsLoading(true);
    setError(null);
    const url = constructStreamUrl();
    setStreamUrl(url);
    
    // Prevent duplicate subscriptions
    if (subscribedRef.current) {
      return;
    }

    // Create a camera subscriber with lower QoS
    let cameraSubscriber = null;
    if (ros && selectedCamera) {
      try {
        // Use a unique clientId to avoid conflicts with other viewers
        const clientId = 'cam_viewer_' + Date.now().toString(36);
        
        cameraSubscriber = new ROSLIB.Topic({
          ros: ros,
          name: selectedCamera,
          messageType: 'sensor_msgs/msg/Image',
          qos: {
            reliability: 'reliable',
            durability: 'volatile',
            history: 'keep_last',
            depth: 1
          }
        });

        // Use a debounced handler to avoid overwhelming the UI
        // Also only call it once instead of on every message
        const debouncedHandler = debounce(() => {
          // After confirming subscription, unsubscribe right away
          // This allows web_video_server to get the topic without us receiving the images
          try {
            cameraSubscriber.unsubscribe();
            subscribedRef.current = true;
          } catch (err) {
            // Silent catch
          }
        }, 500);

        cameraSubscriber.subscribe(debouncedHandler);
      } catch (err) {
        console.error(`Error subscribing to ${selectedCamera}:`, err);
        setError(`Failed to subscribe to camera topic: ${err.message}`);
      }
    }

    return () => {
      // Clear the stream URL
      setStreamUrl('');
      // Reset subscription flag on cleanup
      subscribedRef.current = false;
      // Unsubscribe from the camera topic if it exists
      if (cameraSubscriber) {
        try {
          cameraSubscriber.unsubscribe();
        } catch (err) {
          // Silent catch
        }
      }
    };
  }, [selectedCamera, currentHost, isMinimized, ros]);

  // Add debounce utility
  const debounce = (func, wait) => {
    let timeout;
    return function executedFunction(...args) {
      const later = () => {
        clearTimeout(timeout);
        func(...args);
      };
      clearTimeout(timeout);
      timeout = setTimeout(later, wait);
    };
  };

  // Handle iframe load errors
  const handleIframeError = () => {
    setError('Failed to load camera stream. Please check if the camera topic is being published.');
    setIsLoading(false);
  };

  // Handle iframe load success
  const handleIframeLoad = () => {
    setError(null);
    setIsLoading(false);
  };

  const handleMouseDown = (e) => {
    if (e.target.classList.contains('camera-header')) {
      // Update activity timestamp
      setLastActivityTime(Date.now());
      
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
    
    // Update activity timestamp
    setLastActivityTime(Date.now());
    
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
    // Update activity timestamp
    setLastActivityTime(Date.now());
    
    const newState = !isMinimized;
    setIsMinimized(newState);
    if (newState) {
      // If minimizing, clear stream to unsubscribe
      setStreamUrl('');
      subscribedRef.current = false;
    } else {
      // Restoring viewer, rebuild URL
      const url = constructStreamUrl();
      setStreamUrl(url);
      setIsLoading(true);
    }
  };

  const handleCameraChange = (e) => {
    // Update activity timestamp
    setLastActivityTime(Date.now());
    
    setSelectedCamera(e.target.value);
    subscribedRef.current = false;
  };

  // CPU saving - reduce iframe refresh rate when inactive
  useEffect(() => {
    const lowPowerMode = () => {
      // Check if camera has been inactive for more than 30 seconds
      const now = Date.now();
      const inactiveFor = now - lastActivityTime;
      
      if (inactiveFor > 30000 && iframeRef.current && !isMinimized) {
        // Temporarily hide iframe to save CPU
        if (iframeRef.current.style.opacity !== '0.5') {
          iframeRef.current.style.opacity = '0.5';
          iframeRef.current.style.filter = 'grayscale(50%)';
        }
      } else if (iframeRef.current) {
        // Restore normal display
        iframeRef.current.style.opacity = '1';
        iframeRef.current.style.filter = 'none';
      }
    };
    
    const intervalId = setInterval(lowPowerMode, 5000);
    
    return () => clearInterval(intervalId);
  }, [lastActivityTime, isMinimized]);

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
      onMouseMove={() => setLastActivityTime(Date.now())}
    >
      <div className="camera-border"></div>
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
});

export default CameraViewer; 