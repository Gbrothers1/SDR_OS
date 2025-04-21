import React, { useState, useEffect } from 'react';
import '../styles/CameraViewer.css';

const CameraViewer = ({ ros, topic }) => {
  const [viewerUrl, setViewerUrl] = useState(''); // URL for the web_video_server viewer page
  const [error, setError] = useState(null);
  const [isServerReachable, setIsServerReachable] = useState(false); // Check if server is accessible

  useEffect(() => {
    // Construct the URL for the web_video_server viewer page.
    // Assumes web_video_server is running and accessible.

    if (ros && ros.isConnected) {
      let hostname = 'localhost'; 
      try {
        const url = new URL(ros.socket.url);
        hostname = url.hostname;
        console.log(`CameraViewer: Detected ROS hostname: ${hostname}`);
      } catch (e) {
        console.warn('CameraViewer: Could not parse ROS hostname from URL, defaulting to localhost.');
      }

      // Construct the URL for the specific topic stream viewer page (adjust port if needed)
      const url = `http://${hostname}:8080/stream_viewer?topic=${topic}`;
      setViewerUrl(url);
      setError(null);
      
      // Check if the server is actually reachable (optional but good practice)
      fetch(`http://${hostname}:8080`) // Check base URL
        .then(response => {
          if (response.ok) {
            setIsServerReachable(true);
            console.log(`CameraViewer: Set viewer URL to ${url}`);
          } else {
            throw new Error(`Server responded with status ${response.status}`);
          }
        })
        .catch(err => {
          console.error(`CameraViewer: Error reaching web_video_server at http://${hostname}:8080`, err);
          setError(`Failed to connect to web_video_server at ${hostname}:8080. Is it running?`);
          setIsServerReachable(false);
          setViewerUrl('');
        });

    } else {
      setViewerUrl('');
      setIsServerReachable(false);
      if (ros && !ros.isConnected) {
          setError('Camera feed unavailable (ROS disconnected)');
      }
    }

  }, [ros, topic, ros?.isConnected]);

  return (
    <div className="camera-viewer">
      <div className="camera-header">DevCam Feed ({topic})</div>
      <div className="camera-content">
        {error && <div className="camera-error">{error}</div>}
        {!isServerReachable && !error && <div className="camera-loading">Waiting for video server...</div>}
        {isServerReachable && viewerUrl && (
          <iframe 
            src={viewerUrl} 
            title="Camera Feed Viewer" 
            width="100%" 
            height="100%" 
            frameBorder="0"
            scrolling="no" // Optional: hide scrollbars if viewer fits well
            onError={(e) => { // iframe onError is less reliable than fetch
              console.error(`CameraViewer: Error loading iframe content from ${viewerUrl}`);
              // Error state is likely already set by the fetch check
            }}
          />
        )}
      </div>
    </div>
  );
};

export default CameraViewer; 