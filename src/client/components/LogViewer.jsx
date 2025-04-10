import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import '../styles/LogViewer.css';

const LogViewer = ({ ros }) => {
  const [logs, setLogs] = useState([]);
  const [selectedTopic, setSelectedTopic] = useState('/rosout');
  const [availableTopics, setAvailableTopics] = useState([]);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const logContainerRef = useRef(null);

  useEffect(() => {
    if (!ros) {
      console.warn('ROS connection not available for LogViewer');
      setError('ROS connection not available');
      setIsLoading(false);
      return;
    }

    // Get list of available topics
    const getTopics = () => {
      try {
        const service = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/get_topics',
          serviceType: 'rosapi_msgs/srv/GetTopics'
        });

        const request = new ROSLIB.ServiceRequest({});
        
        service.callService(request, (result) => {
          if (result && result.topics) {
            setAvailableTopics(result.topics);
            console.log('Available topics:', result.topics);
          } else {
            console.warn('No topics available');
            setAvailableTopics([]);
          }
          setIsLoading(false);
        }, (error) => {
          console.error('Error getting topics:', error);
          setError('Failed to get available topics');
          setIsLoading(false);
        });
      } catch (err) {
        console.error('Error setting up topic service:', err);
        setError('Failed to set up topic service');
        setIsLoading(false);
      }
    };

    getTopics();
  }, [ros]);

  useEffect(() => {
    if (!ros || !selectedTopic) {
      return;
    }

    let subscriber = null;
    
    try {
      // Create a subscriber for the selected topic
      subscriber = new ROSLIB.Topic({
        ros: ros,
        name: selectedTopic,
        messageType: 'std_msgs/String' // Default message type
      });

      subscriber.subscribe((message) => {
        setLogs(prevLogs => {
          const newLog = {
            timestamp: new Date().toISOString(),
            topic: selectedTopic,
            message: JSON.stringify(message)
          };
          
          // Keep only the last 100 logs
          const updatedLogs = [...prevLogs, newLog].slice(-100);
          return updatedLogs;
        });
      });
      
      console.log(`Subscribed to topic: ${selectedTopic}`);
    } catch (err) {
      console.error(`Error subscribing to topic ${selectedTopic}:`, err);
      setError(`Failed to subscribe to topic: ${selectedTopic}`);
    }

    return () => {
      if (subscriber) {
        subscriber.unsubscribe();
        console.log(`Unsubscribed from topic: ${selectedTopic}`);
      }
    };
  }, [ros, selectedTopic]);

  // Auto-scroll to bottom when new logs are added
  useEffect(() => {
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [logs]);

  const handleTopicChange = (e) => {
    setSelectedTopic(e.target.value);
    setLogs([]); // Clear logs when changing topics
  };

  const handleClearLogs = () => {
    setLogs([]);
  };

  const formatTimestamp = (timestamp) => {
    const date = new Date(timestamp);
    return date.toLocaleTimeString();
  };

  return (
    <div className="log-viewer">
      <div className="log-viewer-header">
        <h3>Log Viewer</h3>
        <button 
          className="clear-button" 
          onClick={handleClearLogs}
          disabled={logs.length === 0}
        >
          Clear Logs
        </button>
      </div>
      
      {error && <div className="error-message">{error}</div>}
      
      <div className="topic-selector">
        <label htmlFor="topic-select">Select Topic:</label>
        <select 
          id="topic-select" 
          value={selectedTopic} 
          onChange={handleTopicChange}
          disabled={isLoading || !ros}
        >
          <option value="/rosout">/rosout</option>
          {Array.isArray(availableTopics) && availableTopics.map(topic => (
            <option key={topic} value={topic}>{topic}</option>
          ))}
        </select>
      </div>
      
      <div className="log-container" ref={logContainerRef}>
        {isLoading ? (
          <div className="loading">Loading topics...</div>
        ) : logs.length === 0 ? (
          <div className="no-logs">No logs available for this topic</div>
        ) : (
          logs.map((log, index) => (
            <div key={index} className="log-entry">
              <div className="log-header">
                <span className="log-timestamp">{formatTimestamp(log.timestamp)}</span>
                <span className="log-topic">{log.topic}</span>
              </div>
              <pre className="log-message">{log.message}</pre>
            </div>
          ))
        )}
      </div>
    </div>
  );
};

export default LogViewer; 