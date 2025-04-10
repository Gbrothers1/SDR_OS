import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';
import '../styles/LogViewer.css';

const LogViewer = ({ ros }) => {
  const [logs, setLogs] = useState([]);
  const [selectedTopic, setSelectedTopic] = useState('/rosout');
  const [availableTopics, setAvailableTopics] = useState([]);
  const [error, setError] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [rosapiAvailable, setRosapiAvailable] = useState(false);
  const logContainerRef = useRef(null);

  // Default topics with their correct message types
  const defaultTopics = [
    { name: '/rosout', type: 'rcl_interfaces/msg/Log' },
    { name: '/cmd_vel', type: 'geometry_msgs/msg/Twist' },
    { name: '/tf', type: 'tf2_msgs/msg/TFMessage' },
    { name: '/tf_static', type: 'tf2_msgs/msg/TFMessage' },
    { name: '/odom', type: 'nav_msgs/msg/Odometry' },
    { name: '/scan', type: 'sensor_msgs/msg/LaserScan' },
    { name: '/camera/image_raw', type: 'sensor_msgs/msg/Image' },
    { name: '/joint_states', type: 'sensor_msgs/msg/JointState' }
  ];

  // Function to format log level
  const getLogLevel = (level) => {
    switch (level) {
      case 10: return 'DEBUG';
      case 20: return 'INFO';
      case 30: return 'WARN';
      case 40: return 'ERROR';
      case 50: return 'FATAL';
      default: return 'UNKNOWN';
    }
  };

  // Function to format log message
  const formatLogMessage = (message) => {
    if (selectedTopic === '/rosout') {
      // For /rosout messages, show a more readable format
      return `${getLogLevel(message.level)} [${message.name}]: ${message.msg}`;
    }
    // For other topics, show the full message
    return JSON.stringify(message, null, 2);
  };

  useEffect(() => {
    if (!ros) {
      console.warn('ROS connection not available for LogViewer');
      setError('ROS connection not available');
      setIsLoading(false);
      return;
    }

    // Check if rosapi service is available
    const checkRosapiService = () => {
      try {
        const service = new ROSLIB.Service({
          ros: ros,
          name: '/rosapi/get_topics',
          serviceType: 'rosapi_msgs/srv/GetTopics'
        });
        
        const request = new ROSLIB.ServiceRequest({});
        
        service.callService(request, (result) => {
          console.log('ROSAPI service is available:', result);
          setRosapiAvailable(true);
          getTopics();
        }, (error) => {
          console.warn('ROSAPI service is not available:', error);
          setRosapiAvailable(false);
          getTopicsFallback();
        });
      } catch (err) {
        console.warn('Error checking ROSAPI service:', err);
        setRosapiAvailable(false);
        getTopicsFallback();
      }
    };

    // Get list of available topics using rosapi
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
          getTopicsFallback();
        });
      } catch (err) {
        console.error('Error setting up topic service:', err);
        setError('Failed to set up topic service');
        setIsLoading(false);
        getTopicsFallback();
      }
    };

    // Fallback mechanism to get topics
    const getTopicsFallback = () => {
      console.log('Using fallback mechanism to get topics');
      setAvailableTopics(defaultTopics.map(t => t.name));
      setIsLoading(false);
      
      // Try to subscribe to each topic to verify it exists
      defaultTopics.forEach(topic => {
        try {
          const subscriber = new ROSLIB.Topic({
            ros: ros,
            name: topic.name,
            messageType: topic.type
          });
          
          subscriber.subscribe(() => {
            // Topic exists, do nothing
          });
          
          // Unsubscribe after a short delay
          setTimeout(() => {
            subscriber.unsubscribe();
          }, 1000);
        } catch (err) {
          console.warn(`Topic ${topic.name} does not exist:`, err);
        }
      });
    };

    // Check if rosapi service is available
    checkRosapiService();
  }, [ros]);

  useEffect(() => {
    if (!ros || !selectedTopic) {
      return;
    }

    let subscriber = null;
    
    try {
      // Find the correct message type for the selected topic
      const topicInfo = defaultTopics.find(t => t.name === selectedTopic);
      const messageType = topicInfo ? topicInfo.type : 'std_msgs/msg/String';
      
      // Create a subscriber for the selected topic
      subscriber = new ROSLIB.Topic({
        ros: ros,
        name: selectedTopic,
        messageType: messageType
      });

      subscriber.subscribe((message) => {
        setLogs(prevLogs => {
          const newLog = {
            timestamp: new Date().toISOString(),
            topic: selectedTopic,
            message: formatLogMessage(message)
          };
          
          // Keep only the last 100 logs
          const updatedLogs = [...prevLogs, newLog].slice(-100);
          return updatedLogs;
        });
      });
      
      console.log(`Subscribed to topic: ${selectedTopic} with message type: ${messageType}`);
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
          {defaultTopics.map(topic => (
            <option key={topic.name} value={topic.name}>{topic.name}</option>
          ))}
        </select>
        {!rosapiAvailable && (
          <div className="warning-message">
            Using default topics (ROSAPI not available)
          </div>
        )}
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