import React, { useState, useEffect, useRef } from 'react';
import ROSLIB from 'roslib';

const LogViewer = ({ ros }) => {
  const [logs, setLogs] = useState([]);
  const [selectedTopic, setSelectedTopic] = useState('');
  const [availableTopics, setAvailableTopics] = useState([]);
  const logContainerRef = useRef();

  useEffect(() => {
    if (!ros) return;

    // Get list of available topics
    ros.getTopics((topics) => {
      setAvailableTopics(topics);
      if (topics.length > 0 && !selectedTopic) {
        setSelectedTopic(topics[0]);
      }
    });

    // Subscribe to rosout for system logs
    const rosout = new ROSLIB.Topic({
      ros: ros,
      name: '/rosout',
      messageType: 'rosgraph_msgs/Log'
    });

    rosout.subscribe((message) => {
      addLog({
        time: new Date(message.header.stamp.secs * 1000),
        level: message.level,
        name: message.name,
        msg: message.msg
      });
    });

    return () => {
      rosout.unsubscribe();
    };
  }, [ros]);

  useEffect(() => {
    if (!ros || !selectedTopic) return;

    // Subscribe to selected topic
    const topic = new ROSLIB.Topic({
      ros: ros,
      name: selectedTopic,
      messageType: 'std_msgs/String' // This should be determined dynamically
    });

    topic.subscribe((message) => {
      addLog({
        time: new Date(),
        level: 'info',
        name: selectedTopic,
        msg: JSON.stringify(message)
      });
    });

    return () => {
      topic.unsubscribe();
    };
  }, [ros, selectedTopic]);

  const addLog = (log) => {
    setLogs((prevLogs) => {
      const newLogs = [...prevLogs, log].slice(-100); // Keep last 100 logs
      return newLogs;
    });

    // Auto-scroll to bottom
    if (logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  };

  const getLevelStyle = (level) => {
    switch (level) {
      case 1: // DEBUG
        return { color: '#888' };
      case 2: // INFO
        return { color: '#fff' };
      case 4: // WARN
        return { color: '#ff8' };
      case 8: // ERROR
        return { color: '#f88' };
      case 16: // FATAL
        return { color: '#f00' };
      default:
        return { color: '#fff' };
    }
  };

  return (
    <div style={styles.container}>
      <div style={styles.header}>
        <select
          value={selectedTopic}
          onChange={(e) => setSelectedTopic(e.target.value)}
          style={styles.topicSelect}
        >
          {availableTopics.map((topic) => (
            <option key={topic} value={topic}>
              {topic}
            </option>
          ))}
        </select>
        <button
          onClick={() => setLogs([])}
          style={styles.clearButton}
        >
          Clear
        </button>
      </div>
      <div ref={logContainerRef} style={styles.logContainer}>
        {logs.map((log, index) => (
          <div key={index} style={styles.logEntry}>
            <span style={styles.timestamp}>
              {log.time.toLocaleTimeString()}
            </span>
            <span style={getLevelStyle(log.level)}>
              [{log.name}] {log.msg}
            </span>
          </div>
        ))}
      </div>
    </div>
  );
};

const styles = {
  container: {
    display: 'flex',
    flexDirection: 'column',
    height: '100%',
    background: '#1e1e1e',
  },
  header: {
    padding: 10,
    borderBottom: '1px solid #333',
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
  },
  topicSelect: {
    background: '#333',
    color: '#fff',
    border: 'none',
    padding: '5px 10px',
    borderRadius: 3,
  },
  clearButton: {
    background: '#444',
    color: '#fff',
    border: 'none',
    padding: '5px 10px',
    borderRadius: 3,
    cursor: 'pointer',
  },
  logContainer: {
    flex: 1,
    overflowY: 'auto',
    padding: 10,
    fontFamily: 'monospace',
    fontSize: 12,
  },
  logEntry: {
    marginBottom: 2,
    whiteSpace: 'pre-wrap',
    wordBreak: 'break-all',
  },
  timestamp: {
    color: '#666',
    marginRight: 10,
  },
};

export default LogViewer; 