import React, { useState } from 'react';
import '../styles/SettingsModal.css';

function SettingsModal({ onClose }) {
  const [settings, setSettings] = useState({
    rosBridgeUrl: localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090',
    socketUrl: localStorage.getItem('socketUrl') || 'http://localhost:3000'
  });

  const handleChange = (e) => {
    const { name, value } = e.target;
    setSettings(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = (e) => {
    e.preventDefault();
    localStorage.setItem('rosBridgeUrl', settings.rosBridgeUrl);
    localStorage.setItem('socketUrl', settings.socketUrl);
    onClose();
    window.location.reload(); // Reload to apply new settings
  };

  return (
    <div className="settings-modal-overlay">
      <div className="settings-modal">
        <h2>Connection Settings</h2>
        <form onSubmit={handleSubmit}>
          <div className="form-group">
            <label htmlFor="rosBridgeUrl">ROS Bridge URL:</label>
            <input
              type="text"
              id="rosBridgeUrl"
              name="rosBridgeUrl"
              value={settings.rosBridgeUrl}
              onChange={handleChange}
              placeholder="ws://localhost:9090"
            />
          </div>
          <div className="form-group">
            <label htmlFor="socketUrl">Socket URL:</label>
            <input
              type="text"
              id="socketUrl"
              name="socketUrl"
              value={settings.socketUrl}
              onChange={handleChange}
              placeholder="http://localhost:3000"
            />
          </div>
          <div className="button-group">
            <button type="submit" className="save-button">Save</button>
            <button type="button" className="cancel-button" onClick={onClose}>Cancel</button>
          </div>
        </form>
      </div>
    </div>
  );
}

export default SettingsModal; 