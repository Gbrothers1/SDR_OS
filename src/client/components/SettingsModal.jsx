import React, { useState, useEffect } from 'react';
import '../styles/SettingsModal.css';

const SettingsModal = ({ isOpen, onClose, onSave, initialSettings }) => {
  const [settings, setSettings] = useState({
    rosbridgeUrl: localStorage.getItem('rosBridgeUrl') || 'ws://localhost:9090',
    socketUrl: localStorage.getItem('socketUrl') || 'http://localhost:3000',
    enableSpeech: localStorage.getItem('enableSpeech') !== 'false',
    selectedVoice: localStorage.getItem('selectedVoice') || ''
  });

  const [availableVoices, setAvailableVoices] = useState([]);

  useEffect(() => {
    if ('speechSynthesis' in window) {
      const loadVoices = () => {
        const voices = window.speechSynthesis.getVoices();
        // Filter for English voices only
        const englishVoices = voices.filter(voice => voice.lang.startsWith('en-'));
        setAvailableVoices(englishVoices);

        // Set default voice if not already set
        if (!settings.selectedVoice && englishVoices.length > 0) {
          const defaultVoice = englishVoices.find(voice => voice.name.includes('female')) || englishVoices[0];
          setSettings(prev => ({ ...prev, selectedVoice: defaultVoice.name }));
        }
      };

      loadVoices();
      window.speechSynthesis.onvoiceschanged = loadVoices;
    }
  }, []);

  const handleChange = (e) => {
    const { name, value, type, checked } = e.target;
    setSettings(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleSave = () => {
    // Save connection settings
    localStorage.setItem('rosBridgeUrl', settings.rosbridgeUrl);
    localStorage.setItem('socketUrl', settings.socketUrl);
    
    // Save speech settings
    localStorage.setItem('enableSpeech', settings.enableSpeech);
    localStorage.setItem('selectedVoice', settings.selectedVoice);
    
    // Call the onSave callback with all settings
    if (onSave) {
      onSave(settings);
    }
    
    // Close the modal
    onClose();
  };

  const testVoice = () => {
    if ('speechSynthesis' in window && settings.selectedVoice) {
      const utterance = new SpeechSynthesisUtterance('This is a test of the selected voice.');
      const voice = availableVoices.find(v => v.name === settings.selectedVoice);
      if (voice) {
        utterance.voice = voice;
        window.speechSynthesis.speak(utterance);
      }
    }
  };

  if (!isOpen) return null;

  return (
    <div className="settings-modal">
      <div className="settings-content">
        <h2>Settings</h2>
        
        <div className="settings-section">
          <h3>Connection Settings</h3>
          <div className="form-group">
            <label htmlFor="rosbridgeUrl">ROS Bridge URL</label>
            <input
              type="text"
              id="rosbridgeUrl"
              name="rosbridgeUrl"
              value={settings.rosbridgeUrl}
              onChange={handleChange}
            />
          </div>
          <div className="form-group">
            <label htmlFor="socketUrl">Socket.IO URL</label>
            <input
              type="text"
              id="socketUrl"
              name="socketUrl"
              value={settings.socketUrl}
              onChange={handleChange}
            />
          </div>
        </div>

        <div className="settings-section">
          <h3>Audio Settings</h3>
          <div className="form-group">
            <div className="checkbox-group">
              <input
                type="checkbox"
                id="enableSpeech"
                name="enableSpeech"
                checked={settings.enableSpeech}
                onChange={handleChange}
              />
              <label htmlFor="enableSpeech">Enable Voice Feedback</label>
            </div>
          </div>
          
          {settings.enableSpeech && (
            <div className="form-group">
              <label htmlFor="selectedVoice">Select Voice</label>
              <div className="voice-select-container">
                <select
                  id="selectedVoice"
                  name="selectedVoice"
                  value={settings.selectedVoice}
                  onChange={handleChange}
                >
                  {availableVoices.map(voice => (
                    <option key={voice.name} value={voice.name}>
                      {voice.name.replace('en-', '')}
                    </option>
                  ))}
                </select>
                <button className="test-voice-button" onClick={testVoice}>
                  Test Voice
                </button>
              </div>
            </div>
          )}
        </div>

        <div className="button-group">
          <button className="cancel-button" onClick={onClose}>
            Cancel
          </button>
          <button className="save-button" onClick={handleSave}>
            Save Changes
          </button>
        </div>
      </div>
    </div>
  );
};

export default SettingsModal; 