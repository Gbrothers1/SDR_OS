import React, { useState, useEffect } from 'react';
import Settings from './Settings';
import '../styles/SettingsModal.css';

const SettingsModal = ({ isOpen, onClose, onSave }) => {
  // Load saved settings from localStorage
  const loadSavedSettings = () => {
    const savedSettings = localStorage.getItem('robotControllerSettings');
    if (savedSettings) {
      return JSON.parse(savedSettings);
    }
    return null;
  };

  const handleSettingsSave = (settings) => {
    // Call the onSave callback with the settings
    if (onSave) {
      onSave(settings);
    }
  };

  if (!isOpen) return null;

  return (
    <div className="settings-modal-overlay">
      <Settings 
        isOpen={isOpen} 
        onClose={onClose} 
        onSave={handleSettingsSave}
        initialSettings={loadSavedSettings()}
      />
    </div>
  );
};

export default SettingsModal; 