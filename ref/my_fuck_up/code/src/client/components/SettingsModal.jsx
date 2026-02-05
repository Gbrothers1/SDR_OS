import React from 'react';
import Settings from './Settings';

const SettingsModal = ({ isOpen, onClose, onSave, initialSettings }) => {
  // Load saved settings from localStorage if not provided
  const loadSavedSettings = () => {
    if (initialSettings) {
      return initialSettings;
    }
    
    const savedSettings = localStorage.getItem('robotControllerSettings');
    if (savedSettings) {
      try {
        return JSON.parse(savedSettings);
      } catch (e) {
        console.error('Failed to parse saved settings:', e);
      }
    }
    return null;
  };

  // Just pass through to the Settings component
  return (
    <Settings 
      isOpen={isOpen} 
      onClose={onClose} 
      onSave={onSave}
      initialSettings={loadSavedSettings()}
    />
  );
};

export default SettingsModal; 