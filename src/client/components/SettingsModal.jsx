import React from 'react';
import Settings from './Settings';

const SettingsModal = ({ isOpen, onClose, onSave, initialSettings }) => {
  // Just pass through to the Settings component
  return (
    <Settings 
      isOpen={isOpen} 
      onClose={onClose} 
      onSave={onSave}
      initialSettings={initialSettings}
    />
  );
};

export default SettingsModal; 