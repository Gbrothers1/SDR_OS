import React, { useEffect, useCallback } from 'react';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import PolicyLabBrowser from './PolicyLabBrowser';
import '../styles/TrainingPanel.css';

export default function TrainingPanel() {
  const { isOpen, closePanel } = useTrainingPanel();

  const handleKeyDown = useCallback((e) => {
    if (!isOpen) return;
    if (e.key === 'Escape') {
      e.preventDefault();
      closePanel();
    }
  }, [isOpen, closePanel]);

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  return (
    <div className={`training-panel ${isOpen ? 'training-panel--open' : ''}`}>
      <div className="training-panel__header">
        <div>
          <h1 className="training-panel__title">Policy Lab</h1>
          <p>Inspect checkpoints and episode videos before loading a policy into the simulator.</p>
        </div>
        <button className="training-panel__close" onClick={closePanel} aria-label="Close Policy Lab">
          Close
        </button>
      </div>
      <PolicyLabBrowser active={isOpen} />
    </div>
  );
}
