import React, { useEffect, useCallback } from 'react';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import HotRow from './HotRow';
import SessionCarousel from './SessionCarousel';
import '../styles/TrainingPanel.css';

export default function TrainingPanel() {
  const { isOpen, focusZone, navigateFocus, closePanel } = useTrainingPanel();

  // Keyboard navigation
  const handleKeyDown = useCallback((e) => {
    if (!isOpen) return;

    switch (e.key) {
      case 'ArrowUp':
        e.preventDefault();
        navigateFocus('up');
        break;
      case 'ArrowDown':
        e.preventDefault();
        navigateFocus('down');
        break;
      case 'Escape':
        e.preventDefault();
        closePanel();
        break;
      default:
        break;
    }
  }, [isOpen, focusZone, navigateFocus]);

  useEffect(() => {
    window.addEventListener('keydown', handleKeyDown);
    return () => window.removeEventListener('keydown', handleKeyDown);
  }, [handleKeyDown]);

  // Footer hints based on focus zone
  const getFooterHints = () => {
    switch (focusZone) {
      case 'carousel':
        return [
          { key: 'A', action: 'Select' },
          { key: 'Y', action: 'Peek' },
          { key: 'L2', action: 'Clone' },
        ];
      case 'hotrow':
        return [
          { key: 'A', action: 'Edit' },
          { key: 'Y', action: 'Reset' },
          { key: 'L1', action: 'Fine' },
        ];
      default:
        return [];
    }
  };

  const footerHints = getFooterHints();

  return (
    <div className={`training-panel ${isOpen ? 'training-panel--open' : ''}`}>
      {/* PiP Viewer */}
      <div className="training-panel__pip">
        Episode Preview
      </div>

      {/* Main Content */}
      <div className="training-panel__content">
        {/* Carousel Area */}
        <div className="training-panel__carousel">
          <SessionCarousel />
        </div>

        {/* Hot Row */}
        <HotRow />
      </div>

      {/* Footer with context-aware hints */}
      <div className="training-panel__footer">
        <span style={{ marginRight: 'auto', color: 'rgba(255,255,255,0.3)' }}>
          Focus: {focusZone.charAt(0).toUpperCase() + focusZone.slice(1)}
        </span>
        {footerHints.map(({ key, action }) => (
          <div key={key} className="training-panel__footer-hint">
            <span className="training-panel__footer-key">{key}</span>
            <span>{action}</span>
          </div>
        ))}
      </div>
    </div>
  );
}
