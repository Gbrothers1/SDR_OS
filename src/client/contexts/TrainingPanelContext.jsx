import React, { createContext, useContext, useState, useCallback } from 'react';

const TrainingPanelContext = createContext(null);

export function TrainingPanelProvider({ children }) {
  // Panel state
  const [isOpen, setIsOpen] = useState(false);
  const [focusedSessionId, setFocusedSessionId] = useState(null);
  const [focusZone, setFocusZone] = useState('carousel'); // 'carousel' | 'hotrow' | 'advanced'
  const [editingParam, setEditingParam] = useState(null);

  // Toggle panel open/close
  const togglePanel = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  // Open panel
  const openPanel = useCallback(() => {
    setIsOpen(true);
  }, []);

  // Close panel
  const closePanel = useCallback(() => {
    setIsOpen(false);
    setEditingParam(null);
  }, []);

  // Focus navigation
  const navigateFocus = useCallback((direction) => {
    if (direction === 'up') {
      setFocusZone(prev => {
        if (prev === 'carousel') return 'hotrow';
        if (prev === 'hotrow') return 'advanced';
        return prev;
      });
    } else if (direction === 'down') {
      setFocusZone(prev => {
        if (prev === 'advanced') return 'hotrow';
        if (prev === 'hotrow') return 'carousel';
        return prev;
      });
    }
  }, []);

  const value = {
    // State
    isOpen,
    focusedSessionId,
    focusZone,
    editingParam,
    // Actions
    togglePanel,
    openPanel,
    closePanel,
    setFocusedSessionId,
    setFocusZone,
    setEditingParam,
    navigateFocus,
  };

  return (
    <TrainingPanelContext.Provider value={value}>
      {children}
    </TrainingPanelContext.Provider>
  );
}

export function useTrainingPanel() {
  const context = useContext(TrainingPanelContext);
  if (!context) {
    throw new Error('useTrainingPanel must be used within TrainingPanelProvider');
  }
  return context;
}
