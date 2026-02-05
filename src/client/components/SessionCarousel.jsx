import React, { useState, useEffect } from 'react';
import SessionCard from './SessionCard';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import { useSession } from '../contexts/SessionContext';
import '../styles/SessionCarousel.css';

// Format time ago helper
function formatTimeAgo(isoString) {
  if (!isoString) return 'Never';
  const date = new Date(isoString);
  const now = new Date();
  const diffMs = now - date;
  const diffMins = Math.floor(diffMs / 60000);
  if (diffMins < 1) return 'Just now';
  if (diffMins < 60) return `${diffMins}m ago`;
  const diffHours = Math.floor(diffMins / 60);
  if (diffHours < 24) return `${diffHours}h ago`;
  const diffDays = Math.floor(diffHours / 24);
  return `${diffDays}d ago`;
}

export default function SessionCarousel() {
  const { focusZone, focusedSessionId, setFocusedSessionId } = useTrainingPanel();
  const { sessions, createSession } = useSession();
  const [focusedIndex, setFocusedIndex] = useState(0);

  const isFocused = focusZone === 'carousel';

  // Convert sessions to card format
  const sessionCards = sessions.map(s => ({
    id: s.id,
    name: s.name,
    robot: s.robot,
    type: s.type,
    preset: s.preset,
    status: s.status,
    lastRun: formatTimeAgo(s.lastRun),
  }));

  const allItems = [...sessionCards, { id: 'new', isNew: true }];

  // Sync focusedIndex when focusedSessionId changes externally
  useEffect(() => {
    const index = allItems.findIndex(s => s.id === focusedSessionId);
    if (index >= 0 && index !== focusedIndex) {
      setFocusedIndex(index);
    }
  }, [focusedSessionId, allItems]);

  const handleCardClick = (index, item) => {
    setFocusedIndex(index);
    if (item.isNew) {
      // Create a new session with defaults
      const newSession = createSession('Go2', 'RL Training', 'Locomotion');
      if (newSession) {
        setFocusedSessionId(newSession.id);
      }
    } else {
      setFocusedSessionId(item.id);
    }
  };

  return (
    <div className={`session-carousel ${isFocused ? 'session-carousel--focused' : ''}`}>
      {allItems.map((session, index) => (
        <SessionCard
          key={session.id}
          session={session.isNew ? null : session}
          isNew={session.isNew}
          focused={isFocused && focusedSessionId === session.id}
          onClick={() => handleCardClick(index, session)}
        />
      ))}
    </div>
  );
}
