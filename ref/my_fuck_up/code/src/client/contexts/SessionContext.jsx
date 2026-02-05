import React, { createContext, useContext, useState, useCallback } from 'react';
import { useGenesis } from './GenesisContext';

const SessionContext = createContext(null);

// Default session structure
const createDefaultSession = (id, robot = 'Go2') => ({
  id,
  name: `${robot} Training`,
  robot,
  type: 'RL Training', // 'RL Training' | 'Env Capture'
  phase: 'training', // 'teleop' | 'training' | 'slam'
  preset: 'Locomotion', // 'Sit/Stand' | 'Locomotion' | 'Transfer'
  status: 'idle', // 'idle' | 'running' | 'paused' | 'done'
  environment: 'Flat Ground',
  overrides: {},
  createdAt: new Date().toISOString(),
  lastRun: null,
});

export function SessionProvider({ children }) {
  const { socket, genesisConnected } = useGenesis();
  const [sessions, setSessions] = useState([]);
  const [activeSessionId, setActiveSessionId] = useState(null);

  // Get active session
  const activeSession = sessions.find(s => s.id === activeSessionId) || null;

  // Create new session
  const createSession = useCallback((robot, type, preset, name) => {
    const id = `session_${Date.now()}`;
    const session = {
      ...createDefaultSession(id, robot),
      type,
      preset,
      name: name || `${robot} ${preset}`,
    };
    setSessions(prev => [...prev, session]);
    setActiveSessionId(id);

    // Notify backend
    if (socket && genesisConnected) {
      socket.emit('genesis_create_session', { session });
    }

    return session;
  }, [socket, genesisConnected]);

  // Clone session
  const cloneSession = useCallback((sessionId) => {
    const source = sessions.find(s => s.id === sessionId);
    if (!source) return null;

    const id = `session_${Date.now()}`;
    const now = new Date();
    const timestamp = `${(now.getMonth() + 1).toString().padStart(2, '0')}-${now.getDate().toString().padStart(2, '0')} ${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}`;

    const cloned = {
      ...source,
      id,
      name: `${source.name} (clone) ${timestamp}`,
      status: 'idle',
      createdAt: now.toISOString(),
      lastRun: null,
    };

    setSessions(prev => [...prev, cloned]);
    setActiveSessionId(id);

    if (socket && genesisConnected) {
      socket.emit('genesis_clone_session', { sourceId: sessionId, session: cloned });
    }

    return cloned;
  }, [sessions, socket, genesisConnected]);

  // Delete session
  const deleteSession = useCallback((sessionId) => {
    setSessions(prev => prev.filter(s => s.id !== sessionId));
    if (activeSessionId === sessionId) {
      setActiveSessionId(null);
    }

    if (socket && genesisConnected) {
      socket.emit('genesis_delete_session', { sessionId });
    }
  }, [activeSessionId, socket, genesisConnected]);

  // Update session param
  const updateSessionParam = useCallback((sessionId, paramPath, value) => {
    setSessions(prev => prev.map(s => {
      if (s.id !== sessionId) return s;
      return {
        ...s,
        overrides: {
          ...s.overrides,
          [paramPath]: value,
        },
      };
    }));

    if (socket && genesisConnected) {
      socket.emit('genesis_set_param', { sessionId, paramPath, value });
    }
  }, [socket, genesisConnected]);

  const value = {
    sessions,
    activeSession,
    activeSessionId,
    setActiveSessionId,
    createSession,
    cloneSession,
    deleteSession,
    updateSessionParam,
  };

  return (
    <SessionContext.Provider value={value}>
      {children}
    </SessionContext.Provider>
  );
}

export function useSession() {
  const context = useContext(SessionContext);
  if (!context) {
    throw new Error('useSession must be used within SessionProvider');
  }
  return context;
}
