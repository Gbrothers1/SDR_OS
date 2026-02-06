import React, { useState, useCallback } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/GoalComposer.css';

const GoalComposer = () => {
  const { genesisMode } = useGenesis();
  const [socket, setSocket] = useState(null);
  
  // Position state
  const [position, setPosition] = useState({ x: 0, y: 0, z: 0 });
  
  // Orientation state (Euler angles for simplicity)
  const [orientation, setOrientation] = useState({ roll: 0, pitch: 0, yaw: 0 });
  
  // Waypoints list
  const [waypoints, setWaypoints] = useState([]);
  
  // Panel state
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [showWaypoints, setShowWaypoints] = useState(false);
  
  // Don't show in Real Robot mode
  if (genesisMode === 'Real Robot') {
    return null;
  }
  
  // Convert Euler to quaternion (simplified)
  const eulerToQuaternion = (roll, pitch, yaw) => {
    const cr = Math.cos(roll * 0.5);
    const sr = Math.sin(roll * 0.5);
    const cp = Math.cos(pitch * 0.5);
    const sp = Math.sin(pitch * 0.5);
    const cy = Math.cos(yaw * 0.5);
    const sy = Math.sin(yaw * 0.5);
    
    return {
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    };
  };
  
  const handlePositionChange = (axis, value) => {
    setPosition(prev => ({ ...prev, [axis]: parseFloat(value) || 0 }));
  };
  
  const handleOrientationChange = (axis, value) => {
    setOrientation(prev => ({ ...prev, [axis]: parseFloat(value) || 0 }));
  };
  
  const handleSetGoal = useCallback(() => {
    // Get socket from window or context
    const socketInstance = window.socket;
    if (!socketInstance) {
      console.warn('Socket not available');
      return;
    }
    
    const quat = eulerToQuaternion(
      orientation.roll * Math.PI / 180,
      orientation.pitch * Math.PI / 180,
      orientation.yaw * Math.PI / 180
    );
    
    const goalData = {
      position: [position.x, position.y, position.z],
      orientation: [quat.w, quat.x, quat.y, quat.z],
      waypoints: waypoints.map(wp => ({
        position: [wp.x, wp.y, wp.z],
        orientation: wp.quat || [1, 0, 0, 0]
      }))
    };
    
    socketInstance.emit('genesis_set_goal', goalData);
    console.log('Goal set:', goalData);
  }, [position, orientation, waypoints]);
  
  const handleAddWaypoint = () => {
    setWaypoints(prev => [
      ...prev,
      {
        id: Date.now(),
        x: position.x,
        y: position.y,
        z: position.z,
        quat: eulerToQuaternion(
          orientation.roll * Math.PI / 180,
          orientation.pitch * Math.PI / 180,
          orientation.yaw * Math.PI / 180
        )
      }
    ]);
  };
  
  const handleRemoveWaypoint = (id) => {
    setWaypoints(prev => prev.filter(wp => wp.id !== id));
  };
  
  const handleMoveWaypoint = (id, direction) => {
    setWaypoints(prev => {
      const idx = prev.findIndex(wp => wp.id === id);
      if (idx === -1) return prev;
      
      const newIdx = direction === 'up' ? idx - 1 : idx + 1;
      if (newIdx < 0 || newIdx >= prev.length) return prev;
      
      const newWaypoints = [...prev];
      [newWaypoints[idx], newWaypoints[newIdx]] = [newWaypoints[newIdx], newWaypoints[idx]];
      return newWaypoints;
    });
  };
  
  const handleClearGoal = () => {
    setPosition({ x: 0, y: 0, z: 0 });
    setOrientation({ roll: 0, pitch: 0, yaw: 0 });
    setWaypoints([]);
  };
  
  return (
    <div className={`goal-composer ${isCollapsed ? 'collapsed' : ''}`}>
      <div className="goal-composer-header" onClick={() => setIsCollapsed(!isCollapsed)}>
        <span>Goal Composer</span>
        <span className="goal-composer-toggle">{isCollapsed ? '▼' : '▲'}</span>
      </div>
      
      {!isCollapsed && (
        <div className="goal-composer-content">
          {/* Position inputs */}
          <div className="goal-section">
            <div className="goal-section-label">Target Position</div>
            <div className="goal-inputs-grid">
              <div className="goal-input-group">
                <label>X</label>
                <input
                  type="number"
                  step="0.1"
                  value={position.x}
                  onChange={(e) => handlePositionChange('x', e.target.value)}
                />
              </div>
              <div className="goal-input-group">
                <label>Y</label>
                <input
                  type="number"
                  step="0.1"
                  value={position.y}
                  onChange={(e) => handlePositionChange('y', e.target.value)}
                />
              </div>
              <div className="goal-input-group">
                <label>Z</label>
                <input
                  type="number"
                  step="0.1"
                  value={position.z}
                  onChange={(e) => handlePositionChange('z', e.target.value)}
                />
              </div>
            </div>
          </div>
          
          {/* Orientation inputs */}
          <div className="goal-section">
            <div className="goal-section-label">Target Orientation (degrees)</div>
            <div className="goal-inputs-grid">
              <div className="goal-input-group">
                <label>Roll</label>
                <input
                  type="number"
                  step="5"
                  value={orientation.roll}
                  onChange={(e) => handleOrientationChange('roll', e.target.value)}
                />
              </div>
              <div className="goal-input-group">
                <label>Pitch</label>
                <input
                  type="number"
                  step="5"
                  value={orientation.pitch}
                  onChange={(e) => handleOrientationChange('pitch', e.target.value)}
                />
              </div>
              <div className="goal-input-group">
                <label>Yaw</label>
                <input
                  type="number"
                  step="5"
                  value={orientation.yaw}
                  onChange={(e) => handleOrientationChange('yaw', e.target.value)}
                />
              </div>
            </div>
          </div>
          
          {/* Waypoints section */}
          <div className="goal-section">
            <div 
              className="goal-section-label waypoints-toggle"
              onClick={() => setShowWaypoints(!showWaypoints)}
            >
              Waypoints ({waypoints.length}) {showWaypoints ? '▲' : '▼'}
            </div>
            
            {showWaypoints && (
              <div className="waypoints-list">
                {waypoints.map((wp, idx) => (
                  <div key={wp.id} className="waypoint-item">
                    <span className="waypoint-idx">{idx + 1}</span>
                    <span className="waypoint-pos">
                      ({wp.x.toFixed(1)}, {wp.y.toFixed(1)}, {wp.z.toFixed(1)})
                    </span>
                    <div className="waypoint-actions">
                      <button onClick={() => handleMoveWaypoint(wp.id, 'up')}>↑</button>
                      <button onClick={() => handleMoveWaypoint(wp.id, 'down')}>↓</button>
                      <button onClick={() => handleRemoveWaypoint(wp.id)}>✕</button>
                    </div>
                  </div>
                ))}
                
                <button 
                  className="goal-button add-waypoint"
                  onClick={handleAddWaypoint}
                >
                  + Add Current as Waypoint
                </button>
              </div>
            )}
          </div>
          
          {/* Action buttons */}
          <div className="goal-actions">
            <button className="goal-button primary" onClick={handleSetGoal}>
              Set Goal
            </button>
            <button className="goal-button" onClick={handleClearGoal}>
              Clear
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default GoalComposer;
