import React, { useState, useEffect, useRef } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/TrainingDashboard.css';

const TrainingDashboard = () => {
  const { genesisMode, trainingMetrics } = useGenesis();
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [rewardHistory, setRewardHistory] = useState([]);
  const canvasRef = useRef(null);
  
  // Don't show in Real Robot mode
  if (genesisMode === 'Real Robot') {
    return null;
  }
  
  // Update reward history when metrics change
  useEffect(() => {
    if (trainingMetrics?.total_reward !== undefined) {
      setRewardHistory(prev => {
        const newHistory = [...prev, trainingMetrics.total_reward];
        // Keep last 100 data points
        return newHistory.slice(-100);
      });
    }
  }, [trainingMetrics]);
  
  // Draw reward chart
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || rewardHistory.length < 2) return;
    
    const ctx = canvas.getContext('2d');
    const width = canvas.width;
    const height = canvas.height;
    
    // Clear
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, width, height);
    
    // Calculate min/max for scaling
    const min = Math.min(...rewardHistory);
    const max = Math.max(...rewardHistory);
    const range = max - min || 1;
    
    // Draw grid lines
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = (height * i) / 4;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(width, y);
      ctx.stroke();
    }
    
    // Draw reward line
    ctx.strokeStyle = '#00aaff';
    ctx.lineWidth = 2;
    ctx.beginPath();
    
    rewardHistory.forEach((reward, idx) => {
      const x = (idx / (rewardHistory.length - 1)) * width;
      const y = height - ((reward - min) / range) * height;
      
      if (idx === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    
    ctx.stroke();
    
    // Draw current value label
    const currentValue = rewardHistory[rewardHistory.length - 1];
    ctx.fillStyle = '#fff';
    ctx.font = '12px monospace';
    ctx.textAlign = 'right';
    ctx.fillText(currentValue.toFixed(2), width - 5, 15);
    
  }, [rewardHistory]);
  
  const metrics = trainingMetrics || {};
  
  return (
    <div className={`training-dashboard ${isCollapsed ? 'collapsed' : ''}`}>
      <div className="dashboard-header" onClick={() => setIsCollapsed(!isCollapsed)}>
        <span>Training Dashboard</span>
        <span className="dashboard-toggle">{isCollapsed ? '▼' : '▲'}</span>
      </div>
      
      {!isCollapsed && (
        <div className="dashboard-content">
          {/* Reward chart */}
          <div className="dashboard-section">
            <div className="dashboard-section-label">Total Reward</div>
            <canvas 
              ref={canvasRef} 
              className="reward-chart"
              width={240}
              height={80}
            />
          </div>
          
          {/* Metrics grid */}
          <div className="dashboard-section">
            <div className="dashboard-section-label">Metrics</div>
            <div className="metrics-grid">
              <div className="metric-item">
                <span className="metric-label">Step</span>
                <span className="metric-value">{metrics.step_count || 0}</span>
              </div>
              <div className="metric-item">
                <span className="metric-label">FPS</span>
                <span className="metric-value">{metrics.fps || 0}</span>
              </div>
              <div className="metric-item">
                <span className="metric-label">Envs</span>
                <span className="metric-value">{metrics.num_envs || 0}</span>
              </div>
              <div className="metric-item">
                <span className="metric-label">Reward</span>
                <span className="metric-value">{(metrics.total_reward || 0).toFixed(2)}</span>
              </div>
            </div>
          </div>
          
          {/* Episode stats */}
          <div className="dashboard-section">
            <div className="dashboard-section-label">Episode Stats</div>
            <div className="metrics-grid">
              <div className="metric-item">
                <span className="metric-label">Mean Len</span>
                <span className="metric-value">
                  {(metrics.episode_lengths?.mean || 0).toFixed(1)}
                </span>
              </div>
              <div className="metric-item">
                <span className="metric-label">Min Len</span>
                <span className="metric-value">
                  {metrics.episode_lengths?.min || 0}
                </span>
              </div>
              <div className="metric-item">
                <span className="metric-label">Max Len</span>
                <span className="metric-value">
                  {metrics.episode_lengths?.max || 0}
                </span>
              </div>
            </div>
          </div>
          
          {/* Reward components */}
          {metrics.reward_components && Object.keys(metrics.reward_components).length > 0 && (
            <div className="dashboard-section">
              <div className="dashboard-section-label">Reward Components</div>
              <div className="reward-components">
                {Object.entries(metrics.reward_components).map(([name, data]) => (
                  <div key={name} className="reward-component">
                    <span className="component-name">{name}</span>
                    <span className="component-value">
                      {(data.last_mean || 0).toFixed(3)}
                    </span>
                    <span className="component-weight">
                      w: {data.weight}
                    </span>
                  </div>
                ))}
              </div>
            </div>
          )}
          
          {/* Mode indicator */}
          <div className="dashboard-section">
            <div className="dashboard-section-label">Mode</div>
            <div className="mode-indicator">
              <span className={`mode-badge ${metrics.mode || 'unknown'}`}>
                {metrics.mode || genesisMode}
              </span>
              <span className="actor-badge">
                Actor: {metrics.actor_tag || 'N/A'}
              </span>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default TrainingDashboard;
