import React, { useState, useEffect } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/GenesisRobotSelector.css';

const GenesisRobotSelector = () => {
  const { socket, robotList, loadRobot, unloadRobot, currentRobot } = useGenesis();
  const [selectedRobot, setSelectedRobot] = useState(null);
  const [robotDetails, setRobotDetails] = useState(null);
  const [showDetails, setShowDetails] = useState(false);
  const [filter, setFilter] = useState('all');

  useEffect(() => {
    console.log('[GenesisRobotSelector] Robot list updated:', robotList);
  }, [robotList]);

  const categories = {
    all: robotList,
    manipulator: robotList.filter(r => r.category === 'manipulator'),
    mobile: robotList.filter(r => r.category === 'mobile'),
    humanoid: robotList.filter(r => r.category === 'humanoid'),
    quadruped: robotList.filter(r => r.category === 'quadruped')
  };

  const categoryLabels = {
    all: 'All Robots',
    manipulator: 'Manipulators',
    mobile: 'Mobile Robots',
    humanoid: 'Humanoids',
    quadruped: 'Quadrupeds'
  };

  const handleRobotSelect = (robot) => {
    console.log('[GenesisRobotSelector] Robot selected:', robot);
    setSelectedRobot(robot.name);
    setShowDetails(true);

    if (socket) {
      socket.emit('genesis_get_robot_info', { robot_name: robot.name });
    }
  };

  const handleRefresh = () => {
    console.log('[GenesisRobotSelector] Refreshing robot list');
    if (socket) {
      socket.emit('genesis_scan_robots');
    }
  };

  const handleLoadRobot = () => {
    if (selectedRobot && socket) {
      loadRobot(selectedRobot);
      setShowDetails(false);
    }
  };

  useEffect(() => {
    if (!socket) return;

    const handleRobotInfo = (data) => {
      setRobotDetails(data);
    };

    socket.on('genesis_robot_info', handleRobotInfo);

    return () => {
      socket.off('genesis_robot_info', handleRobotInfo);
    };
  }, [socket]);

  const filteredRobots = categories[filter] || [];

  return (
    <div className="robot-selector-content">
      {/* Filter tabs */}
      <div className="category-tabs">
        {Object.keys(categoryLabels).map(category => (
          <button
            key={category}
            className={`category-tab ${filter === category ? 'active' : ''}`}
            onClick={() => setFilter(category)}
          >
            {categoryLabels[category]}
            <span className="count">({categories[category].length})</span>
          </button>
        ))}
      </div>

      {/* Status bar with loaded robot indicator */}
      <div className="selector-header">
        <div className="robot-count">
          {filteredRobots.length} robot{filteredRobots.length !== 1 ? 's' : ''} found
        </div>

        {currentRobot && (
          <div className="loaded-indicator">
            <span className="loaded-label">Loaded:</span>
            <span className="loaded-name">{currentRobot.label || currentRobot.name}</span>
            <button
              className="unload-button"
              onClick={unloadRobot}
              title="Unload robot"
            >
              Unload
            </button>
          </div>
        )}

        <button
          className="refresh-button"
          onClick={handleRefresh}
        >
          Refresh
        </button>
      </div>

      {/* Robot grid */}
      <div className="robot-grid">
        {filteredRobots.length === 0 ? (
          <div className="no-robots">
            <p>No robots found in this category</p>
            <p className="hint">Run genesis_ros setup to configure robots</p>
          </div>
        ) : (
          filteredRobots.map(robot => (
            <RobotCard
              key={robot.name}
              robot={robot}
              isSelected={selectedRobot === robot.name}
              isLoaded={currentRobot?.name === robot.name}
              onSelect={handleRobotSelect}
            />
          ))
        )}
      </div>

      {/* Robot details panel */}
      {showDetails && robotDetails && (
        <div className="robot-details-overlay" onClick={() => setShowDetails(false)}>
          <div className="robot-details-panel" onClick={(e) => e.stopPropagation()}>
            <button className="close-button" onClick={() => setShowDetails(false)}>
              {'\u2715'}
            </button>

            <h2>{robotDetails.label || robotDetails.name}</h2>
            <p className="description">{robotDetails.description}</p>

            <div className="details-grid">
              <div className="detail-item">
                <span className="detail-label">Category:</span>
                <span className="detail-value">{robotDetails.category}</span>
              </div>

              {robotDetails.specs && (
                <>
                  {robotDetails.specs.dof && (
                    <div className="detail-item">
                      <span className="detail-label">DOF:</span>
                      <span className="detail-value">{robotDetails.specs.dof}</span>
                    </div>
                  )}

                  {robotDetails.specs.payload && (
                    <div className="detail-item">
                      <span className="detail-label">Payload:</span>
                      <span className="detail-value">{robotDetails.specs.payload} kg</span>
                    </div>
                  )}

                  {robotDetails.specs.reach && (
                    <div className="detail-item">
                      <span className="detail-label">Reach:</span>
                      <span className="detail-value">{robotDetails.specs.reach} m</span>
                    </div>
                  )}

                  {robotDetails.specs.mass && (
                    <div className="detail-item">
                      <span className="detail-label">Mass:</span>
                      <span className="detail-value">{robotDetails.specs.mass} kg</span>
                    </div>
                  )}
                </>
              )}

              {robotDetails.control && (
                <div className="detail-item">
                  <span className="detail-label">Control:</span>
                  <span className="detail-value">{robotDetails.control.type}</span>
                </div>
              )}
            </div>

            {robotDetails.sensors && robotDetails.sensors.length > 0 && (
              <div className="sensors-list">
                <h3>Sensors</h3>
                <ul>
                  {robotDetails.sensors.map((sensor, i) => (
                    <li key={i}>
                      <span className="sensor-type">{sensor.type}</span>
                      {sensor.label && <span className="sensor-label">: {sensor.label}</span>}
                    </li>
                  ))}
                </ul>
              </div>
            )}

            <div className="details-actions">
              <button className="cancel-button" onClick={() => setShowDetails(false)}>
                Cancel
              </button>
              <button className="load-button" onClick={handleLoadRobot}>
                {currentRobot && currentRobot.name !== selectedRobot ? 'Switch Robot' : 'Load Robot'}
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

const RobotCard = ({ robot, isSelected, isLoaded, onSelect }) => {
  const getCategoryIcon = (category) => {
    switch (category) {
      case 'manipulator':
        return '\uD83E\uDDBE';
      case 'mobile':
        return '\uD83E\uDD16';
      case 'humanoid':
        return '\uD83E\uDDCD';
      case 'quadruped':
        return '\uD83D\uDC15';
      default:
        return '\u2699\uFE0F';
    }
  };

  const specs = robot.specs || {};
  const dof = specs.dof || robot.dof;
  const mass = specs.mass || robot.mass;
  const payload = specs.payload || robot.payload;
  const reach = specs.reach || robot.reach;

  const cardClasses = [
    'robot-card',
    isSelected ? 'selected' : '',
    isLoaded ? 'loaded' : ''
  ].filter(Boolean).join(' ');

  return (
    <div
      className={cardClasses}
      onClick={() => onSelect(robot)}
    >
      {isLoaded && <div className="loaded-badge">Active</div>}
      <div className="robot-preview">
        {robot.thumbnail ? (
          <img src={robot.thumbnail} alt={robot.label} />
        ) : (
          <div className="robot-icon">
            {getCategoryIcon(robot.category)}
          </div>
        )}
      </div>

      <div className="robot-info">
        <h4>{robot.label || robot.name}</h4>
        <p className="robot-description">
          {robot.description ? (
            robot.description.length > 80
              ? robot.description.substring(0, 80) + '...'
              : robot.description
          ) : (
            'No description available'
          )}
        </p>

        <div className="robot-specs">
          {dof && (
            <span className="spec">
              <span className="spec-label">DOF:</span> {dof}
            </span>
          )}
          {payload && (
            <span className="spec">
              <span className="spec-label">Load:</span> {payload}kg
            </span>
          )}
          {reach && (
            <span className="spec">
              <span className="spec-label">Reach:</span> {reach}m
            </span>
          )}
        </div>

        <div className="robot-category">
          <span className="category-badge">{robot.category || 'unknown'}</span>
        </div>
      </div>
    </div>
  );
};

export default GenesisRobotSelector;
