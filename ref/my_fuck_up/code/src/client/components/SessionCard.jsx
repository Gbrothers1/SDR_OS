import React, { useState, useEffect } from 'react';
import URDFPreview from './URDFPreview';
import '../styles/SessionCard.css';

// Load URDF registry for robot-to-URDF mapping
let urdfRegistry = null;

export default function SessionCard({
  session = null,
  isNew = false,
  focused = false,
  onClick,
}) {
  if (isNew) {
    return (
      <div
        className={`session-card session-card--new ${focused ? 'session-card--focused' : ''}`}
        onClick={onClick}
      >
        <span className="session-card__new-icon">+</span>
        <span className="session-card__new-text">New Session</span>
      </div>
    );
  }

  // Load URDF registry once
  const [registry, setRegistry] = useState(urdfRegistry);

  useEffect(() => {
    if (!urdfRegistry) {
      fetch('/configs/urdf_registry.json')
        .then(res => res.json())
        .then(data => {
          urdfRegistry = data;
          setRegistry(data);
        })
        .catch(err => console.warn('Failed to load URDF registry:', err));
    }
  }, []);

  if (!session) return null;

  const { name, robot, type, preset, status, lastRun } = session;

  // Find URDF config for this robot (case-insensitive match)
  const robotKey = robot ? robot.toLowerCase().replace(/\s+/g, '_') : null;
  const robotConfig = registry?.robots?.[robotKey];

  return (
    <div
      className={`session-card ${focused ? 'session-card--focused' : ''}`}
      onClick={onClick}
    >
      {/* 3D URDF Preview Area */}
      <div className="session-card__preview">
        {robotConfig?.urdf ? (
          <URDFPreview
            urdfPath={robotConfig.urdf}
            cameraDistance={robotConfig.cameraDistance || 1.5}
            cameraHeight={robotConfig.cameraHeight || 0.8}
            autoRotate={true}
          />
        ) : (
          <span className="session-card__preview-placeholder">{robot || 'Robot'}</span>
        )}
      </div>

      {/* Session Info */}
      <div className="session-card__info">
        <div className="session-card__header">
          <div>
            <h3 className="session-card__name">{name}</h3>
            <span className="session-card__robot">{robot}</span>
          </div>
        </div>

        <div className="session-card__badges">
          <span className="session-card__badge session-card__badge--type">
            {type}
          </span>
          <span className="session-card__badge session-card__badge--preset">
            {preset}
          </span>
        </div>

        <div className="session-card__status-row">
          <span className="session-card__badge session-card__badge--status">
            {status}
          </span>
          <span className="session-card__time">{lastRun}</span>
        </div>
      </div>
    </div>
  );
}
