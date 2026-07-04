import React, { useMemo } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/ActionAvailabilityHUD.css';

// Skill configuration — add a new entry here to surface a new skill.
// glyph:   button label shown on the chip (gamepad face-button name)
// kind:    maps to available[kind] from the perception message
// color:   CSS custom-property name for the chip accent color
const SKILL_CHIPS = [
  { glyph: 'X', kind: 'jump',   label: 'JUMP',  color: '--hud-color-jump'  },
  { glyph: 'B', kind: 'crouch', label: 'CROUCH', color: '--hud-color-crouch' },
  { glyph: 'Y', kind: 'climb',  label: 'CLIMB',  color: '--hud-color-climb'  },
];

// Obstacle kind -> display icon character
const KIND_ICONS = {
  hurdle: 'H',
  crawl_bar: 'C',
  box: 'B',
};

const LaneProjection = ({ obstacles }) => {
  // 1D lane strip: robot at left (x=0), obstacles placed by dx (metres to the right).
  // Visible window: 0..2.5 m in front of the robot.
  const WINDOW_M = 2.5;
  const STRIP_W = 240;
  const STRIP_H = 32;

  const toX = (dx) => Math.min(STRIP_W - 8, Math.max(4, (dx / WINDOW_M) * STRIP_W));

  return (
    <svg
      className="hud-lane"
      viewBox={`0 0 ${STRIP_W} ${STRIP_H}`}
      width={STRIP_W}
      height={STRIP_H}
      aria-label="Obstacle lane projection"
    >
      {/* Lane baseline */}
      <line x1="0" y1={STRIP_H / 2} x2={STRIP_W} y2={STRIP_H / 2} className="hud-lane-line" />

      {/* Robot marker */}
      <rect x="2" y={STRIP_H / 2 - 6} width="8" height="12" rx="2" className="hud-lane-robot" />

      {/* Obstacle markers */}
      {obstacles.map((obs, i) => {
        const x = toX(obs.dx);
        const heightFrac = Math.min(1, obs.z_high / 0.3);
        const barH = Math.max(4, heightFrac * (STRIP_H - 4));
        const y = STRIP_H / 2 - barH / 2;
        return (
          <g key={i} className={`hud-lane-obs hud-lane-obs--${obs.kind}`}>
            <rect x={x - 3} y={y} width="6" height={barH} rx="1" />
            <text x={x} y={STRIP_H - 2} textAnchor="middle" className="hud-lane-kind">
              {KIND_ICONS[obs.kind] || '?'}
            </text>
          </g>
        );
      })}

      {/* 0.8 m jump-range marker */}
      <line
        x1={toX(0.8)}
        y1="2"
        x2={toX(0.8)}
        y2={STRIP_H - 2}
        className="hud-lane-range-marker"
        strokeDasharray="3 2"
      />
    </svg>
  );
};

const ActionAvailabilityHUD = () => {
  const { policyPerception } = useGenesis();

  const available = policyPerception?.available ?? {};
  const skillActive = policyPerception?.skill_active ?? {};
  const obstacles = policyPerception?.obstacles ?? [];

  // Only render when at least one skill availability is defined (env exposes perception).
  const hasPerception = policyPerception !== null;
  if (!hasPerception) return null;

  return (
    <div className="action-hud" aria-label="Action availability">
      {/* Lane projection strip */}
      {obstacles.length > 0 && (
        <div className="action-hud__lane-row">
          <LaneProjection obstacles={obstacles} />
        </div>
      )}

      {/* Skill chips */}
      <div className="action-hud__chips">
        {SKILL_CHIPS.map(({ glyph, kind, label, color }) => {
          const isAvailable = !!available[kind];
          const isActive = !!skillActive[kind];
          return (
            <div
              key={kind}
              className={[
                'action-hud__chip',
                isAvailable ? 'action-hud__chip--available' : 'action-hud__chip--unavailable',
                isActive ? 'action-hud__chip--active' : '',
              ].join(' ').trim()}
              style={{ '--chip-color': `var(${color})` }}
              title={isAvailable ? `${label} available` : `${label} not available`}
            >
              <span className="action-hud__chip-glyph">{glyph}</span>
              <span className="action-hud__chip-label">{label}</span>
            </div>
          );
        })}
      </div>
    </div>
  );
};

export default ActionAvailabilityHUD;
