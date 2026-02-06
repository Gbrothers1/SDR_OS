import React from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/SimTelemetryPane.css';

const SPARKLINE_CHARS = '\u2581\u2582\u2583\u2584\u2585\u2586\u2587\u2588';

const sparkline = (values, min = -1, max = 1) => {
  if (!values || values.length === 0) return '';
  const range = max - min || 1;
  return values.map(v => {
    const idx = Math.round(((v - min) / range) * (SPARKLINE_CHARS.length - 1));
    return SPARKLINE_CHARS[Math.max(0, Math.min(SPARKLINE_CHARS.length - 1, idx))];
  }).join('');
};

const MiniBar = ({ value, min = 0, max = 1, color = 'var(--color-accent-blue)' }) => {
  const pct = Math.max(0, Math.min(100, ((value - min) / (max - min || 1)) * 100));
  const isNeg = value < 0;
  return (
    <div className="sim-tel__minibar">
      <div
        className="sim-tel__minibar-fill"
        style={{
          width: `${Math.abs(pct)}%`,
          background: isNeg ? 'var(--color-accent-red)' : color,
          marginLeft: isNeg ? 'auto' : 0,
        }}
      />
    </div>
  );
};

const Vec3Row = ({ label, values }) => {
  if (!values || values.length < 3) return null;
  return (
    <div className="sim-tel__vec3-row">
      <span className="sim-tel__obs-label">{label}</span>
      <span className="sim-tel__obs-val">{values[0].toFixed(2)}</span>
      <span className="sim-tel__obs-val">{values[1].toFixed(2)}</span>
      <span className="sim-tel__obs-val">{values[2].toFixed(2)}</span>
    </div>
  );
};

const HighDimRow = ({ label, values, count }) => {
  if (!values || values.length === 0) return null;
  return (
    <div className="sim-tel__highdim-row">
      <span className="sim-tel__obs-label">{label}</span>
      <span className="sim-tel__obs-count">({count})</span>
      <span className="sim-tel__sparkline">{sparkline(values, -2, 2)}</span>
    </div>
  );
};

const SimTelemetryPane = ({ compact = false }) => {
  const {
    trainingMetrics,
    obsBreakdown,
    rewardBreakdown,
    velocityCommand,
  } = useGenesis();

  const step = trainingMetrics?.step_count ?? 0;
  const anomalyCount = trainingMetrics?.anomaly_count ?? 0;
  const anomalyResetCount = trainingMetrics?.anomaly_reset_count ?? 0;
  const sps = trainingMetrics?.fps ?? 0;
  const totalReward = trainingMetrics?.total_reward ?? 0;

  if (compact) {
    return (
      <div className="sim-tel sim-tel--compact">
        <div className="sim-tel__strip-row">
          <span className="sim-tel__strip-label">SIM</span>
          <span className="sim-tel__strip-val">AD {anomalyCount}</span>
          <span className="sim-tel__strip-val">AR {anomalyResetCount}</span>
          <span className="sim-tel__strip-val">S {step.toLocaleString()}</span>
          <span className="sim-tel__strip-val">R {totalReward.toFixed(1)}</span>
          <span className="sim-tel__strip-val">{sps.toFixed(0)} SPS</span>
        </div>
      </div>
    );
  }

  const obs = obsBreakdown || trainingMetrics?.obs_breakdown;
  const rew = rewardBreakdown || trainingMetrics?.reward_breakdown;
  const velCmd = velocityCommand || trainingMetrics?.velocity_command;

  return (
    <div className="sim-tel">
      {/* Header stats */}
      <div className="sim-tel__header">
        <span>AD {anomalyCount}</span>
        <span>AR {anomalyResetCount}</span>
        <span>STEP {step.toLocaleString()}</span>
        <span>{sps.toFixed(0)} SPS</span>
      </div>

      {/* Velocity Command */}
      {velCmd && (
        <div className="sim-tel__section">
          <div className="sim-tel__section-title">VELOCITY CMD</div>
          <div className="sim-tel__vel-row">
            <span className="sim-tel__vel-label">Vx</span>
            <MiniBar value={velCmd.lin_vel_x || 0} min={-1} max={1} />
            <span className="sim-tel__vel-val">{(velCmd.lin_vel_x || 0).toFixed(2)} m/s</span>
          </div>
          <div className="sim-tel__vel-row">
            <span className="sim-tel__vel-label">Vy</span>
            <MiniBar value={velCmd.lin_vel_y || 0} min={-0.5} max={0.5} />
            <span className="sim-tel__vel-val">{(velCmd.lin_vel_y || 0).toFixed(2)} m/s</span>
          </div>
          <div className="sim-tel__vel-row">
            <span className="sim-tel__vel-label">{'\u03C9z'}</span>
            <MiniBar value={velCmd.ang_vel_z || 0} min={-1} max={1} />
            <span className="sim-tel__vel-val">{(velCmd.ang_vel_z || 0).toFixed(2)} r/s</span>
          </div>
        </div>
      )}

      {/* Observations */}
      {obs && (
        <div className="sim-tel__section">
          <div className="sim-tel__section-title">
            OBSERVATIONS
            <span className="sim-tel__dim-badge">48 dim</span>
          </div>
          <Vec3Row label="Ang Vel" values={obs.angular_velocity} />
          <Vec3Row label="Lin Vel" values={obs.linear_velocity} />
          <Vec3Row label="Gravity" values={obs.projected_gravity} />
          <Vec3Row label="Vel Cmd" values={obs.velocity_command} />
          <HighDimRow label="DoF Pos" values={obs.dof_position} count={12} />
          <HighDimRow label="DoF Vel" values={obs.dof_velocity} count={12} />
          <HighDimRow label="Actions" values={obs.actions} count={12} />
        </div>
      )}

      {/* Rewards */}
      {rew && (
        <div className="sim-tel__section">
          <div className="sim-tel__section-title">
            REWARDS
            <span className="sim-tel__reward-total">{'\u03A3'} = {totalReward.toFixed(1)}</span>
          </div>
          {Object.entries(rew).map(([name, value]) => (
            <div key={name} className="sim-tel__reward-row">
              <span className="sim-tel__reward-name">{name}</span>
              <MiniBar value={value} min={-1} max={1} color={value >= 0 ? 'var(--color-accent-green)' : 'var(--color-accent-red)'} />
              <span className="sim-tel__reward-val">{value.toFixed(3)}</span>
            </div>
          ))}
        </div>
      )}

      {/* Robot State */}
      <div className="sim-tel__section">
        <div className="sim-tel__section-title">ROBOT STATE</div>
        <div className="sim-tel__state-grid">
          <span className="sim-tel__state-label">Reward</span>
          <span className="sim-tel__state-val">{totalReward.toFixed(2)}</span>
          <span className="sim-tel__state-label">Anomalies</span>
          <span className="sim-tel__state-val">{anomalyCount}</span>
          <span className="sim-tel__state-label">Resets</span>
          <span className="sim-tel__state-val">{anomalyResetCount}</span>
        </div>
      </div>
    </div>
  );
};

export default SimTelemetryPane;
