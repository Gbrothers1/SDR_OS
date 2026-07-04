import React, { useEffect, useRef, useState, useCallback } from 'react';
import ROSLIB from 'roslib';
import { useGenesis } from '../contexts/GenesisContext';
import { useSettings } from '../contexts/SettingsContext';
import '../styles/SensorPanel.css';

// ─── Constants ───────────────────────────────────────────────────────────────

const STALE_THRESHOLD_MS = 3000;   // topic considered stale after 3 s
const HISTORY_LEN = 60;            // sparkline samples to keep

// ─── Utilities ───────────────────────────────────────────────────────────────

const fmt = (v, decimals = 3) => {
  if (v == null || Number.isNaN(v)) return '—';
  return Number(v).toFixed(decimals);
};

const fmtDeg = (rad) => {
  if (rad == null) return '—';
  return `${(rad * (180 / Math.PI)).toFixed(1)}°`;
};

const fmtHz = (rate) => {
  if (rate == null) return '—';
  if (rate < 0.5) return `${(rate * 1000).toFixed(0)} mHz`;
  return `${rate.toFixed(1)} Hz`;
};

const pushHistory = (arr, value, len = HISTORY_LEN) => {
  const next = arr.slice(-(len - 1));
  next.push(value);
  return next;
};

// ─── Sparkline ───────────────────────────────────────────────────────────────

const Sparkline = ({ data, color = '#5b8def', height = 28, width = 80 }) => {
  const canvasRef = useRef(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || data.length < 2) return;
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0, 0, width, height);

    const min = Math.min(...data);
    const max = Math.max(...data);
    const range = max - min || 1;

    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    data.forEach((v, i) => {
      const x = (i / (data.length - 1)) * width;
      const y = height - ((v - min) / range) * (height - 2) - 1;
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.stroke();
  }, [data, color, height, width]);

  return (
    <canvas
      ref={canvasRef}
      width={width}
      height={height}
      className="sensor-panel__sparkline"
    />
  );
};

// ─── Topic health badge ───────────────────────────────────────────────────────

const TopicBadge = ({ name, lastSeen, hz }) => {
  const age = lastSeen ? Date.now() - lastSeen : null;
  const ok = age != null && age < STALE_THRESHOLD_MS;
  const label = ok ? fmtHz(hz) : age == null ? 'no data' : 'stale';
  return (
    <div className={`sensor-panel__badge ${ok ? 'sensor-panel__badge--ok' : 'sensor-panel__badge--dead'}`}>
      <span className="sensor-panel__badge-dot" />
      <span className="sensor-panel__badge-name">{name}</span>
      <span className="sensor-panel__badge-hz">{label}</span>
    </div>
  );
};

// ─── IMU section ─────────────────────────────────────────────────────────────

const StateSection = ({ perception, safetyState }) => {
  const gait = perception?.gait;
  const avail = perception?.available || {};
  const active = perception?.skill_active || {};
  const rows = [
    ['Gait', gait ? `${gait.name.toUpperCase()}  T=${Number(gait.period).toFixed(2)}s` : 'n/a (no gait policy)'],
    ['Mode', gait && gait.mode ? gait.mode.toUpperCase() : '—'],
    ['Safety', safetyState ? `${safetyState.mode}${safetyState.reason && safetyState.reason !== 'ok' ? ` (${safetyState.reason})` : ''}` : 'n/a'],
    ['Jump (X)', active.jump ? 'ACTIVE' : avail.jump ? 'available' : '—'],
    ['Crouch (B)', active.crouch ? 'ACTIVE' : avail.crouch ? 'available' : '—'],
    ['Climb (Y)', active.climb ? 'ACTIVE' : avail.climb ? 'available' : '—'],
  ];
  return (
    <div className="sensor-panel__section">
      <div className="sensor-panel__section-title">Robot State <span className="sensor-panel__source">(sim telemetry)</span></div>
      <table className="sensor-panel__kv-table"><tbody>
        {rows.map(([k, v]) => (
          <tr key={k}><td className="sensor-panel__kv-key">{k}</td><td className="sensor-panel__kv-val">{v}</td></tr>
        ))}
      </tbody></table>
      {!perception && <div className="sensor-panel__waiting">Waiting for sim perception telemetry…</div>}
    </div>
  );
};

const ImuSection = ({ imu, topicHealth }) => {
  if (!imu) {
    return (
      <div className="sensor-panel__section">
        <div className="sensor-panel__section-title">IMU</div>
        <TopicBadge name={topicHealth.name} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
        <div className="sensor-panel__empty">Waiting for IMU data…</div>
      </div>
    );
  }

  const { orientation, linear_acceleration: la, angular_velocity: av } = imu;
  const quat = orientation || {};
  const accel = la || {};
  const gyro = av || {};

  const roll = Math.atan2(
    2 * (quat.w * quat.x + quat.y * quat.z),
    1 - 2 * (quat.x * quat.x + quat.y * quat.y)
  );
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (quat.w * quat.y - quat.z * quat.x))));
  const yaw = Math.atan2(
    2 * (quat.w * quat.z + quat.x * quat.y),
    1 - 2 * (quat.y * quat.y + quat.z * quat.z)
  );

  return (
    <div className="sensor-panel__section">
      <div className="sensor-panel__section-title">
        IMU
        <TopicBadge name={topicHealth.name} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
      </div>

      <div className="sensor-panel__grid2">
        <div className="sensor-panel__group">
          <div className="sensor-panel__group-title">Orientation</div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Roll</span>
            <span className="sensor-panel__val">{fmtDeg(roll)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Pitch</span>
            <span className="sensor-panel__val">{fmtDeg(pitch)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Yaw</span>
            <span className="sensor-panel__val">{fmtDeg(yaw)}</span>
          </div>
        </div>

        <div className="sensor-panel__group">
          <div className="sensor-panel__group-title">Accel (m/s²)</div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label x">X</span>
            <span className="sensor-panel__val">{fmt(accel.x)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label y">Y</span>
            <span className="sensor-panel__val">{fmt(accel.y)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label z">Z</span>
            <span className="sensor-panel__val">{fmt(accel.z)}</span>
          </div>
        </div>

        <div className="sensor-panel__group">
          <div className="sensor-panel__group-title">Gyro (rad/s)</div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label x">X</span>
            <span className="sensor-panel__val">{fmt(gyro.x)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label y">Y</span>
            <span className="sensor-panel__val">{fmt(gyro.y)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label z">Z</span>
            <span className="sensor-panel__val">{fmt(gyro.z)}</span>
          </div>
        </div>

        <div className="sensor-panel__group sensor-panel__group--spark">
          <div className="sensor-panel__group-title">Accel |total|</div>
          <Sparkline data={topicHealth.accelHistory} color="#5b8def" />
        </div>
      </div>
    </div>
  );
};

// ─── Odometry section ─────────────────────────────────────────────────────────

const OdomSection = ({ odom, topicHealth }) => {
  if (!odom) {
    return (
      <div className="sensor-panel__section">
        <div className="sensor-panel__section-title">Odometry</div>
        <TopicBadge name={topicHealth.name} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
        <div className="sensor-panel__empty">Waiting for odom data…</div>
      </div>
    );
  }

  const pos = odom.pose?.pose?.position || {};
  const ori = odom.pose?.pose?.orientation || {};
  const lv = odom.twist?.twist?.linear || {};
  const av = odom.twist?.twist?.angular || {};

  const yaw = Math.atan2(
    2 * (ori.w * ori.z + ori.x * ori.y),
    1 - 2 * (ori.y * ori.y + ori.z * ori.z)
  );

  const speed = Math.sqrt((lv.x || 0) ** 2 + (lv.y || 0) ** 2 + (lv.z || 0) ** 2);

  return (
    <div className="sensor-panel__section">
      <div className="sensor-panel__section-title">
        Odometry
        <TopicBadge name={topicHealth.name} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
      </div>

      <div className="sensor-panel__grid2">
        <div className="sensor-panel__group">
          <div className="sensor-panel__group-title">Position (m)</div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label x">X</span>
            <span className="sensor-panel__val">{fmt(pos.x)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label y">Y</span>
            <span className="sensor-panel__val">{fmt(pos.y)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label z">Z</span>
            <span className="sensor-panel__val">{fmt(pos.z)}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Yaw</span>
            <span className="sensor-panel__val">{fmtDeg(yaw)}</span>
          </div>
        </div>

        <div className="sensor-panel__group">
          <div className="sensor-panel__group-title">Velocity</div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label x">Vx</span>
            <span className="sensor-panel__val">{fmt(lv.x)} m/s</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label y">Vy</span>
            <span className="sensor-panel__val">{fmt(lv.y)} m/s</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Wz</span>
            <span className="sensor-panel__val">{fmt(av.z)} r/s</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">|V|</span>
            <span className="sensor-panel__val">{fmt(speed)} m/s</span>
          </div>
        </div>

        <div className="sensor-panel__group sensor-panel__group--spark sensor-panel__group--span2">
          <div className="sensor-panel__group-title">Speed history</div>
          <Sparkline data={topicHealth.speedHistory} color="#4caf7d" width={160} />
        </div>
      </div>
    </div>
  );
};

// ─── Joint states section ─────────────────────────────────────────────────────

const JointSection = ({ joints, topicHealth }) => {
  if (!joints || !joints.name?.length) {
    return (
      <div className="sensor-panel__section">
        <div className="sensor-panel__section-title">Joint States</div>
        <TopicBadge name={topicHealth.name} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
        <div className="sensor-panel__empty">Waiting for joint states…</div>
      </div>
    );
  }

  const names = joints.name || [];
  const pos = joints.position || [];
  const vel = joints.velocity || [];
  const eff = joints.effort || [];

  return (
    <div className="sensor-panel__section">
      <div className="sensor-panel__section-title">
        Joint States
        <TopicBadge name={topicHealth.name} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
      </div>
      <div className="sensor-panel__joint-table">
        <div className="sensor-panel__joint-row sensor-panel__joint-row--header">
          <span>Joint</span>
          <span>Pos (rad)</span>
          <span>Vel (r/s)</span>
          <span>Eff (Nm)</span>
        </div>
        {names.map((name, i) => (
          <div key={name} className="sensor-panel__joint-row">
            <span className="sensor-panel__joint-name" title={name}>{name}</span>
            <span>{fmt(pos[i])}</span>
            <span>{fmt(vel[i])}</span>
            <span>{eff.length > i ? fmt(eff[i]) : '—'}</span>
          </div>
        ))}
      </div>
    </div>
  );
};

// ─── Lidar section ────────────────────────────────────────────────────────────

const LidarSection = ({ scan, topicHealth }) => {
  const topicName = topicHealth.name;

  if (!scan) {
    return (
      <div className="sensor-panel__section">
        <div className="sensor-panel__section-title">
          Lidar / Scan
          <TopicBadge name={topicName} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
        </div>
        <div className="sensor-panel__empty">
          Waiting for scan data…
          <div className="sensor-panel__empty-hint">
            Needs a publisher on <code>{topicName}</code>
            (sensor_msgs/LaserScan or PointCloud2).
          </div>
        </div>
      </div>
    );
  }

  const ranges = scan.ranges || [];
  const validRanges = ranges.filter((r) => isFinite(r) && r > 0);
  const minR = validRanges.length ? Math.min(...validRanges).toFixed(2) : '—';
  const maxR = validRanges.length ? Math.max(...validRanges).toFixed(2) : '—';
  const meanR = validRanges.length
    ? (validRanges.reduce((a, b) => a + b, 0) / validRanges.length).toFixed(2)
    : '—';
  const beams = ranges.length;
  const angleMin = fmtDeg(scan.angle_min);
  const angleMax = fmtDeg(scan.angle_max);

  return (
    <div className="sensor-panel__section">
      <div className="sensor-panel__section-title">
        Lidar / Scan
        <TopicBadge name={topicName} lastSeen={topicHealth.lastSeen} hz={topicHealth.hz} />
      </div>
      <div className="sensor-panel__grid2">
        <div className="sensor-panel__group">
          <div className="sensor-panel__group-title">Scan stats</div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Beams</span>
            <span className="sensor-panel__val">{beams}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">FOV</span>
            <span className="sensor-panel__val">{angleMin} → {angleMax}</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Min r</span>
            <span className="sensor-panel__val">{minR} m</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Max r</span>
            <span className="sensor-panel__val">{maxR} m</span>
          </div>
          <div className="sensor-panel__row">
            <span className="sensor-panel__label">Mean r</span>
            <span className="sensor-panel__val">{meanR} m</span>
          </div>
        </div>
      </div>
    </div>
  );
};

// ─── ROS connection health bar ────────────────────────────────────────────────

const RosHealthBar = ({ rosConnected, rosBridgeUrl }) => (
  <div className={`sensor-panel__ros-bar ${rosConnected ? 'sensor-panel__ros-bar--ok' : 'sensor-panel__ros-bar--dead'}`}>
    <span className="sensor-panel__ros-dot" />
    <span>
      {rosConnected
        ? `ROS connected — ${rosBridgeUrl}`
        : `ROS disconnected — connecting to ${rosBridgeUrl}`}
    </span>
  </div>
);

// ─── Main component ───────────────────────────────────────────────────────────

const SensorPanel = ({ ros, rosConnected }) => {
  const { getSetting } = useSettings();

  // Topic names from settings
  const imuTopic     = getSetting('topics', 'imu', '/imu/data');
  const odomTopic    = getSetting('topics', 'odom', '/odom');
  const jointsTopic  = getSetting('topics', 'joint_states', '/joint_states');
  const scanTopic    = getSetting('topics', 'scan', '/scan');
  const rosBridgeUrl = getSetting('connection', 'rosBridgeUrl', '');

  // Latest messages
  const [imuMsg,    setImuMsg]    = useState(null);
  const [odomMsg,   setOdomMsg]   = useState(null);
  const [jointsMsg, setJointsMsg] = useState(null);
  const [scanMsg,   setScanMsg]   = useState(null);

  const { policyPerception, safetyState } = useGenesis();

  // Per-topic health tracking
  const imuHealth    = useRef({ lastSeen: null, count: 0, windowStart: null, hz: null, accelHistory: [] });
  const odomHealth   = useRef({ lastSeen: null, count: 0, windowStart: null, hz: null, speedHistory: [] });
  const jointsHealth = useRef({ lastSeen: null, count: 0, windowStart: null, hz: null });
  const scanHealth   = useRef({ lastSeen: null, count: 0, windowStart: null, hz: null });

  // Rendered health snapshots (updated at render time)
  const [imuHealthSnap,    setImuHealthSnap]    = useState({ name: imuTopic,    lastSeen: null, hz: null, accelHistory: [] });
  const [odomHealthSnap,   setOdomHealthSnap]   = useState({ name: odomTopic,   lastSeen: null, hz: null, speedHistory: [] });
  const [jointsHealthSnap, setJointsHealthSnap] = useState({ name: jointsTopic, lastSeen: null, hz: null });
  const [scanHealthSnap,   setScanHealthSnap]   = useState({ name: scanTopic,   lastSeen: null, hz: null });

  // Active section tab
  const [tab, setTab] = useState('state');

  // Update hz estimate in a rolling 2-second window
  const updateHz = useCallback((ref) => {
    const now = Date.now();
    ref.count += 1;
    ref.lastSeen = now;
    if (!ref.windowStart) {
      ref.windowStart = now;
    } else if (now - ref.windowStart >= 2000) {
      ref.hz = ref.count / ((now - ref.windowStart) / 1000);
      ref.count = 0;
      ref.windowStart = now;
    }
  }, []);

  // Subscriptions — recreate whenever ros instance changes
  useEffect(() => {
    if (!ros) return;

    const subs = [];

    // IMU
    const imuSub = new ROSLIB.Topic({
      ros,
      name: imuTopic,
      messageType: 'sensor_msgs/Imu',
      throttle_rate: 100,
    });
    imuSub.subscribe((msg) => {
      updateHz(imuHealth.current);
      const accel = msg.linear_acceleration || {};
      const mag = Math.sqrt((accel.x || 0) ** 2 + (accel.y || 0) ** 2 + (accel.z || 0) ** 2);
      imuHealth.current.accelHistory = pushHistory(imuHealth.current.accelHistory, mag);
      setImuMsg({ ...msg });
      setImuHealthSnap({
        name: imuTopic,
        lastSeen: imuHealth.current.lastSeen,
        hz: imuHealth.current.hz,
        accelHistory: [...imuHealth.current.accelHistory],
      });
    });
    subs.push(imuSub);

    // Odometry
    const odomSub = new ROSLIB.Topic({
      ros,
      name: odomTopic,
      messageType: 'nav_msgs/Odometry',
      throttle_rate: 100,
    });
    odomSub.subscribe((msg) => {
      updateHz(odomHealth.current);
      const lv = msg.twist?.twist?.linear || {};
      const speed = Math.sqrt((lv.x || 0) ** 2 + (lv.y || 0) ** 2 + (lv.z || 0) ** 2);
      odomHealth.current.speedHistory = pushHistory(odomHealth.current.speedHistory, speed);
      setOdomMsg({ ...msg });
      setOdomHealthSnap({
        name: odomTopic,
        lastSeen: odomHealth.current.lastSeen,
        hz: odomHealth.current.hz,
        speedHistory: [...odomHealth.current.speedHistory],
      });
    });
    subs.push(odomSub);

    // Joint states
    const jointsSub = new ROSLIB.Topic({
      ros,
      name: jointsTopic,
      messageType: 'sensor_msgs/JointState',
      throttle_rate: 100,
    });
    jointsSub.subscribe((msg) => {
      updateHz(jointsHealth.current);
      setJointsMsg({ ...msg });
      setJointsHealthSnap({
        name: jointsTopic,
        lastSeen: jointsHealth.current.lastSeen,
        hz: jointsHealth.current.hz,
      });
    });
    subs.push(jointsSub);

    // LaserScan
    const scanSub = new ROSLIB.Topic({
      ros,
      name: scanTopic,
      messageType: 'sensor_msgs/LaserScan',
      throttle_rate: 200,
    });
    scanSub.subscribe((msg) => {
      updateHz(scanHealth.current);
      setScanMsg({ ...msg });
      setScanHealthSnap({
        name: scanTopic,
        lastSeen: scanHealth.current.lastSeen,
        hz: scanHealth.current.hz,
      });
    });
    subs.push(scanSub);

    return () => {
      subs.forEach((s) => {
        try { s.unsubscribe(); } catch (_) {}
      });
    };
  }, [ros, imuTopic, odomTopic, jointsTopic, scanTopic, updateHz]);

  // Stale-detection ticker — update snapshots every second so badges update
  useEffect(() => {
    const interval = setInterval(() => {
      setImuHealthSnap((prev) => ({
        ...prev,
        lastSeen: imuHealth.current.lastSeen,
        hz: imuHealth.current.hz,
      }));
      setOdomHealthSnap((prev) => ({
        ...prev,
        lastSeen: odomHealth.current.lastSeen,
        hz: odomHealth.current.hz,
      }));
      setJointsHealthSnap((prev) => ({
        ...prev,
        lastSeen: jointsHealth.current.lastSeen,
        hz: jointsHealth.current.hz,
      }));
      setScanHealthSnap((prev) => ({
        ...prev,
        lastSeen: scanHealth.current.lastSeen,
        hz: scanHealth.current.hz,
      }));
    }, 1000);
    return () => clearInterval(interval);
  }, []);

  const TABS = [
    { key: 'state',  label: 'Robot State' },
    { key: 'imu',    label: 'IMU' },
    { key: 'odom',   label: 'Odom' },
    { key: 'joints', label: 'Joints' },
    { key: 'lidar',  label: 'Lidar' },
  ];

  return (
    <div className="sensor-panel">
      <RosHealthBar rosConnected={rosConnected} rosBridgeUrl={rosBridgeUrl} />

      <div className="sensor-panel__tabs">
        {TABS.map(({ key, label }) => (
          <button
            key={key}
            className={`sensor-panel__tab ${tab === key ? 'sensor-panel__tab--active' : ''}`}
            onClick={() => setTab(key)}
          >
            {label}
          </button>
        ))}
      </div>

      <div className="sensor-panel__content">
        {tab === 'state'  && <StateSection  perception={policyPerception} safetyState={safetyState} />}
        {tab === 'imu'    && <ImuSection    imu={imuMsg}       topicHealth={imuHealthSnap}    />}
        {tab === 'odom'   && <OdomSection   odom={odomMsg}     topicHealth={odomHealthSnap}   />}
        {tab === 'joints' && <JointSection  joints={jointsMsg} topicHealth={jointsHealthSnap} />}
        {tab === 'lidar'  && <LidarSection  scan={scanMsg}     topicHealth={scanHealthSnap}   />}
      </div>
    </div>
  );
};

export default SensorPanel;
