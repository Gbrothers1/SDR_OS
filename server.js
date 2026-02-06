const express = require('express');
const path = require('path');
const http = require('http');
const socketIo = require('socket.io');
const fs = require('fs');

const app = express();
const server = http.createServer(app);
const io = socketIo(server, {
  cors: {
    origin: '*',
    methods: ['GET', 'POST'],
  },
});

// ── JSON body parsing for future API routes ──────────────────────
app.use(express.json());

// ── HTTP request logger (BEFORE static middleware so it runs first) ──
app.use((req, res, next) => {
  // Skip noisy asset requests in dev
  if (req.url.endsWith('.js') || req.url.endsWith('.css') || req.url.endsWith('.map')) {
    return next();
  }
  const timestamp = new Date().toISOString();
  console.log(`[${timestamp}] HTTP ${req.method} ${req.url}`);
  next();
});

// ── Create dist directory if it doesn't exist ────────────────────
const distPath = path.join(__dirname, 'dist');
if (!fs.existsSync(distPath)) {
  fs.mkdirSync(distPath, { recursive: true });
  console.log('Created dist directory');
}

// ── Static file serving ──────────────────────────────────────────
// dist/ — Webpack-built SPA (bundle.js, index.html)
app.use(express.static(distPath, {
  etag: false,
  lastModified: false,
  setHeaders: (res, filePath) => {
    if (filePath.endsWith('.js') || filePath.endsWith('.css')) {
      res.setHeader('Cache-Control', 'no-cache, no-store, must-revalidate');
      res.setHeader('Pragma', 'no-cache');
      res.setHeader('Expires', '0');
    }
  }
}));

// configs/ — URDF registry, robot YAML configs
const configsPath = path.join(__dirname, 'configs');
app.use('/configs', express.static(configsPath));

// assets/ — URDF files, 3D models (DAE, OBJ, MTL)
const assetsPath = path.join(__dirname, 'assets');
app.use('/assets', express.static(assetsPath));

// ── Health / status endpoint (used by Docker Compose healthcheck) ─
app.get('/api/status', (req, res) => {
  res.json({
    ok: true,
    uptime: process.uptime(),
    clients: connectedClients,
    genesisBridge: genesisBridgeConnected,
  });
});

// ── Connection tracking ──────────────────────────────────────────
let connectedClients = 0;
let genesisBridgeConnected = false;

// ── Genesis events to forward between clients ────────────────────
// Every event in this list is forwarded via socket.broadcast.emit
// when received from any client (browser or Genesis bridge).
const GENESIS_EVENTS = [
  // Simulation control
  'genesis_load_robot',
  'genesis_unload_robot',
  'genesis_load_script',
  'genesis_reset',
  'genesis_pause',
  'genesis_set_mode',
  'genesis_select_env',
  'genesis_camera',
  'genesis_set_goal',
  'genesis_set_alpha',
  'genesis_set_control_mode',
  'genesis_velocity_command',
  'genesis_estop',
  'genesis_set_command_source',
  'genesis_skill_command',
  'genesis_mapping_type',

  // Status and info
  'genesis_status',
  'genesis_env_info',
  'genesis_robot_list',
  'genesis_robot_info',
  'genesis_robot_loaded',
  'genesis_robot_unloaded',
  'genesis_robot_load_failed',
  'genesis_memory_estimate',
  'genesis_init_status',
  'genesis_get_memory_estimate',
  'genesis_get_init_status',
  'genesis_get_robot_info',
  'genesis_scan_robots',

  // Training and metrics
  'genesis_training_metrics',
  'genesis_obs_breakdown',
  'genesis_reward_breakdown',
  'genesis_frame_stats',
  'genesis_load_policy',
  'genesis_list_policies',
  'genesis_policy_list',
  'genesis_policy_load_status',

  // Camera
  'genesis_get_camera_list',
  'genesis_camera_list',

  // Scripts, profiles, settings
  'genesis_script_status',
  'load_profile',
  'genesis_settings',

  // Training panel (session CRUD, checkpoints, demos, environments)
  'genesis_create_session',
  'genesis_clone_session',
  'genesis_delete_session',
  'genesis_set_param',
  'genesis_save_checkpoint',
  'genesis_load_checkpoint',
  'genesis_list_checkpoints',
  'genesis_save_episode_highlight',
  'genesis_tag_demo',
  'genesis_trim_demo',
  'genesis_export_dataset',
  'genesis_list_environments',
  'genesis_import_environment',
  'genesis_set_environment',
  'genesis_start_slam',
  'genesis_stop_slam',
  'genesis_drop_anchor',
  'genesis_export_map',
];

// ── Socket.io connection handling ────────────────────────────────
io.on('connection', (socket) => {
  connectedClients++;
  const clientId = socket.id.substring(0, 8);
  const clientIp = socket.handshake.address;
  const timestamp = new Date().toISOString();

  console.log(`[${timestamp}] Client connected — ID: ${clientId}, IP: ${clientIp}, Total: ${connectedClients}`);

  // ── Controller events (gamepad relay) ─────────────────────────

  socket.on('robot_control', (data) => {
    console.log(`[${new Date().toISOString()}] Robot control from ${clientId}:`, data);
    io.emit('robot_control', data);
  });

  // Button states — reliable transport
  socket.on('controller_button_states', (buttonStates) => {
    const now = Date.now();

    // One-time shape log for debugging
    if (!socket._loggedButtonShape) {
      try {
        console.log(
          `[${new Date().toISOString()}] Button payload from ${clientId}: ` +
          `${typeof buttonStates} keys=${Object.keys(buttonStates || {}).join(',')}`
        );
      } catch (e) { /* ignore */ }
      socket._loggedButtonShape = true;
    }

    // Throttled tick log (1/sec)
    if (!socket._lastButtonLog || now - socket._lastButtonLog > 1000) {
      console.log(`[${new Date().toISOString()}] Button tick from ${clientId}`);
      socket._lastButtonLog = now;
    }

    // Track L1 (deadman) state changes
    const l1Value = (
      buttonStates &&
      typeof buttonStates === 'object' &&
      Object.prototype.hasOwnProperty.call(buttonStates, 'L1')
    ) ? buttonStates.L1 : undefined;
    if (l1Value !== undefined && socket.lastL1State !== l1Value) {
      console.log(`[${new Date().toISOString()}] L1 state from ${clientId}: ${l1Value}`);
      socket.lastL1State = l1Value;
    }

    io.emit('controller_button_states', buttonStates);
  });

  // Joystick state — volatile (drops stale frames, critical for low-latency control)
  socket.on('controller_joystick_state', (joystickState) => {
    const now = Date.now();
    if (!socket._lastJoystickLog || now - socket._lastJoystickLog > 1000) {
      console.log(`[${new Date().toISOString()}] Joystick tick from ${clientId}`);
      socket._lastJoystickLog = now;
    }
    io.volatile.emit('controller_joystick_state', joystickState);
  });

  // Controller mapping type (Steam Deck vs Xbox auto-detection)
  socket.on('controller_mapping_type', (data) => {
    console.log(`[${new Date().toISOString()}] Controller mapping type from ${clientId}:`, data.type);
    io.emit('controller_mapping_type', data);
  });

  // Haptics / vibration
  socket.on('controller_vibration', (payload) => {
    io.emit('controller_vibration', payload);
  });

  // ── Genesis bridge identification protocol ────────────────────

  socket.on('genesis_identify', (data) => {
    console.log(`[${new Date().toISOString()}] Genesis bridge identified — ID: ${clientId}`);
    genesisBridgeConnected = true;
    socket.isGenesisBridge = true;
    io.emit('genesis_status', { connected: true, bridge_id: clientId });
  });

  // ── Forward all Genesis events between clients ────────────────

  GENESIS_EVENTS.forEach(eventName => {
    socket.on(eventName, (data) => {
      console.log(`[${new Date().toISOString()}] Genesis event: ${eventName} from ${clientId}`);
      socket.broadcast.emit(eventName, data);
    });
  });

  // ── Disconnect ────────────────────────────────────────────────

  socket.on('disconnect', () => {
    connectedClients--;
    console.log(`[${new Date().toISOString()}] Client disconnected — ID: ${clientId}, Remaining: ${connectedClients}`);

    if (socket.isGenesisBridge) {
      genesisBridgeConnected = false;
      io.emit('genesis_status', { connected: false });
      console.log(`[${new Date().toISOString()}] Genesis bridge disconnected`);
    }
  });
});

// ── SPA fallback — serve index.html for all unmatched routes ─────
// This must come AFTER static middleware and API routes.
app.get('*', (req, res) => {
  res.setHeader('Cache-Control', 'no-cache, no-store, must-revalidate');
  res.setHeader('Pragma', 'no-cache');
  res.setHeader('Expires', '0');
  res.sendFile(path.join(distPath, 'index.html'));
});

// ── Start server ─────────────────────────────────────────────────
const PORT = process.env.PORT || 3000;
const HOST = process.env.HOST || '0.0.0.0';
server.listen(PORT, HOST, () => {
  console.log(`SDR_OS server running on ${HOST}:${PORT}`);
  console.log(`Open http://localhost:${PORT} in your browser`);
});

// ── Graceful shutdown ────────────────────────────────────────────
function shutdown(signal) {
  console.log(`\n[${new Date().toISOString()}] ${signal} received — shutting down`);
  io.close(() => {
    server.close(() => {
      console.log('Server closed');
      process.exit(0);
    });
  });
  // Force exit after 5s if something hangs
  setTimeout(() => process.exit(1), 5000);
}

process.on('SIGTERM', () => shutdown('SIGTERM'));
process.on('SIGINT', () => shutdown('SIGINT'));
