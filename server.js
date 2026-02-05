const express = require('express');
const path = require('path');
const http = require('http');
const socketIo = require('socket.io');
const fs = require('fs');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

// Create dist directory if it doesn't exist
const distPath = path.join(__dirname, 'dist');
if (!fs.existsSync(distPath)) {
  fs.mkdirSync(distPath, { recursive: true });
  console.log('Created dist directory');
}

// Serve static files from the dist directory with no-cache for development
app.use(express.static(distPath, {
  etag: false,
  lastModified: false,
  setHeaders: (res, path) => {
    // Disable caching for JS/CSS bundles during development
    if (path.endsWith('.js') || path.endsWith('.css')) {
      res.setHeader('Cache-Control', 'no-cache, no-store, must-revalidate');
      res.setHeader('Pragma', 'no-cache');
      res.setHeader('Expires', '0');
    }
  }
}));

// Serve configs directory for URDF registry and other config files
const configsPath = path.join(__dirname, 'configs');
app.use('/configs', express.static(configsPath));

// Serve assets directory for URDF files
const assetsPath = path.join(__dirname, 'assets');
app.use('/assets', express.static(assetsPath));

// Track connected clients
let connectedClients = 0;
let genesisBridgeConnected = false;

// Genesis events to forward between clients
const GENESIS_EVENTS = [
  'genesis_load_robot',
  'genesis_load_script',
  'genesis_reset',
  'genesis_pause',
  'genesis_set_mode',
  'genesis_select_env',
  'genesis_camera',
  'genesis_set_goal',
  'genesis_set_alpha',
  'genesis_status',
  'genesis_env_info',
  'genesis_robot_list',
  'genesis_script_status',
  'genesis_frame_stats',
  'genesis_training_metrics',
  'genesis_scan_robots',
  'genesis_get_robot_info',
  'genesis_robot_info',
  'genesis_get_camera_list',
  'genesis_camera_list',
  'genesis_set_control_mode',
  'load_profile',
  'genesis_obs_breakdown',
  'genesis_reward_breakdown',
  'genesis_velocity_command',
  'genesis_estop',
  'genesis_set_command_source',
  'genesis_load_policy',
  'genesis_list_policies',
  'genesis_policy_list',
  'genesis_policy_load_status',
  'genesis_settings',
  'genesis_unload_robot',
  'genesis_robot_loaded',
  'genesis_robot_unloaded',
  'genesis_robot_load_failed',
  'genesis_memory_estimate',
  'genesis_init_status',
  'genesis_get_memory_estimate',
  'genesis_get_init_status',
  'genesis_skill_command',
  'genesis_mapping_type',
];

// Handle WebSocket connections
io.on('connection', (socket) => {
  connectedClients++;
  const clientId = socket.id.substring(0, 8);
  const clientIp = socket.handshake.address;
  const timestamp = new Date().toISOString();
  
  console.log(`[${timestamp}] Client connected - ID: ${clientId}, IP: ${clientIp}, Total clients: ${connectedClients}`);
  
  // Handle robot control commands
  socket.on('robot_control', (data) => {
    console.log(`[${new Date().toISOString()}] Robot control command from ${clientId}:`, data);
    // Broadcast to all connected clients
    io.emit('robot_control', data);
  });
  
  // Handle controller button states with low latency
  socket.on('controller_button_states', (buttonStates) => {
    const now = Date.now();
    // One-time shape log for debugging
    if (!socket._loggedButtonShape) {
      try {
        console.log(
          `[${new Date().toISOString()}] Button payload from ${clientId}: ` +
          `${typeof buttonStates} keys=${Object.keys(buttonStates || {}).join(',')}`
        );
        console.log(
          `[${new Date().toISOString()}] Button payload sample from ${clientId}: ` +
          `${JSON.stringify(buttonStates)}`
        );
      } catch (e) {
        console.log(`[${new Date().toISOString()}] Button payload logging failed: ${e}`);
      }
      socket._loggedButtonShape = true;
    }

    // Throttle per-socket spam to keep event loop responsive
    if (!socket._lastButtonLog || now - socket._lastButtonLog > 1000) {
      console.log(`[${new Date().toISOString()}] Button tick from ${clientId}`);
      socket._lastButtonLog = now;
    }

    const l1Value = (
      buttonStates &&
      typeof buttonStates === 'object' &&
      Object.prototype.hasOwnProperty.call(buttonStates, 'L1')
    ) ? buttonStates.L1 : undefined;
    if (l1Value !== undefined && socket.lastL1State !== l1Value) {
      console.log(
        `[${new Date().toISOString()}] L1 state from ${clientId}: ${l1Value}`
      );
      socket.lastL1State = l1Value;
    }
    // Broadcast button states to ALL clients including Genesis bridge
    io.emit('controller_button_states', buttonStates);
  });
  
  // Handle joystick state updates with low latency
  socket.on('controller_joystick_state', (joystickState) => {
    const now = Date.now();
    if (!socket._lastJoystickLog || now - socket._lastJoystickLog > 1000) {
      console.log(`[${new Date().toISOString()}] Joystick tick from ${clientId}`);
      socket._lastJoystickLog = now;
    }
    // Broadcast joystick state to ALL clients including Genesis bridge
    io.volatile.emit('controller_joystick_state', joystickState);
  });

  // Forward controller mapping type to all clients (including bridge)
  socket.on('controller_mapping_type', (data) => {
    console.log(`[${new Date().toISOString()}] Controller mapping type from ${clientId}:`, data.type);
    io.emit('controller_mapping_type', data);
  });

  // Forward controller haptics/vibration commands to all clients
  socket.on('controller_vibration', (payload) => {
    io.emit('controller_vibration', payload);
  });

  // Genesis bridge identification
  socket.on('genesis_identify', (data) => {
    console.log(`[${new Date().toISOString()}] Genesis bridge identified - ID: ${clientId}`);
    genesisBridgeConnected = true;
    socket.isGenesisBridge = true;
    // Broadcast status to all clients
    io.emit('genesis_status', { connected: true, bridge_id: clientId });
  });
  
  // Forward Genesis events to all clients
  GENESIS_EVENTS.forEach(eventName => {
    socket.on(eventName, (data) => {
      console.log(`[${new Date().toISOString()}] Genesis event: ${eventName} from ${clientId}`);
      // Broadcast to all other clients
      socket.broadcast.emit(eventName, data);
    });
  });
  
  socket.on('disconnect', () => {
    connectedClients--;
    console.log(`[${new Date().toISOString()}] Client disconnected - ID: ${clientId}, Remaining clients: ${connectedClients}`);
    
    // If Genesis bridge disconnected, notify all clients
    if (socket.isGenesisBridge) {
      genesisBridgeConnected = false;
      io.emit('genesis_status', { connected: false });
      console.log(`[${new Date().toISOString()}] Genesis bridge disconnected`);
    }
  });
});

// Log HTTP requests
app.use((req, res, next) => {
  const timestamp = new Date().toISOString();
  console.log(`[${timestamp}] HTTP ${req.method} ${req.url}`);
  next();
});

// Serve the main application for all routes
app.get('/', (req, res) => {
  res.header('Cache-Control', 'no-cache, no-store, must-revalidate');
  res.header('Pragma', 'no-cache');
  res.header('Expires', '0');
  res.sendFile(path.join(distPath, 'index.html'));
});

const PORT = process.env.PORT || 3000;
const HOST = process.env.HOST || '0.0.0.0';
server.listen(PORT, HOST, () => {
  console.log(`Server running on ${HOST}:${PORT}`);
  console.log(`Open http://localhost:${PORT} in your browser`);
}); 
