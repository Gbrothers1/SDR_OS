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
  });
});

// ── Connection tracking ──────────────────────────────────────────
let connectedClients = 0;

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

  // ── Disconnect ────────────────────────────────────────────────

  socket.on('disconnect', () => {
    connectedClients--;
    console.log(`[${new Date().toISOString()}] Client disconnected — ID: ${clientId}, Remaining: ${connectedClients}`);
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

// ── WebSocket proxy for /stream/ws → transport-server ────────────
// Forwards binary video/telemetry stream from transport-server to browser.
const TRANSPORT_HOST = process.env.TRANSPORT_HOST || 'localhost';
const TRANSPORT_PORT = process.env.TRANSPORT_PORT || 8080;

server.on('upgrade', (req, clientSocket, head) => {
  // Let Socket.io handle its own upgrade path
  if (req.url.startsWith('/socket.io')) return;

  if (req.url === '/stream/ws') {
    const options = {
      hostname: TRANSPORT_HOST,
      port: TRANSPORT_PORT,
      path: '/stream/ws',
      method: 'GET',
      headers: {
        ...req.headers,
        host: `${TRANSPORT_HOST}:${TRANSPORT_PORT}`,
      },
    };

    const proxyReq = http.request(options);
    proxyReq.on('upgrade', (proxyRes, proxySocket, proxyHead) => {
      // Send the 101 response back to the client
      clientSocket.write(
        'HTTP/1.1 101 Switching Protocols\r\n' +
        `upgrade: ${proxyRes.headers['upgrade']}\r\n` +
        `connection: ${proxyRes.headers['connection']}\r\n` +
        `sec-websocket-accept: ${proxyRes.headers['sec-websocket-accept']}\r\n` +
        '\r\n'
      );

      // Bidirectional pipe
      proxySocket.pipe(clientSocket);
      clientSocket.pipe(proxySocket);

      proxySocket.on('error', () => clientSocket.destroy());
      clientSocket.on('error', () => proxySocket.destroy());
    });

    proxyReq.on('error', (err) => {
      console.error(`[${new Date().toISOString()}] Stream proxy error: ${err.message}`);
      clientSocket.destroy();
    });

    proxyReq.end();
  }
});

// ── Start server ─────────────────────────────────────────────────
const PORT = process.env.PORT || 3000;
const HOST = process.env.HOST || '0.0.0.0';
server.listen(PORT, HOST, () => {
  console.log(`SDR_OS server running on ${HOST}:${PORT}`);
  console.log(`Stream proxy: /stream/ws → ${TRANSPORT_HOST}:${TRANSPORT_PORT}`);
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
