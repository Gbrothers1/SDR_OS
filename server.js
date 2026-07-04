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

const POLICY_ROOTS = new Map();
const policyRootSpec = process.env.SDR_POLICY_ROOTS || 'workspace=/policy-library/workspace';
for (const entry of policyRootSpec.split(',')) {
  const separator = entry.indexOf('=');
  if (separator <= 0) continue;
  const label = entry.slice(0, separator).trim();
  const root = entry.slice(separator + 1).trim();
  if (label && root) POLICY_ROOTS.set(label, root);
}

const VIDEO_TYPES = new Map([
  ['.mp4', 'video/mp4'],
  ['.webm', 'video/webm'],
  ['.mov', 'video/quicktime'],
  ['.m4v', 'video/x-m4v'],
]);

function isSafePathSegment(value) {
  return (
    typeof value === 'string' &&
    value.length > 0 &&
    value !== '.' &&
    value !== '..' &&
    !value.includes('/') &&
    !value.includes('\\') &&
    !value.includes('\0')
  );
}

async function resolvePolicyDirectory(source, policyName) {
  if (!isSafePathSegment(source) || !isSafePathSegment(policyName)) {
    const error = new Error('Invalid policy identifier');
    error.status = 400;
    throw error;
  }

  const configuredRoot = POLICY_ROOTS.get(source);
  if (!configuredRoot) {
    const error = new Error(`Unknown policy source: ${source}`);
    error.status = 404;
    throw error;
  }

  let root;
  let policyDir;
  try {
    root = await fs.promises.realpath(configuredRoot);
    policyDir = await fs.promises.realpath(path.join(root, policyName));
  } catch (cause) {
    const error = new Error('Policy artifacts not found');
    error.status = 404;
    error.cause = cause;
    throw error;
  }

  if (policyDir !== root && !policyDir.startsWith(`${root}${path.sep}`)) {
    const error = new Error('Policy path is outside the configured library');
    error.status = 403;
    throw error;
  }

  const stat = await fs.promises.stat(policyDir);
  if (!stat.isDirectory()) {
    const error = new Error('Policy artifact is not a directory');
    error.status = 404;
    throw error;
  }

  return policyDir;
}

function extractStep(name) {
  const match = name.match(/(\d+)(?=[^0-9]*\.pt$)/i);
  return match ? Number(match[1]) : null;
}

function compareCheckpoints(a, b) {
  if (a.step != null && b.step != null && a.step !== b.step) {
    return b.step - a.step;
  }
  if (a.step != null) return -1;
  if (b.step != null) return 1;
  return b.modifiedMs - a.modifiedMs || a.name.localeCompare(b.name);
}

async function describeFiles(directory, entries, kind) {
  const artifacts = [];
  const batchSize = 64;

  for (let index = 0; index < entries.length; index += batchSize) {
    const batch = entries.slice(index, index + batchSize);
    const described = await Promise.all(batch.map(async (entry) => {
      const stat = await fs.promises.stat(path.join(directory, entry.name));
      return {
        name: entry.name,
        size_bytes: stat.size,
        modified_iso: stat.mtime.toISOString(),
        modifiedMs: stat.mtimeMs,
        ...(kind === 'checkpoint' ? { step: extractStep(entry.name) } : {}),
      };
    }));
    artifacts.push(...described);
  }

  return artifacts;
}

function mediaUrl(source, policyName, fileName) {
  return `/api/lab/media/${encodeURIComponent(source)}/${encodeURIComponent(policyName)}/${encodeURIComponent(fileName)}`;
}

// ── Policy tags store ─────────────────────────────────────────────
// Central JSON map {policyName: [tags]} on a writable mount (the policy
// roots are read-only). Keyed by run NAME so the same run mirrored from
// several sources shares one tag set. The sim runner reads the same file
// (host: artifacts/lab/policy_tags.json) and attaches tags to the policy
// list, so tags edited in the Lab appear in the Policy Library panel.
const TAGS_DIR = process.env.SDR_LAB_DATA || '/lab-data';
const TAGS_PATH = path.join(TAGS_DIR, 'policy_tags.json');

function readTags() {
  try {
    const parsed = JSON.parse(fs.readFileSync(TAGS_PATH, 'utf8'));
    return parsed && typeof parsed === 'object' && !Array.isArray(parsed) ? parsed : {};
  } catch {
    return {};
  }
}

function writeTags(tags) {
  fs.mkdirSync(TAGS_DIR, { recursive: true });
  const tmpPath = `${TAGS_PATH}.tmp`;
  fs.writeFileSync(tmpPath, `${JSON.stringify(tags, null, 2)}\n`);
  fs.renameSync(tmpPath, TAGS_PATH);
}

function normalizeTags(rawTags) {
  if (!Array.isArray(rawTags)) return null;
  const cleaned = [];
  for (const tag of rawTags) {
    if (typeof tag !== 'string') return null;
    const trimmed = tag.trim().toLowerCase().slice(0, 40);
    if (trimmed && !cleaned.includes(trimmed)) cleaned.push(trimmed);
    if (cleaned.length >= 20) break;
  }
  return cleaned;
}

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
    policySources: [...POLICY_ROOTS.keys()],
  });
});

// ── Policy tags ───────────────────────────────────────────────────
app.get('/api/lab/tags', (req, res) => {
  res.setHeader('Cache-Control', 'no-store');
  res.json({ tags: readTags() });
});

app.put('/api/lab/tags/:policy', (req, res) => {
  const policyName = req.params.policy;
  if (!isSafePathSegment(policyName)) {
    return res.status(400).json({ error: 'Invalid policy name' });
  }
  const tags = normalizeTags(req.body && req.body.tags);
  if (tags === null) {
    return res.status(400).json({ error: 'tags must be an array of strings' });
  }
  try {
    const all = readTags();
    if (tags.length > 0) {
      all[policyName] = tags;
    } else {
      delete all[policyName];
    }
    writeTags(all);
    res.json({ name: policyName, tags });
  } catch (error) {
    console.error(`[${new Date().toISOString()}] Tag write error:`, error);
    res.status(500).json({ error: 'Failed to persist tags' });
  }
});

// ── Lab artifact inspection ───────────────────────────────────────
app.get('/api/lab/policies/:source/:policy', async (req, res) => {
  try {
    const { source, policy: policyName } = req.params;
    const policyDir = await resolvePolicyDirectory(source, policyName);
    const entries = await fs.promises.readdir(policyDir, { withFileTypes: true });
    const checkpointEntries = entries.filter(
      (entry) => entry.isFile() && path.extname(entry.name).toLowerCase() === '.pt'
    );

    const videosDir = path.join(policyDir, 'videos');
    let videoEntries = [];
    try {
      const entriesInVideos = await fs.promises.readdir(videosDir, { withFileTypes: true });
      videoEntries = entriesInVideos.filter(
        (entry) => entry.isFile() && VIDEO_TYPES.has(path.extname(entry.name).toLowerCase())
      );
    } catch (error) {
      if (error.code !== 'ENOENT') throw error;
    }

    const [checkpoints, videos] = await Promise.all([
      describeFiles(policyDir, checkpointEntries, 'checkpoint'),
      describeFiles(videosDir, videoEntries, 'video'),
    ]);

    checkpoints.sort(compareCheckpoints);
    videos.sort((a, b) => b.modifiedMs - a.modifiedMs || a.name.localeCompare(b.name));

    const checkpointBytes = checkpoints.reduce((total, item) => total + item.size_bytes, 0);
    const videoBytes = videos.reduce((total, item) => total + item.size_bytes, 0);

    res.setHeader('Cache-Control', 'no-store');
    res.json({
      source,
      name: policyName,
      checkpoint_count: checkpoints.length,
      checkpoint_bytes: checkpointBytes,
      video_count: videos.length,
      video_bytes: videoBytes,
      checkpoints: checkpoints.map(({ modifiedMs, ...item }) => item),
      videos: videos.map(({ modifiedMs, ...item }) => ({
        ...item,
        url: mediaUrl(source, policyName, item.name),
      })),
    });
  } catch (error) {
    const status = error.status || 500;
    if (status >= 500) {
      console.error(`[${new Date().toISOString()}] Lab artifact error:`, error);
    }
    res.status(status).json({ error: error.message || 'Failed to inspect policy artifacts' });
  }
});

app.get('/api/lab/media/:source/:policy/:file', async (req, res) => {
  try {
    const { source, policy: policyName, file } = req.params;
    if (!isSafePathSegment(file)) {
      return res.status(400).json({ error: 'Invalid media file' });
    }

    const extension = path.extname(file).toLowerCase();
    const contentType = VIDEO_TYPES.get(extension);
    if (!contentType) {
      return res.status(415).json({ error: 'Unsupported media type' });
    }

    const policyDir = await resolvePolicyDirectory(source, policyName);
    const videosDir = await fs.promises.realpath(path.join(policyDir, 'videos'));
    const mediaPath = await fs.promises.realpath(path.join(videosDir, file));
    if (!mediaPath.startsWith(`${videosDir}${path.sep}`)) {
      return res.status(403).json({ error: 'Media path is outside the policy directory' });
    }

    const stat = await fs.promises.stat(mediaPath);
    if (!stat.isFile()) {
      return res.status(404).json({ error: 'Media file not found' });
    }

    const range = req.headers.range;
    res.setHeader('Accept-Ranges', 'bytes');
    res.setHeader('Content-Type', contentType);
    res.setHeader('Cache-Control', 'private, max-age=3600');

    if (!range) {
      res.setHeader('Content-Length', stat.size);
      fs.createReadStream(mediaPath).pipe(res);
      return;
    }

    const match = range.match(/^bytes=(\d*)-(\d*)$/);
    if (!match) {
      res.setHeader('Content-Range', `bytes */${stat.size}`);
      return res.sendStatus(416);
    }

    let start = match[1] ? Number(match[1]) : 0;
    let end = match[2] ? Number(match[2]) : stat.size - 1;
    if (!match[1] && match[2]) {
      const suffixLength = Number(match[2]);
      start = Math.max(stat.size - suffixLength, 0);
      end = stat.size - 1;
    }

    if (
      !Number.isInteger(start) ||
      !Number.isInteger(end) ||
      start < 0 ||
      end < start ||
      start >= stat.size
    ) {
      res.setHeader('Content-Range', `bytes */${stat.size}`);
      return res.sendStatus(416);
    }

    end = Math.min(end, stat.size - 1);
    res.status(206);
    res.setHeader('Content-Range', `bytes ${start}-${end}/${stat.size}`);
    res.setHeader('Content-Length', end - start + 1);
    fs.createReadStream(mediaPath, { start, end }).pipe(res);
  } catch (error) {
    const status = error.status || (error.code === 'ENOENT' ? 404 : 500);
    if (status >= 500) {
      console.error(`[${new Date().toISOString()}] Lab media error:`, error);
    }
    if (!res.headersSent) {
      res.status(status).json({ error: error.message || 'Failed to read media file' });
    } else {
      res.destroy(error);
    }
  }
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
