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

// Serve static files from the dist directory
app.use(express.static(distPath));

// Track connected clients
let connectedClients = 0;

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
  
  socket.on('disconnect', () => {
    connectedClients--;
    console.log(`[${new Date().toISOString()}] Client disconnected - ID: ${clientId}, Remaining clients: ${connectedClients}`);
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
server.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
  console.log(`Open http://localhost:${PORT} in your browser`);
}); 