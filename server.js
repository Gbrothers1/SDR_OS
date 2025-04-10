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
app.get('*', (req, res) => {
  const indexPath = path.join(distPath, 'index.html');
  
  // Check if index.html exists, if not, create a simple one
  if (!fs.existsSync(indexPath)) {
    console.log('index.html not found, creating a simple one');
    const simpleHtml = `
      <!DOCTYPE html>
      <html>
        <head>
          <title>Robot Controller</title>
          <style>
            body { font-family: Arial, sans-serif; text-align: center; padding: 50px; }
            h1 { color: #333; }
            p { color: #666; }
          </style>
        </head>
        <body>
          <h1>Robot Controller</h1>
          <p>Please run 'npm run build' to build the application.</p>
        </body>
      </html>
    `;
    fs.writeFileSync(indexPath, simpleHtml);
  }
  
  res.sendFile(indexPath);
});

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Server running on port ${PORT}`);
  console.log(`Open http://localhost:${PORT} in your browser`);
}); 