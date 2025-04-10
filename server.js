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

// Handle WebSocket connections
io.on('connection', (socket) => {
  console.log('Client connected');
  
  // Handle robot control commands
  socket.on('robot_control', (data) => {
    console.log('Robot control command:', data);
    // Broadcast to all connected clients
    io.emit('robot_control', data);
  });
  
  socket.on('disconnect', () => {
    console.log('Client disconnected');
  });
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
}); 