const express = require('express');
const path = require('path');
const app = express();
const port = 3000;

// Serve static files from the client directory
app.use(express.static(path.join(__dirname, '../client')));

// Start the server
app.listen(port, () => {
  console.log(`Simple server running at http://localhost:${port}`);
  console.log(`Open http://localhost:${port}/simple.html in your browser`);
}); 