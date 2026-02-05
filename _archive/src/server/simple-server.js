const express = require('express');
const path = require('path');
const app = express();
const port = 3000;

// Serve static files from the client directory
app.use(express.static(path.join(__dirname, '../../client')));

// Parse JSON bodies
app.use(express.json());

// Serve the main HTML file
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '../../client/simple.html'));
});

// Handle robot control commands
app.post('/api/control', (req, res) => {
    const { action } = req.body;
    console.log(`Received control command: ${action}`);
    
    // Here we'll add the actual robot control logic later
    // For now, just acknowledge the command
    res.json({ success: true, message: `Command received: ${action}` });
});

app.listen(port, () => {
    console.log(`Server running at http://localhost:${port}`);
}); 