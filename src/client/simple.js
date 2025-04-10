// Simple JavaScript file
console.log('Simple.js loaded successfully!');

// Add a simple message to the page
document.addEventListener('DOMContentLoaded', () => {
    const body = document.body;
    const message = document.createElement('div');
    message.style.color = 'white';
    message.style.fontSize = '24px';
    message.style.textAlign = 'center';
    message.style.marginTop = '50px';
    message.textContent = 'Robot Controller (Simple Version)';
    body.appendChild(message);
});

// Simple version of the robot controller that doesn't rely on React
document.addEventListener('DOMContentLoaded', () => {
  // Create a simple UI
  const container = document.createElement('div');
  container.style.cssText = `
    display: flex;
    width: 100%;
    height: 100vh;
    background-color: #121212;
    color: white;
    font-family: Arial, sans-serif;
  `;

  // Create 3D viewer
  const viewer = document.createElement('div');
  viewer.style.cssText = `
    flex: 2;
    position: relative;
    background-color: #1a1a1a;
  `;
  container.appendChild(viewer);

  // Create log panel
  const logPanel = document.createElement('div');
  logPanel.style.cssText = `
    flex: 1;
    border-left: 1px solid #333;
    padding: 10px;
    overflow-y: auto;
  `;
  container.appendChild(logPanel);

  // Add log header
  const logHeader = document.createElement('div');
  logHeader.style.cssText = `
    padding: 10px;
    border-bottom: 1px solid #333;
    display: flex;
    justify-content: space-between;
    align-items: center;
  `;
  
  const logTitle = document.createElement('h3');
  logTitle.textContent = 'Logs';
  logHeader.appendChild(logTitle);
  
  const clearButton = document.createElement('button');
  clearButton.textContent = 'Clear';
  clearButton.style.cssText = `
    background: #444;
    color: white;
    border: none;
    padding: 5px 10px;
    border-radius: 3px;
    cursor: pointer;
  `;
  clearButton.addEventListener('click', () => {
    logContent.innerHTML = '';
  });
  logHeader.appendChild(clearButton);
  
  logPanel.appendChild(logHeader);

  // Add log content
  const logContent = document.createElement('div');
  logContent.style.cssText = `
    padding: 10px;
    font-family: monospace;
    font-size: 12px;
  `;
  logPanel.appendChild(logContent);

  // Add control overlay
  const overlay = document.createElement('div');
  overlay.style.cssText = `
    position: absolute;
    bottom: 20px;
    left: 20px;
    background: rgba(0, 0, 0, 0.5);
    padding: 10px;
    border-radius: 5px;
    color: white;
    font-size: 12px;
  `;
  overlay.innerHTML = `
    <div>Left Stick: <span id="left-stick">X: 0.00, Y: 0.00</span></div>
    <div>Right Stick: <span id="right-stick">X: 0.00, Y: 0.00</span></div>
    <div>Triggers: <span id="triggers">L2: 0%, R2: 0%</span></div>
  `;
  viewer.appendChild(overlay);

  // Add to document
  document.body.appendChild(container);

  // Add a message to the log
  function addLog(message) {
    const logEntry = document.createElement('div');
    logEntry.style.marginBottom = '5px';
    logEntry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    logContent.appendChild(logEntry);
    logContent.scrollTop = logContent.scrollHeight;
  }

  // Add initial log message
  addLog('Robot Controller initialized');

  // Try to connect to ROS
  try {
    // This is a placeholder for ROS connection
    // In a real implementation, you would use roslib to connect to ROS
    addLog('Attempting to connect to ROS...');
    
    // Simulate connection after a delay
    setTimeout(() => {
      addLog('Connected to ROS bridge');
    }, 1000);
  } catch (error) {
    addLog(`Error connecting to ROS: ${error.message}`);
  }

  // Handle gamepad input
  let gamepad = null;
  
  function handleGamepadConnected(e) {
    gamepad = e.gamepad;
    addLog(`Gamepad connected: ${gamepad.id}`);
  }
  
  function handleGamepadDisconnected(e) {
    if (gamepad && gamepad.index === e.gamepad.index) {
      addLog(`Gamepad disconnected: ${gamepad.id}`);
      gamepad = null;
    }
  }
  
  window.addEventListener('gamepadconnected', handleGamepadConnected);
  window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);
  
  // Poll gamepad
  function pollGamepad() {
    if (gamepad) {
      const gamepads = navigator.getGamepads();
      const currentGamepad = gamepads[gamepad.index];
      
      if (currentGamepad) {
        // Update stick values
        const leftStickX = currentGamepad.axes[0].toFixed(2);
        const leftStickY = currentGamepad.axes[1].toFixed(2);
        const rightStickX = currentGamepad.axes[2].toFixed(2);
        const rightStickY = currentGamepad.axes[3].toFixed(2);
        
        document.getElementById('left-stick').textContent = `X: ${leftStickX}, Y: ${leftStickY}`;
        document.getElementById('right-stick').textContent = `X: ${rightStickX}, Y: ${rightStickY}`;
        
        // Update trigger values
        const l2 = ((currentGamepad.buttons[6].value) * 100).toFixed(0);
        const r2 = ((currentGamepad.buttons[7].value) * 100).toFixed(0);
        
        document.getElementById('triggers').textContent = `L2: ${l2}%, R2: ${r2}%`;
      }
    }
    
    requestAnimationFrame(pollGamepad);
  }
  
  pollGamepad();
}); 