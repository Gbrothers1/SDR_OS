# Web-Based Robot Controller for Steam Deck

A web-based robot controller application that can be accessed from any device with a web browser, including the Steam Deck. This application provides:
- 3D visualization of robot telemetry
- Real-time control using device inputs
- ROS2 message monitoring and logging
- Wireless remote control capabilities

## Features
- Real-time 3D robot model visualization using Three.js
- Device-specific control overlay (optimized for Steam Deck)
- ROS2 topic monitoring and logging
- Wireless remote control interface
- Customizable control mappings
- Works on any device with a modern web browser

## Architecture
- Frontend: HTML5, CSS3, JavaScript (React.js, Three.js)
- Backend: Node.js with Express
- ROS2 Bridge: rosbridge-server
- Communication: WebSockets

## Requirements
- Node.js 14+ (for the web server)
- ROS2 (tested with Humble)
- rosbridge-server package
- Modern web browser

## Installation

1. Clone this repository
2. Install ROS2 dependencies:
```bash
sudo apt-get update
sudo apt-get install ros-humble-rosbridge-server
```

3. Install Node.js dependencies:
```bash
npm install
```

4. Start the ROS2 bridge:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

5. Start the web server:
```bash
npm start
```

## Usage

1. Open a web browser and navigate to `http://localhost:3000`
2. Configure your robot's ROS2 topics in the settings
3. Connect to your robot's ROS2 network
4. Use the device controls to operate the robot

## Configuration

The application can be configured through the settings menu:
- ROS2 topic mappings
- Control sensitivity
- 3D model customization
- UI layout preferences

## License

MIT License 