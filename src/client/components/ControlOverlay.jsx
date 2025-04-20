import React, { useEffect, useRef, useState } from 'react';
import '../styles/ControlOverlay.css';
import ROSLIB from 'roslib';

const ControlOverlay = ({ onControlChange, controlState, ros, socket }) => {
  const gamepadRef = useRef(null);
  const animationFrameRef = useRef();
  const [buttonStates, setButtonStates] = useState({
    // Face buttons
    A: false, B: false, X: false, Y: false,
    // D-pad
    DpadUp: false, DpadDown: false, DpadLeft: false, DpadRight: false,
    // Shoulder buttons
    L1: false, L2: false, L3: false, L4: false,
    R1: false, R2: false, R3: false, R4: false,
  });
  const [showMappings, setShowMappings] = useState(false);
  const [isHidden, setIsHidden] = useState(true);
  const [isGamepadConnected, setIsGamepadConnected] = useState(false);
  // Store local controlState for non-gamepad clients
  const [localControlState, setLocalControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });

  // ROS mapping information with cmd_vel topic names
  const rosMappings = {
    leftStick: {
      x: "cmd_vel.linear.x - Forward/Backward movement",
      y: "cmd_vel.angular.z - Rotation (turning)"
    },
    rightStick: {
      x: "cmd_vel.linear.y - Lateral movement (strafing)",
      y: "cmd_vel.angular.x - Pitch control"
    },
    triggers: {
      left: "cmd_vel.linear.z - Up/Down movement",
      right: "cmd_vel.angular.y - Roll control"
    },
    dpad: {
      up: "cmd_vel.linear.x (positive) - Forward",
      down: "cmd_vel.linear.x (negative) - Backward",
      left: "cmd_vel.angular.z (positive) - Turn left",
      right: "cmd_vel.angular.z (negative) - Turn right"
    },
    buttons: {
      A: "cmd_vel.linear.z (positive) - Ascend",
      B: "cmd_vel.linear.z (negative) - Descend",
      X: "cmd_vel.linear.y (positive) - Strafe right",
      Y: "cmd_vel.linear.y (negative) - Strafe left"
    }
  };

  useEffect(() => {
    const handleGamepadConnected = (e) => {
      console.log('Gamepad connected:', e.gamepad);
      gamepadRef.current = e.gamepad;
      setIsGamepadConnected(true);
    };

    const handleGamepadDisconnected = (e) => {
      console.log('Gamepad disconnected:', e.gamepad);
      if (gamepadRef.current && gamepadRef.current.index === e.gamepad.index) {
        gamepadRef.current = null;
        setIsGamepadConnected(false);
      }
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    // Subscribe to ROS topics for controller state updates
    let telemetrySubscriber = null;
    let buttonStatePublisher = null;
    let joystickStatePublisher = null;

    if (ros) {
      // All clients subscribe to telemetry for a complete state picture
      telemetrySubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/robot/telemetry/all',
        messageType: 'std_msgs/String'
      });

      telemetrySubscriber.subscribe((message) => {
        try {
          const data = JSON.parse(message.data);
          // If telemetry includes controller state data, use it (unless we have our own gamepad)
          if (data.controller && !isGamepadConnected) {
            if (data.controller.buttonStates) {
              setButtonStates(data.controller.buttonStates);
            }
            
            if (data.controller.joystickState) {
              setLocalControlState(data.controller.joystickState);
            }
          }
        } catch (err) {
          console.error('Error parsing telemetry data:', err);
        }
      });

      // For clients with a gamepad, create publishers for button and joystick states
      if (isGamepadConnected) {
        buttonStatePublisher = new ROSLIB.Topic({
          ros: ros,
          name: '/controller/button_states',
          messageType: 'std_msgs/String'
        });

        joystickStatePublisher = new ROSLIB.Topic({
          ros: ros,
          name: '/controller/joystick_state',
          messageType: 'std_msgs/String'
        });
      }
    }

    // Start gamepad polling
    const pollGamepad = () => {
      if (gamepadRef.current) {
        const gamepads = navigator.getGamepads();
        const gamepad = gamepads[gamepadRef.current.index];

        if (gamepad) {
          // Update button states
          const newButtonStates = {
            // Face buttons (mapping may need adjustment based on your Steam Deck)
            A: gamepad.buttons[0].pressed,
            B: gamepad.buttons[1].pressed,
            X: gamepad.buttons[2].pressed,
            Y: gamepad.buttons[3].pressed,
            // D-pad
            DpadUp: gamepad.buttons[12].pressed,
            DpadDown: gamepad.buttons[13].pressed,
            DpadLeft: gamepad.buttons[14].pressed,
            DpadRight: gamepad.buttons[15].pressed,
            // Shoulder buttons
            L1: gamepad.buttons[4].pressed,
            L2: gamepad.buttons[6].pressed,
            L3: gamepad.buttons[10].pressed,
            L4: gamepad.buttons[8].pressed,
            R1: gamepad.buttons[5].pressed,
            R2: gamepad.buttons[7].pressed,
            R3: gamepad.buttons[11].pressed,
            R4: gamepad.buttons[9].pressed,
          };

          setButtonStates(newButtonStates);

          // Update control state for ROS - using original mapping
          const newState = {
            linear: {
              x: gamepad.axes[0] * 1.0, // Left stick X
              y: gamepad.axes[1] * 1.0, // Left stick Y
              z: (gamepad.buttons[6].value - gamepad.buttons[7].value) * 0.5 // L2 - R2
            },
            angular: {
              x: gamepad.axes[2] * 1.0, // Right stick X
              y: gamepad.axes[3] * 1.0, // Right stick Y
              z: (gamepad.buttons[4].value - gamepad.buttons[5].value) * 1.0 // L1 - R1
            }
          };
          
          // Publish button states via ROS topic
          if (buttonStatePublisher) {
            const buttonStateMsg = new ROSLIB.Message({
              data: JSON.stringify(newButtonStates)
            });
            buttonStatePublisher.publish(buttonStateMsg);
          }

          // Publish joystick state via ROS topic
          if (joystickStatePublisher) {
            const joystickStateMsg = new ROSLIB.Message({
              data: JSON.stringify(newState)
            });
            joystickStatePublisher.publish(joystickStateMsg);
          }

          // Also use socket if available for backward compatibility
          if (socket) {
            socket.emit('controller_button_states', newButtonStates);
            socket.emit('controller_joystick_state', newState);
          }

          if (onControlChange) {
            onControlChange(newState);
          }
        }
      }

      animationFrameRef.current = requestAnimationFrame(pollGamepad);
    };

    pollGamepad();

    return () => {
      if (animationFrameRef.current) {
        cancelAnimationFrame(animationFrameRef.current);
      }
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected);
      if (telemetrySubscriber) {
        telemetrySubscriber.unsubscribe();
      }
    };
  }, [onControlChange, ros, socket, isGamepadConnected]);

  // Socket.io event listener for backward compatibility
  useEffect(() => {
    if (socket && !isGamepadConnected) {
      // Listen for button state updates from other clients
      socket.on('controller_button_states', (remoteButtonStates) => {
        // Only update if we don't have a gamepad connected
        if (!isGamepadConnected) {
          setButtonStates(remoteButtonStates);
        }
      });

      // Listen for joystick state updates from other clients
      socket.on('controller_joystick_state', (remoteJoystickState) => {
        if (!isGamepadConnected) {
          setLocalControlState(remoteJoystickState);
        }
      });

      return () => {
        socket.off('controller_button_states');
        socket.off('controller_joystick_state');
      };
    }
  }, [socket, isGamepadConnected]);

  const toggleMappings = (e) => {
    e.stopPropagation(); // Prevent click from toggling hide
    setShowMappings(!showMappings);
  };

  const toggleHide = (e) => {
    // Only toggle hide if the click is directly on the overlay background or the hide button
    if (e.target === e.currentTarget || e.target.classList.contains('hide-toggle')) {
      e.stopPropagation(); // Prevent bubbling
      setIsHidden(!isHidden);
      // Reset showMappings when hiding the overlay
      if (!isHidden) {
        setShowMappings(false);
      }
    }
  };

  // Use the appropriate control state based on whether this client has a gamepad or not
  const displayControlState = isGamepadConnected ? controlState : localControlState;
  
  // Ensure controlState is defined to prevent errors
  const safeControlState = displayControlState || {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  };

  return (
    <div 
      className={`control-overlay ${isHidden ? 'hidden' : ''}`}
      onClick={toggleHide} // Use onClick for both hidden and visible states, logic handled in toggleHide
    >
      {!isHidden && (
        <>
          {/* Control buttons */}
          <div className="control-buttons">
            <button 
              className="control-button mappings-toggle" 
              onClick={toggleMappings}
            >
              {showMappings ? 'Hide Mappings' : 'Show Mappings'}
            </button>
            <button 
              className="control-button hide-toggle" 
              onClick={toggleHide} // Bind toggleHide to the hide button too
            >
              Hide Controls
            </button>
          </div>

          <div className="control-content" onClick={(e) => e.stopPropagation()}> {/* Prevent clicks inside content from hiding overlay */}
            {/* Controls Section */}
            <div className="control-section">
              {/* Left Section - Left Stick and D-pad */}
              <div className="control-group">
                <div className="control-label">Left Stick</div>
                <div className="joystick-container">
                  <div 
                    className="joystick-dot"
                    style={{
                      transform: `translate(${safeControlState.linear.x * 30}px, ${safeControlState.linear.y * 30}px)`
                    }}
                  />
                </div>
                <div className="value-display">
                  X: {safeControlState.linear.x.toFixed(2)}
                  Y: {safeControlState.linear.y.toFixed(2)}
                </div>
              </div>

              <div className="control-group">
                <div className="control-label">D-pad</div>
                <div className="dpad">
                  <div className={`dpad-button ${buttonStates.DpadUp ? 'pressed' : ''}`}>↑</div>
                  <div className={`dpad-button ${buttonStates.DpadRight ? 'pressed' : ''}`}>→</div>
                  <div className={`dpad-button ${buttonStates.DpadDown ? 'pressed' : ''}`}>↓</div>
                  <div className={`dpad-button ${buttonStates.DpadLeft ? 'pressed' : ''}`}>←</div>
                  <div className="dpad-button dpad-center"></div>
                </div>
              </div>

              {/* Middle Section - Face Buttons and Right Stick */}
              <div className="control-group">
                <div className="control-label">Face Buttons</div>
                <div className="button-grid">
                  <div className={`button ${buttonStates.Y ? 'pressed' : ''}`}>Y</div>
                  <div className={`button ${buttonStates.B ? 'pressed' : ''}`}>B</div>
                  <div className={`button ${buttonStates.X ? 'pressed' : ''}`}>X</div>
                  <div className={`button ${buttonStates.A ? 'pressed' : ''}`}>A</div>
                </div>
              </div>

              <div className="control-group">
                <div className="control-label">Right Stick</div>
                <div className="joystick-container">
                  <div 
                    className="joystick-dot"
                    style={{
                      transform: `translate(${safeControlState.angular.x * 30}px, ${safeControlState.angular.y * 30}px)`
                    }}
                  />
                </div>
                <div className="value-display">
                  X: {safeControlState.angular.x.toFixed(2)}
                  Y: {safeControlState.angular.y.toFixed(2)}
                </div>
              </div>

              {/* Right Section - Triggers and Shoulder Buttons */}
              <div className="control-group">
                <div className="control-label">Left Triggers</div>
                <div className="trigger-container">
                  <div className="trigger">
                    <div 
                      className="trigger-fill"
                      style={{ width: `${(buttonStates.L2 ? 1 : 0) * 100}%` }}
                    />
                  </div>
                  <div className="value-display">L2: {((buttonStates.L2 ? 1 : 0) * 100).toFixed(0)}%</div>
                  <div className={`button ${buttonStates.L1 ? 'pressed' : ''}`}>L1</div>
                  <div className={`button ${buttonStates.L3 ? 'pressed' : ''}`}>L3</div>
                  <div className={`button ${buttonStates.L4 ? 'pressed' : ''}`}>L4</div>
                </div>
              </div>

              <div className="control-group">
                <div className="control-label">Right Triggers</div>
                <div className="trigger-container">
                  <div className="trigger">
                    <div 
                      className="trigger-fill"
                      style={{ width: `${(buttonStates.R2 ? 1 : 0) * 100}%` }}
                    />
                  </div>
                  <div className="value-display">R2: {((buttonStates.R2 ? 1 : 0) * 100).toFixed(0)}%</div>
                  <div className={`button ${buttonStates.R1 ? 'pressed' : ''}`}>R1</div>
                  <div className={`button ${buttonStates.R3 ? 'pressed' : ''}`}>R3</div>
                  <div className={`button ${buttonStates.R4 ? 'pressed' : ''}`}>R4</div>
                </div>
              </div>
            </div>

            {/* ROS Mappings Section - Overlay */}
            <div className={`mapping-section ${showMappings ? 'visible' : ''}`} onClick={(e) => e.stopPropagation()}> {/* Prevent clicks inside mapping from hiding overlay */}
              {showMappings && (
                <>
                  <div className="mapping-title">ROS Mappings</div>
                  
                  <div className="mapping-info">
                    <div className="mapping-title">Left Stick</div>
                    <div>{rosMappings.leftStick.x}</div>
                    <div>{rosMappings.leftStick.y}</div>
                  </div>
                  
                  <div className="mapping-info">
                    <div className="mapping-title">Right Stick</div>
                    <div>{rosMappings.rightStick.x}</div>
                    <div>{rosMappings.rightStick.y}</div>
                  </div>
                  
                  <div className="mapping-info">
                    <div className="mapping-title">Triggers</div>
                    <div>{rosMappings.triggers.left}</div>
                    <div>{rosMappings.triggers.right}</div>
                  </div>
                  
                  <div className="mapping-info">
                    <div className="mapping-title">D-pad</div>
                    <div>{rosMappings.dpad.up}</div>
                    <div>{rosMappings.dpad.down}</div>
                    <div>{rosMappings.dpad.left}</div>
                    <div>{rosMappings.dpad.right}</div>
                  </div>
                  
                  <div className="mapping-info">
                    <div className="mapping-title">Buttons</div>
                    <div>{rosMappings.buttons.A}</div>
                    <div>{rosMappings.buttons.B}</div>
                    <div>{rosMappings.buttons.X}</div>
                    <div>{rosMappings.buttons.Y}</div>
                  </div>
                </>
              )}
            </div>
          </div>
        </>
      )}
    </div>
  );
};

export default ControlOverlay; 