import React, { useEffect, useRef, useState, useCallback } from 'react';
import '../styles/ControlOverlay.css';
import ROSLIB from 'roslib';
import soundEffects from '../audio/SoundEffects';
import { useSettings } from '../contexts/SettingsContext';

const ControlOverlay = ({ onControlChange, controlState, ros, socket }) => {
  const gamepadRef = useRef(null);
  const animationFrameRef = useRef();
  const { getSetting, updateSettings } = useSettings();
  
  // Get minimized state from settings
  const initialMinimized = getSetting('ui', 'controllerMinimized', false);
  console.log('ControlOverlay initializing with minimized state:', initialMinimized);
  
  // Use isHidden for UI state (instead of tracking a separate isVisible state)
  const [isHidden, setIsHidden] = useState(initialMinimized);
  
  // Log when isHidden changes
  useEffect(() => {
    console.log('ControlOverlay isHidden state changed to:', isHidden);
  }, [isHidden]);
  
  // Sync with settings when they change
  useEffect(() => {
    const minimizedInSettings = getSetting('ui', 'controllerMinimized', false);
    if (isHidden !== minimizedInSettings) {
      console.log('Syncing minimized state with settings:', minimizedInSettings);
      setIsHidden(minimizedInSettings);
    }
  }, [getSetting, isHidden]);
  
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
  const [isGamepadConnected, setIsGamepadConnected] = useState(false);
  // Store local controlState for non-gamepad clients
  const [localControlState, setLocalControlState] = useState({
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  });

  // Initialize audio context on mount and first interaction
  useEffect(() => {
    let isSubscribed = true;

    const initAudio = async () => {
      try {
        if (isSubscribed) {
          await soundEffects.initializeAudio();
        }
      } catch (error) {
        console.warn('Failed to initialize audio:', error);
      }
    };

    const handleFirstInteraction = () => {
      initAudio();
      if (isSubscribed) {
        document.removeEventListener('click', handleFirstInteraction, true);
        document.removeEventListener('keydown', handleFirstInteraction, true);
        document.removeEventListener('touchstart', handleFirstInteraction, true);
        document.removeEventListener('mousedown', handleFirstInteraction, true);
      }
    };

    document.addEventListener('click', handleFirstInteraction, true);
    document.addEventListener('keydown', handleFirstInteraction, true);
    document.addEventListener('touchstart', handleFirstInteraction, true);
    document.addEventListener('mousedown', handleFirstInteraction, true);

    initAudio();

    return () => {
      isSubscribed = false;
      document.removeEventListener('click', handleFirstInteraction, true);
      document.removeEventListener('keydown', handleFirstInteraction, true);
      document.removeEventListener('touchstart', handleFirstInteraction, true);
      document.removeEventListener('mousedown', handleFirstInteraction, true);
    };
  }, []);

  // Gamepad connection handlers
  useEffect(() => {
    const handleGamepadConnected = (e) => {
      console.log('Gamepad connected:', e.gamepad);
      gamepadRef.current = e.gamepad;
      setIsGamepadConnected(true);
      soundEffects.playMenuButtonClick().catch(console.error); // Feedback for connection
    };

    const handleGamepadDisconnected = (e) => {
      console.log('Gamepad disconnected:', e.gamepad);
      if (gamepadRef.current && gamepadRef.current.index === e.gamepad.index) {
        gamepadRef.current = null;
        setIsGamepadConnected(false);
        soundEffects.playMenuButtonClick().catch(console.error); // Feedback for disconnection
      }
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    return () => {
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected);
    };
  }, []);

  // Gamepad polling and ROS/Socket publishing
  useEffect(() => {
    let buttonStatePublisher = null;
    let joystickStatePublisher = null;

    if (ros && isGamepadConnected) {
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

    const pollGamepad = () => {
      if (gamepadRef.current) {
        const gamepads = navigator.getGamepads();
        const gamepad = gamepads[gamepadRef.current.index];

        if (gamepad) {
          // Track previous button states for sound effects
          const prevButtonStates = { ...buttonStates };

          // Update button states
          const newButtonStates = {
            A: gamepad.buttons[0].pressed,
            B: gamepad.buttons[1].pressed,
            X: gamepad.buttons[2].pressed,
            Y: gamepad.buttons[3].pressed,
            DpadUp: gamepad.buttons[12].pressed,
            DpadDown: gamepad.buttons[13].pressed,
            DpadLeft: gamepad.buttons[14].pressed,
            DpadRight: gamepad.buttons[15].pressed,
            L1: gamepad.buttons[4].pressed,
            L2: gamepad.buttons[6].pressed,
            L3: gamepad.buttons[10].pressed,
            L4: gamepad.buttons[8].pressed,
            R1: gamepad.buttons[5].pressed,
            R2: gamepad.buttons[7].pressed,
            R3: gamepad.buttons[11].pressed,
            R4: gamepad.buttons[9].pressed,
          };

          // Play sounds for newly pressed buttons
          Object.entries(newButtonStates).forEach(([button, isPressed]) => {
            if (isPressed && !prevButtonStates[button]) {
              if (button.startsWith('Dpad')) {
                soundEffects.playDpadClick().catch(console.error);
              } else if (['A', 'B', 'X', 'Y'].includes(button)) {
                soundEffects.playFaceButtonClick().catch(console.error);
              } else {
                soundEffects.playButtonClick().catch(console.error);
              }
            }
          });

          setButtonStates(newButtonStates);

          // Update control state
          const newState = {
            linear: {
              x: gamepad.axes[0] * 1.0,
              y: gamepad.axes[1] * 1.0,
              z: (gamepad.buttons[6].value - gamepad.buttons[7].value) * 0.5
            },
            angular: {
              x: gamepad.axes[2] * 1.0,
              y: gamepad.axes[3] * 1.0,
              z: (gamepad.buttons[4].value - gamepad.buttons[5].value) * 1.0
            }
          };

          // Publish states
          if (buttonStatePublisher) {
            buttonStatePublisher.publish(new ROSLIB.Message({
              data: JSON.stringify(newButtonStates)
            }));
          }

          if (joystickStatePublisher) {
            joystickStatePublisher.publish(new ROSLIB.Message({
              data: JSON.stringify(newState)
            }));
          }

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
    };
  }, [onControlChange, ros, socket, isGamepadConnected, buttonStates]);

  // Socket event handlers for remote gamepad updates
  useEffect(() => {
    if (socket && !isGamepadConnected) {
      socket.on('controller_button_states', (remoteButtonStates) => {
        if (!isGamepadConnected) {
          setButtonStates(remoteButtonStates);
        }
      });

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

  // ROS mapping information
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

  // Click handlers with sound effects
  const handleDpadPress = useCallback(async (direction) => {
    await soundEffects.playDpadClick().catch(console.error);
    setButtonStates(prev => ({
      ...prev,
      [`Dpad${direction}`]: true
    }));
  }, []);

  const handleFaceButtonPress = useCallback(async (button) => {
    await soundEffects.playFaceButtonClick().catch(console.error);
    setButtonStates(prev => ({
      ...prev,
      [button]: true
    }));
  }, []);

  const handleTriggerPress = useCallback(async (trigger) => {
    await soundEffects.playButtonClick().catch(console.error);
    setButtonStates(prev => ({
      ...prev,
      [trigger]: true
    }));
  }, []);

  const toggleMappings = useCallback(async (e) => {
    e.stopPropagation();
    await soundEffects.playMenuButtonClick().catch(console.error);
    setShowMappings(prev => !prev);
  }, []);

  const toggleHide = useCallback(async (e) => {
    if (e.target === e.currentTarget || e.target.classList.contains('hide-toggle')) {
      e.stopPropagation();
      await soundEffects.playMenuButtonClick().catch(console.error);
      
      const newHiddenState = !isHidden;
      setIsHidden(newHiddenState);
      
      // Update the settings context with the new minimized state
      // This won't cause the component to unmount, just change its appearance
      updateSettings({
        ui: {
          controllerMinimized: newHiddenState
        }
      });
      
      if (newHiddenState) {
        setShowMappings(false);
      }
    }
  }, [isHidden, updateSettings]);

  // Use the appropriate control state based on whether this client has a gamepad or not
  const displayControlState = isGamepadConnected ? controlState : localControlState;
  
  // Ensure controlState is defined to prevent errors
  const safeControlState = displayControlState || {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  };

  return (
    <div className="controls-container">
    <div 
      className={`control-overlay ${isHidden ? 'hidden' : ''}`}
      onClick={toggleHide}
    >
      <div className="control-border"></div>
      {!isHidden && (
        <>
          <div className="control-buttons">
            <h3>CONTROLS</h3>
            <div>
              <button 
                className="control-button mappings-toggle" 
                onClick={toggleMappings}
              >
                {showMappings ? 'Hide Mappings' : 'Show Mappings'}
              </button>
              <button 
                className="control-button hide-toggle" 
                  onClick={toggleHide}
              >
                Hide
              </button>
            </div>
          </div>

            <div className="control-content">
            <div className="control-section">
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
                    <div 
                      className={`dpad-button ${buttonStates.DpadUp ? 'pressed' : ''}`}
                      onClick={() => handleDpadPress('Up')}
                    >↑</div>
                    <div 
                      className={`dpad-button ${buttonStates.DpadRight ? 'pressed' : ''}`}
                      onClick={() => handleDpadPress('Right')}
                    >→</div>
                    <div 
                      className={`dpad-button ${buttonStates.DpadDown ? 'pressed' : ''}`}
                      onClick={() => handleDpadPress('Down')}
                    >↓</div>
                    <div 
                      className={`dpad-button ${buttonStates.DpadLeft ? 'pressed' : ''}`}
                      onClick={() => handleDpadPress('Left')}
                    >←</div>
                  <div className="dpad-button dpad-center"></div>
                </div>
              </div>

              <div className="control-group">
                <div className="control-label">Face Buttons</div>
                <div className="button-grid">
                    <div 
                      className={`button ${buttonStates.Y ? 'pressed' : ''}`}
                      onClick={() => handleFaceButtonPress('Y')}
                    ><span>Y</span></div>
                    <div 
                      className={`button ${buttonStates.B ? 'pressed' : ''}`}
                      onClick={() => handleFaceButtonPress('B')}
                    ><span>B</span></div>
                    <div 
                      className={`button ${buttonStates.X ? 'pressed' : ''}`}
                      onClick={() => handleFaceButtonPress('X')}
                    ><span>X</span></div>
                    <div 
                      className={`button ${buttonStates.A ? 'pressed' : ''}`}
                      onClick={() => handleFaceButtonPress('A')}
                    ><span>A</span></div>
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
                    <div 
                      className={`button ${buttonStates.L1 ? 'pressed' : ''}`}
                      onClick={() => handleTriggerPress('L1')}
                    >L1</div>
                    <div 
                      className={`button ${buttonStates.L3 ? 'pressed' : ''}`}
                      onClick={() => handleTriggerPress('L3')}
                    >L3</div>
                    <div 
                      className={`button ${buttonStates.L4 ? 'pressed' : ''}`}
                      onClick={() => handleTriggerPress('L4')}
                    >L4</div>
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
                    <div 
                      className={`button ${buttonStates.R1 ? 'pressed' : ''}`}
                      onClick={() => handleTriggerPress('R1')}
                    >R1</div>
                    <div 
                      className={`button ${buttonStates.R3 ? 'pressed' : ''}`}
                      onClick={() => handleTriggerPress('R3')}
                    >R3</div>
                    <div 
                      className={`button ${buttonStates.R4 ? 'pressed' : ''}`}
                      onClick={() => handleTriggerPress('R4')}
                    >R4</div>
                  </div>
                </div>
              </div>
            </div>
          </>
        )}
            </div>

      <div className={`mapping-section ${showMappings ? 'visible' : ''}`} onClick={(e) => e.stopPropagation()}>
        <div className="mapping-border"></div>
              {showMappings && (
          <div className="mapping-content">
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
          </div>
      )}
      </div>
    </div>
  );
};

export default ControlOverlay; 