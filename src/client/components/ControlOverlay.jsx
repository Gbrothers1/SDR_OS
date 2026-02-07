import React, { useEffect, useRef, useState, useCallback, useMemo } from 'react';
import { createPortal } from 'react-dom';
import '../styles/ControlOverlay.css';
import ROSLIB from 'roslib';
import soundEffects from '../audio/SoundEffects';
import { useSettings } from '../contexts/SettingsContext';

const ControlOverlay = ({ onControlChange, controlState, ros, socket, sendGamepadAxes, sendCommand, sendVelocityCommand, onExpandChange }) => {
  const gamepadRef = useRef(null);
  const animationFrameRef = useRef();
  const { getSetting, updateSettings } = useSettings();
  const buttonStatesTopic = getSetting('topics', 'button_states', '/controller/button_states');
  const joystickStateTopic = getSetting('topics', 'joystick_state', '/controller/joystick_state');
  const cmdVelTopic = getSetting('topics', 'cmd_vel', '/cmd_vel');

  // Get minimized state from settings
  const initialMinimized = getSetting('ui', 'controllerMinimized', false);

  // Use isHidden for UI state (instead of tracking a separate isVisible state)
  const [isHidden, setIsHidden] = useState(initialMinimized);

  // Sync with settings when they change
  useEffect(() => {
    const minimizedInSettings = getSetting('ui', 'controllerMinimized', false);
    if (isHidden !== minimizedInSettings) {
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
  const [rawButtons, setRawButtons] = useState([]);
  const [rawAxes, setRawAxes] = useState([]);
  const [lastEmittedButtons, setLastEmittedButtons] = useState(null);
  const lastEmittedRef = useRef(0);
  const buttonStatesRef = useRef(buttonStates);
  const lastJoystickEmitRef = useRef(0);
  const lastButtonEmitRef = useRef(0);
  const lastUiUpdateRef = useRef(0);
  const joystickSendIntervalMs = 1000 / 30;
  const buttonSendIntervalMs = 1000 / 10;
  const uiUpdateIntervalMs = 1000 / 60;
  const debugGamepad = useMemo(() => {
    try {
      return localStorage.getItem('debugGamepad') === 'true';
    } catch (e) {
      return false;
    }
  }, []);
  const lastDebugUpdateRef = useRef(0);
  const [isGamepadConnected, setIsGamepadConnected] = useState(false);
  const isGamepadConnectedRef = useRef(false);
  const [showGamepadPrompt, setShowGamepadPrompt] = useState(false);
  const gamepadMissCount = useRef(0);
  const GAMEPAD_MISS_THRESHOLD = 60; // ~1 second at 60fps before declaring disconnect
  const portalTarget = document.getElementById('overflow-panel-left');

  useEffect(() => {
    buttonStatesRef.current = buttonStates;
  }, [buttonStates]);

  useEffect(() => {
    isGamepadConnectedRef.current = isGamepadConnected;
  }, [isGamepadConnected]);

  // Detect Safari browser
  const isSafari = /^((?!chrome|android).)*safari/i.test(navigator.userAgent);
  
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
      console.log('Gamepad buttons:', e.gamepad.buttons.length, 'axes:', e.gamepad.axes.length);
      gamepadRef.current = e.gamepad;
      gamepadMissCount.current = 0;
      setIsGamepadConnected(true);
      setShowGamepadPrompt(false); // Hide Safari prompt when gamepad connects
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

    const tryAttachFirstGamepad = () => {
      if (gamepadRef.current) {
        return true;
      }
      // Safari requires navigator.getGamepads() to be called - it may return null
      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      if (!gamepads) {
        console.log('Gamepad API returned null (Safari before button press?)');
        return false;
      }
      const firstGamepad = Array.from(gamepads).find(Boolean);
      if (firstGamepad) {
        console.log('Gamepad detected via polling:', firstGamepad);
        console.log('Gamepad buttons:', firstGamepad.buttons.length, 'axes:', firstGamepad.axes.length);
        gamepadRef.current = firstGamepad;
        setIsGamepadConnected(true);
        setShowGamepadPrompt(false); // Hide Safari prompt
        soundEffects.playMenuButtonClick().catch(console.error);
        return true;
      }
      return false;
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);
    window.addEventListener('focus', tryAttachFirstGamepad);
    document.addEventListener('visibilitychange', tryAttachFirstGamepad);

    // Safari may not fire gamepadconnected until a user gesture; poll once on mount
    tryAttachFirstGamepad();
    
    // Safari-specific: Continuously poll for gamepad detection
    // Safari requires pressing a button on the controller before it exposes gamepads,
    // AND the polling must be happening when the button is pressed
    let safariPollInterval = null;
    if (isSafari) {
      console.log('Safari detected - enabling aggressive gamepad polling');
      safariPollInterval = setInterval(() => {
        if (!gamepadRef.current) {
          const found = tryAttachFirstGamepad();
          if (found) {
            clearInterval(safariPollInterval);
          }
        } else {
          clearInterval(safariPollInterval);
        }
      }, 500); // Poll every 500ms for Safari
    }
    
    // Show prompt after a delay if no gamepad detected
    const safariPromptTimeout = setTimeout(() => {
      if (!gamepadRef.current && isSafari) {
        console.log('Safari detected, no gamepad found - showing prompt');
        setShowGamepadPrompt(true);
      }
    }, 2000);

    return () => {
      clearTimeout(safariPromptTimeout);
      if (safariPollInterval) clearInterval(safariPollInterval);
      window.removeEventListener('gamepadconnected', handleGamepadConnected);
      window.removeEventListener('gamepaddisconnected', handleGamepadDisconnected);
      window.removeEventListener('focus', tryAttachFirstGamepad);
      document.removeEventListener('visibilitychange', tryAttachFirstGamepad);
    };
  }, [isSafari]);

  // ROS publisher refs — created lazily when both ros and gamepad are available
  const buttonStatePublisherRef = useRef(null);
  const joystickStatePublisherRef = useRef(null);
  const cmdVelPublisherRef = useRef(null);
  const rosRef = useRef(ros);
  const socketRef = useRef(socket);
  const onControlChangeRef = useRef(onControlChange);

  useEffect(() => {
    rosRef.current = ros;
    buttonStatePublisherRef.current = null;
    joystickStatePublisherRef.current = null;
    cmdVelPublisherRef.current = null;
  }, [ros]);
  useEffect(() => { socketRef.current = socket; }, [socket]);
  useEffect(() => { onControlChangeRef.current = onControlChange; }, [onControlChange]);

  // Gamepad polling and ROS/Socket publishing
  useEffect(() => {
    const getOrCreatePublishers = () => {
      if (rosRef.current && isGamepadConnectedRef.current) {
        if (!buttonStatePublisherRef.current) {
          buttonStatePublisherRef.current = new ROSLIB.Topic({
            ros: rosRef.current,
            name: buttonStatesTopic,
            messageType: 'std_msgs/String'
          });
        }
        if (!joystickStatePublisherRef.current) {
          joystickStatePublisherRef.current = new ROSLIB.Topic({
            ros: rosRef.current,
            name: joystickStateTopic,
            messageType: 'std_msgs/String'
          });
        }
        if (!cmdVelPublisherRef.current) {
          cmdVelPublisherRef.current = new ROSLIB.Topic({
            ros: rosRef.current,
            name: cmdVelTopic,
            messageType: 'geometry_msgs/Twist'
          });
        }
      }
      return {
        buttonStatePublisher: buttonStatePublisherRef.current,
        joystickStatePublisher: joystickStatePublisherRef.current,
        cmdVelPublisher: cmdVelPublisherRef.current,
      };
    };

    const pollGamepad = () => {
      // Safari-safe: navigator.getGamepads() may return null before user presses a button
      let gamepads;
      try {
        gamepads = navigator.getGamepads ? navigator.getGamepads() : null;
      } catch (e) {
        console.warn('Error accessing gamepad API:', e);
        gamepads = null;
      }
      
      if (!gamepads) {
        // Safari returns null until a button is pressed on the controller
        animationFrameRef.current = requestAnimationFrame(pollGamepad);
        return;
      }
      
      if (!gamepadRef.current) {
        const firstGamepad = Array.from(gamepads).find(Boolean);
        if (firstGamepad) {
          console.log('Gamepad detected via polling:', firstGamepad);
          console.log('Gamepad buttons:', firstGamepad.buttons.length, 'axes:', firstGamepad.axes.length);
          gamepadRef.current = firstGamepad;
          setIsGamepadConnected(true);
          setShowGamepadPrompt(false); // Hide Safari prompt
          soundEffects.playMenuButtonClick().catch(console.error);
        }
      }

      if (gamepadRef.current) {
        let gamepad = gamepads[gamepadRef.current.index];

        // If lookup by stored index fails, scan all gamepads for a match
        if (!gamepad) {
          const fallback = Array.from(gamepads).find(Boolean);
          if (fallback) {
            gamepad = fallback;
            gamepadRef.current = fallback;
            gamepadMissCount.current = 0;
          }
        }

        if (gamepad) {
          gamepadMissCount.current = 0;
          if (debugGamepad) {
            const now = Date.now();
            if (now - lastDebugUpdateRef.current > 200) {
              setRawButtons(
                gamepad.buttons.map((button, index) => ({
                  index,
                  pressed: button?.pressed || false,
                  value: button?.value ?? 0
                }))
              );
              setRawAxes(
                gamepad.axes.map((value, index) => ({
                  index,
                  value: value ?? 0
                }))
              );
              lastDebugUpdateRef.current = now;
            }
          }

          // Track previous button states for sound effects
          const prevButtonStates = buttonStatesRef.current;

          // Helper to safely access button state (Safari may have fewer buttons)
          const getButton = (index) => gamepad.buttons[index] || { pressed: false, value: 0 };
          const getAxis = (index) => gamepad.axes[index] ?? 0;

          // Update button states with safe access
          const newButtonStates = {
            A: getButton(0).pressed,
            B: getButton(1).pressed,
            X: getButton(2).pressed,
            Y: getButton(3).pressed,
            DpadUp: getButton(12).pressed,
            DpadDown: getButton(13).pressed,
            DpadLeft: getButton(14).pressed,
            DpadRight: getButton(15).pressed,
            L1: getButton(4).pressed,
            L2: getButton(6).pressed,
            L3: getButton(10).pressed,
            L4: getButton(8).pressed,
            R1: getButton(5).pressed,
            R2: getButton(7).pressed,
            R3: getButton(11).pressed,
            R4: getButton(9).pressed,
          };

          const buttonsChanged = Object.keys(newButtonStates).some(
            (key) => newButtonStates[key] !== prevButtonStates[key]
          );

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

          if (buttonsChanged) {
            setButtonStates(newButtonStates);
          }
          buttonStatesRef.current = newButtonStates;

          // Update control state - ROS Twist format for local display
          const newState = {
            linear: {
              x: getAxis(0) * 1.0,
              y: getAxis(1) * 1.0,
              z: (getButton(6).value - getButton(7).value) * 0.5
            },
            angular: {
              x: getAxis(2) * 1.0,
              y: getAxis(3) * 1.0,
              z: (getButton(4).value - getButton(5).value) * 1.0
            }
          };

          // Genesis bridge format - raw stick values
          const joystickState = {
            leftStickX: getAxis(0),
            leftStickY: getAxis(1),
            rightStickX: getAxis(2),
            rightStickY: getAxis(3),
          };

          const nowEmit = Date.now();
          const shouldSendButtons = buttonsChanged ||
            (nowEmit - lastButtonEmitRef.current >= buttonSendIntervalMs);
          const shouldSendJoystick = nowEmit - lastJoystickEmitRef.current >= joystickSendIntervalMs;
          const shouldUpdateUi = nowEmit - lastUiUpdateRef.current >= uiUpdateIntervalMs;

          const { buttonStatePublisher, joystickStatePublisher, cmdVelPublisher } = getOrCreatePublishers();

          if (shouldSendButtons) {
            if (buttonStatePublisher) {
              buttonStatePublisher.publish(new ROSLIB.Message({
                data: JSON.stringify(newButtonStates)
              }));
            }
            // DataChannel path (WebRTC) — send button events as commands
            if (sendCommand) {
              Object.entries(newButtonStates).forEach(([id, pressed]) => {
                sendCommand({ type: 'button', id: parseInt(id, 10), pressed });
              });
            }
            if (socketRef.current) {
              socketRef.current.emit('controller_button_states', newButtonStates);
            }
            lastButtonEmitRef.current = nowEmit;
          }

          if (shouldSendJoystick) {
            // ROS /cmd_vel — standard geometry_msgs/Twist for any ROS2 robot
            if (cmdVelPublisher) {
              cmdVelPublisher.publish(new ROSLIB.Message({
                linear: { x: newState.linear.x, y: newState.linear.y, z: newState.linear.z },
                angular: { x: newState.angular.x, y: newState.angular.y, z: newState.angular.z }
              }));
            }
            // Genesis NATS set_cmd_vel — sim-specific with safety stack
            if (sendVelocityCommand) {
              sendVelocityCommand(newState.linear.x, newState.linear.y, newState.angular.z);
            }
            if (joystickStatePublisher) {
              joystickStatePublisher.publish(new ROSLIB.Message({
                data: JSON.stringify(newState)
              }));
            }
            // DataChannel path (WebRTC) — send axes as binary
            if (sendGamepadAxes) {
              const axes = [getAxis(0), getAxis(1), getAxis(2), getAxis(3)];
              const bitmask = Object.entries(newButtonStates).reduce(
                (mask, [id, pressed]) => pressed ? mask | (1 << parseInt(id, 10)) : mask, 0
              );
              sendGamepadAxes(axes, bitmask);
            }
            if (socketRef.current) {
              const socketEmitter = socketRef.current.volatile || socketRef.current;
              socketEmitter.emit('controller_joystick_state', joystickState);
            }
            lastJoystickEmitRef.current = nowEmit;

            // Debug logging (only log occasionally to avoid spam)
            if (!window._lastJoystickLog || Date.now() - window._lastJoystickLog > 1000) {
              console.log('Sending joystick state:', joystickState);
              window._lastJoystickLog = Date.now();
            }
          }

          if (debugGamepad && shouldSendButtons) {
            if (nowEmit - lastEmittedRef.current > 200) {
              setLastEmittedButtons({
                time: nowEmit,
                buttons: newButtonStates
              });
              lastEmittedRef.current = nowEmit;
            }
          }

          if (shouldUpdateUi) {
            setLocalControlState(newState);
            lastUiUpdateRef.current = nowEmit;
            if (onControlChangeRef.current) {
              onControlChangeRef.current(newState);
            }
          }
        } else {
          gamepadMissCount.current++;
          if (gamepadMissCount.current >= GAMEPAD_MISS_THRESHOLD && isGamepadConnectedRef.current) {
            console.log(`Gamepad missing for ${gamepadMissCount.current} frames; marking disconnected`);
            gamepadRef.current = null;
            gamepadMissCount.current = 0;
            setIsGamepadConnected(false);
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
      buttonStatePublisherRef.current = null;
      joystickStatePublisherRef.current = null;
      cmdVelPublisherRef.current = null;
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [buttonStatesTopic, joystickStateTopic, cmdVelTopic]);

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
          // Convert Genesis bridge format back to display format if needed
          if (remoteJoystickState.leftStickX !== undefined) {
            // Convert from Genesis format to display format
            setLocalControlState({
              linear: {
                x: remoteJoystickState.leftStickX,
                y: remoteJoystickState.leftStickY,
                z: 0
              },
              angular: {
                x: remoteJoystickState.rightStickX,
                y: remoteJoystickState.rightStickY,
                z: 0
              }
            });
          } else {
            // Already in display format
            setLocalControlState(remoteJoystickState);
          }
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

  useEffect(() => {
    if (onExpandChange) onExpandChange(showMappings);
  }, [showMappings, onExpandChange]);

  // Use the appropriate control state based on whether this client has a gamepad or not
  const displayControlState = isGamepadConnected ? controlState : localControlState;
  
  // Ensure controlState is defined to prevent errors
  const safeControlState = displayControlState || {
    linear: { x: 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: 0 }
  };

  return (
    <div className="controls-container">
    {/* Safari gamepad prompt */}
    {showGamepadPrompt && !isGamepadConnected && (
      <div className="gamepad-prompt" onClick={() => setShowGamepadPrompt(false)}>
        <div className="gamepad-prompt-content">
          <p><strong>Gamepad not detected</strong></p>
          <p>Safari requires you to press any button on your controller to enable gamepad access.</p>
          <p style={{ fontSize: '0.9em', opacity: 0.7 }}>Click to dismiss</p>
        </div>
      </div>
    )}
    <div
      className={`control-overlay ${isHidden ? 'hidden' : ''}`}
    >
      {isHidden ? (
        <div className="control-buttons control-buttons--collapsed" onClick={toggleHide}>
          <h3>CONTROLS</h3>
          <button className="control-button hide-toggle" onClick={toggleHide}>Show</button>
        </div>
      ) : (
        <>
          <div className="control-buttons">
            <h3>CONTROLS {isSafari && !isGamepadConnected && '(Press controller button)'}</h3>
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
                      transform: `translate(${safeControlState.linear.x * 19}px, ${safeControlState.linear.y * 19}px)`
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
                      transform: `translate(${safeControlState.angular.x * 19}px, ${safeControlState.angular.y * 19}px)`
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

              {debugGamepad && (
                <div className="control-group gamepad-debug">
                  <div className="control-label">Gamepad Debug (raw indices)</div>
                  <div className="debug-line">
                    {rawButtons.length === 0 && <span className="debug-pill">No buttons</span>}
                    {rawButtons.map((button) => (
                      <span
                        key={`btn-${button.index}`}
                        className={`debug-pill ${button.pressed ? 'active' : ''}`}
                      >
                        B{button.index}:{button.pressed ? '1' : '0'}
                      </span>
                    ))}
                  </div>
                  <div className="debug-line">
                    {rawAxes.length === 0 && <span className="debug-pill">No axes</span>}
                    {rawAxes.map((axis) => (
                      <span key={`axis-${axis.index}`} className="debug-pill">
                        A{axis.index}:{axis.value.toFixed(2)}
                      </span>
                    ))}
                  </div>
                  <div className="value-display">
                    Expected L1 index: 4 → {rawButtons[4]?.pressed ? 'PRESSED' : 'not pressed'}
                  </div>
                  <div className="value-display">
                    Last emit L1: {lastEmittedButtons?.buttons?.L1 ? 'true' : 'false'}
                    {lastEmittedButtons?.time ? ` @ ${new Date(lastEmittedButtons.time).toLocaleTimeString()}` : ''}
                  </div>
                </div>
              )}
            </div>
          </>
        )}
            </div>

      {showMappings && portalTarget && createPortal(
        <div className="mapping-section visible" onClick={(e) => e.stopPropagation()}>
          <div className="mapping-border"></div>
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
        </div>,
        portalTarget
      )}
    </div>
  );
};

export default ControlOverlay; 
