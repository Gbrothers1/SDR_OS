import React, { useEffect, useRef } from 'react';

const ControlOverlay = ({ onControlChange, controlState }) => {
  const overlayRef = useRef();
  const gamepadRef = useRef(null);
  const animationFrameRef = useRef();

  useEffect(() => {
    const handleGamepadConnected = (e) => {
      console.log('Gamepad connected:', e.gamepad);
      gamepadRef.current = e.gamepad;
    };

    const handleGamepadDisconnected = (e) => {
      console.log('Gamepad disconnected:', e.gamepad);
      if (gamepadRef.current && gamepadRef.current.index === e.gamepad.index) {
        gamepadRef.current = null;
      }
    };

    window.addEventListener('gamepadconnected', handleGamepadConnected);
    window.addEventListener('gamepaddisconnected', handleGamepadDisconnected);

    // Start gamepad polling
    const pollGamepad = () => {
      if (gamepadRef.current) {
        const gamepads = navigator.getGamepads();
        const gamepad = gamepads[gamepadRef.current.index];

        if (gamepad) {
          // Steam Deck control mapping
          const newState = {
            linear: {
              x: gamepad.axes[0] * 1.0, // Left stick X
              y: -gamepad.axes[1] * 1.0, // Left stick Y
              z: (gamepad.buttons[6].value - gamepad.buttons[7].value) * 0.5 // L2 - R2
            },
            angular: {
              x: gamepad.axes[2] * 1.0, // Right stick X
              y: -gamepad.axes[3] * 1.0, // Right stick Y
              z: (gamepad.buttons[4].value - gamepad.buttons[5].value) * 1.0 // L1 - R1
            }
          };

          onControlChange(newState);
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
    };
  }, [onControlChange]);

  return (
    <div ref={overlayRef} style={styles.overlay}>
      {/* Left Stick */}
      <div style={styles.stick}>
        <div style={styles.stickLabel}>Left Stick</div>
        <div style={styles.stickIndicator}>
          <div style={{
            ...styles.stickDot,
            transform: `translate(${controlState.linear.x * 20}px, ${-controlState.linear.y * 20}px)`
          }} />
        </div>
        <div style={styles.stickValues}>
          X: {controlState.linear.x.toFixed(2)}
          Y: {controlState.linear.y.toFixed(2)}
        </div>
      </div>

      {/* Right Stick */}
      <div style={{...styles.stick, right: 20}}>
        <div style={styles.stickLabel}>Right Stick</div>
        <div style={styles.stickIndicator}>
          <div style={{
            ...styles.stickDot,
            transform: `translate(${controlState.angular.x * 20}px, ${-controlState.angular.y * 20}px)`
          }} />
        </div>
        <div style={styles.stickValues}>
          X: {controlState.angular.x.toFixed(2)}
          Y: {controlState.angular.y.toFixed(2)}
        </div>
      </div>

      {/* Triggers */}
      <div style={styles.triggers}>
        <div>L2: {((controlState.linear.z + 0.5) * 100).toFixed(0)}%</div>
        <div>R2: {((-controlState.linear.z + 0.5) * 100).toFixed(0)}%</div>
      </div>
    </div>
  );
};

const styles = {
  overlay: {
    position: 'absolute',
    top: 0,
    left: 0,
    right: 0,
    bottom: 0,
    pointerEvents: 'none',
    zIndex: 1000,
  },
  stick: {
    position: 'absolute',
    bottom: 20,
    left: 20,
    background: 'rgba(0, 0, 0, 0.5)',
    padding: 10,
    borderRadius: 5,
  },
  stickLabel: {
    fontSize: 12,
    marginBottom: 5,
    color: '#fff',
  },
  stickIndicator: {
    width: 40,
    height: 40,
    border: '1px solid rgba(255, 255, 255, 0.3)',
    borderRadius: '50%',
    position: 'relative',
  },
  stickDot: {
    width: 8,
    height: 8,
    background: '#fff',
    borderRadius: '50%',
    position: 'absolute',
    top: '50%',
    left: '50%',
    marginLeft: -4,
    marginTop: -4,
  },
  stickValues: {
    fontSize: 10,
    marginTop: 5,
    color: '#fff',
  },
  triggers: {
    position: 'absolute',
    top: 20,
    right: 20,
    background: 'rgba(0, 0, 0, 0.5)',
    padding: 10,
    borderRadius: 5,
    color: '#fff',
    fontSize: 12,
  },
};

export default ControlOverlay; 