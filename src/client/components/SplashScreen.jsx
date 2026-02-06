import React, { useState, useEffect, useRef, useCallback } from 'react';
import '../styles/SplashScreen.css';

const SplashScreen = ({ onComplete }) => {
  const [phase, setPhase] = useState('logo');   // logo → status → exit
  const [statusLines, setStatusLines] = useState([]);
  const termRef = useRef(null);
  const hasStarted = useRef(false);
  // Stable ref to the latest onComplete — avoids re-triggering the effect
  const onCompleteRef = useRef(onComplete);
  onCompleteRef.current = onComplete;

  // Boot sequence — real-ish status lines, no fake failures
  useEffect(() => {
    if (hasStarted.current) return;
    hasStarted.current = true;

    const lines = [
      { text: 'SDR_OS v1.0  —  System boot', delay: 600 },
      { text: 'Loading UI bundle …', delay: 400 },
      { text: 'Socket.io transport ready', delay: 350 },
      { text: 'Gamepad API available', delay: 300 },
      { text: 'WebGL renderer initialised', delay: 350 },
      { text: 'ROS bridge: will connect on demand', delay: 300, dim: true },
      { text: 'Sim bridge: will connect on demand', delay: 300, dim: true },
      { text: 'All modules loaded — launching cockpit', delay: 500, success: true },
    ];

    let cancelled = false;

    const run = async () => {
      // Hold on logo briefly
      await wait(900);
      if (cancelled) return;
      setPhase('status');

      for (const line of lines) {
        await wait(line.delay);
        if (cancelled) return;
        setStatusLines(prev => [...prev, line]);
      }

      // Brief pause so user can read the last line
      await wait(700);
      if (cancelled) return;
      setPhase('exit');

      // Fade-out transition, then hand off
      await wait(500);
      if (cancelled) return;
      onCompleteRef.current();
    };

    run();
    return () => { cancelled = true; };
  }, []); // empty deps — runs once, uses ref for callback

  // Auto-scroll terminal
  useEffect(() => {
    if (termRef.current) {
      termRef.current.scrollTop = termRef.current.scrollHeight;
    }
  }, [statusLines]);

  // Skip on click / key / gamepad
  const skip = () => {
    if (phase === 'exit') return;
    setPhase('exit');
    setTimeout(onComplete, 120);
  };

  useEffect(() => {
    const onKey = () => skip();
    window.addEventListener('keydown', onKey, { once: true });
    return () => window.removeEventListener('keydown', onKey);
  }, [phase]);

  return (
    <div
      className={`splash ${phase === 'exit' ? 'splash--fade-out' : ''}`}
      onClick={skip}
    >
      {/* Scan-line overlay */}
      <div className="splash__scanlines" />

      {/* Centre logo block */}
      <div className={`splash__logo ${phase !== 'logo' ? 'splash__logo--small' : ''}`}>
        <span className="splash__logo-glyph">SDR</span>
        <span className="splash__logo-sub">Operating System</span>
      </div>

      {/* Boot terminal */}
      {phase === 'status' && (
        <div className="splash__term" ref={termRef}>
          {statusLines.map((l, i) => (
            <div
              key={i}
              className={
                'splash__line' +
                (l.success ? ' splash__line--ok' : '') +
                (l.dim ? ' splash__line--dim' : '')
              }
            >
              <span className="splash__line-prefix">{'>'}</span>
              {l.text}
            </div>
          ))}
          <span className="splash__cursor" />
        </div>
      )}

      {/* Skip hint */}
      {phase !== 'exit' && (
        <div className="splash__skip">press any key or tap to skip</div>
      )}
    </div>
  );
};

function wait(ms) {
  return new Promise(r => setTimeout(r, ms));
}

export default SplashScreen;
