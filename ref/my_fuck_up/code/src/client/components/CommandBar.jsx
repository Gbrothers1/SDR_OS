import React, { useState, useRef, useCallback, useEffect } from 'react';
import { useGenesis } from '../contexts/GenesisContext';
import '../styles/CommandBar.css';

// Registry of available commands
const COMMANDS = [
  { cmd: 'load robot', args: '<name>', desc: 'Load a robot by name', action: 'loadRobot' },
  { cmd: 'unload robot', args: '', desc: 'Unload current robot', action: 'unloadRobot' },
  { cmd: 'reset', args: '', desc: 'Reset environment', action: 'reset' },
  { cmd: 'pause', args: '', desc: 'Pause simulation', action: 'pause' },
  { cmd: 'resume', args: '', desc: 'Resume simulation', action: 'resume' },
  { cmd: 'set mode', args: '<mode>', desc: 'Set pipeline mode', action: 'setMode' },
  { cmd: 'set alpha', args: '<0-1>', desc: 'Set blend alpha', action: 'setAlpha' },
  { cmd: 'estop', args: '', desc: 'Emergency stop', action: 'estop' },
];

const CommandBar = ({ onClose, onOpenSettings }) => {
  const [input, setInput] = useState('');
  const [selectedIdx, setSelectedIdx] = useState(0);
  const [recentCommands, setRecentCommands] = useState(() => {
    try {
      return JSON.parse(localStorage.getItem('sdr_recent_commands') || '[]');
    } catch { return []; }
  });
  const inputRef = useRef(null);
  const { loadRobot, unloadRobot, reset, pause, setMode, setAlpha, robotList } = useGenesis();

  // Auto-focus input
  useEffect(() => {
    inputRef.current?.focus();
  }, []);

  // Filter commands based on input
  const suggestions = input.length === 0
    ? COMMANDS
    : COMMANDS.filter(c =>
        c.cmd.includes(input.toLowerCase()) ||
        c.desc.toLowerCase().includes(input.toLowerCase())
      );

  const executeCommand = useCallback((cmdStr) => {
    const trimmed = cmdStr.trim().toLowerCase();

    // Save to recent
    const updated = [cmdStr, ...recentCommands.filter(c => c !== cmdStr)].slice(0, 8);
    setRecentCommands(updated);
    localStorage.setItem('sdr_recent_commands', JSON.stringify(updated));

    // Parse and execute
    if (trimmed.startsWith('load robot ')) {
      const name = cmdStr.trim().slice('load robot '.length).trim();
      loadRobot(name);
    } else if (trimmed === 'unload robot') {
      unloadRobot();
    } else if (trimmed === 'reset') {
      reset();
    } else if (trimmed === 'pause') {
      pause(true);
    } else if (trimmed === 'resume') {
      pause(false);
    } else if (trimmed === 'estop') {
      pause(true);
    } else if (trimmed.startsWith('set mode ')) {
      const mode = cmdStr.trim().slice('set mode '.length).trim();
      setMode(mode);
    } else if (trimmed.startsWith('set alpha ')) {
      const alpha = parseFloat(cmdStr.trim().slice('set alpha '.length));
      if (!isNaN(alpha)) setAlpha(alpha);
    } else if (trimmed === 'settings') {
      if (onOpenSettings) onOpenSettings();
    }

    setInput('');
    if (onClose) onClose();
  }, [recentCommands, loadRobot, unloadRobot, reset, pause, setMode, setAlpha, onClose, onOpenSettings]);

  const handleKeyDown = useCallback((e) => {
    if (e.key === 'ArrowDown') {
      e.preventDefault();
      setSelectedIdx(i => Math.min(i + 1, suggestions.length - 1));
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      setSelectedIdx(i => Math.max(i - 1, 0));
    } else if (e.key === 'Enter') {
      e.preventDefault();
      if (input.trim()) {
        executeCommand(input);
      } else if (suggestions[selectedIdx]) {
        setInput(suggestions[selectedIdx].cmd + ' ');
      }
    } else if (e.key === 'Tab' && suggestions[selectedIdx]) {
      e.preventDefault();
      setInput(suggestions[selectedIdx].cmd + ' ');
    } else if (e.key === 'Escape') {
      if (onClose) onClose();
    }
  }, [input, suggestions, selectedIdx, executeCommand, onClose]);

  return (
    <div className="command-bar">
      <div className="command-bar__input-row">
        <input
          ref={inputRef}
          className="command-bar__input"
          type="text"
          value={input}
          onChange={(e) => { setInput(e.target.value); setSelectedIdx(0); }}
          onKeyDown={handleKeyDown}
          placeholder="Type a command..."
          autoComplete="off"
          spellCheck={false}
        />
      </div>

      {/* Suggestions */}
      {suggestions.length > 0 && (
        <div className="command-bar__suggestions">
          {suggestions.map((s, i) => (
            <div
              key={s.cmd}
              className={`command-bar__suggestion${i === selectedIdx ? ' command-bar__suggestion--selected' : ''}`}
              onClick={() => {
                if (s.args) {
                  setInput(s.cmd + ' ');
                  inputRef.current?.focus();
                } else {
                  executeCommand(s.cmd);
                }
              }}
            >
              <span className="command-bar__suggestion-label">
                {s.cmd}{s.args ? ` ${s.args}` : ''}
              </span>
              <span className="command-bar__suggestion-desc">{s.desc}</span>
            </div>
          ))}
        </div>
      )}

      {/* Recent commands */}
      {recentCommands.length > 0 && (
        <div className="command-bar__recent">
          <span className="command-bar__recent-label">Recent</span>
          {recentCommands.map((cmd, i) => (
            <div
              key={i}
              className="command-bar__recent-item"
              onClick={() => executeCommand(cmd)}
            >
              {cmd}
            </div>
          ))}
        </div>
      )}
    </div>
  );
};

export default CommandBar;
