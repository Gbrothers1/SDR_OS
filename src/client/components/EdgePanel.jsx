import React, { useCallback, useEffect } from 'react';
import '../styles/EdgePanel.css';

const EdgePanel = ({ side = 'left', isOpen, onOpen, onClose, pinned, onTogglePin, title, children }) => {
  const handleScrimClick = useCallback(() => {
    if (!pinned && onClose) {
      onClose();
    }
  }, [pinned, onClose]);

  // Close on Escape
  useEffect(() => {
    if (!isOpen) return;

    const handleKey = (e) => {
      if (e.key === 'Escape' && onClose) {
        onClose();
      }
    };
    window.addEventListener('keydown', handleKey);
    return () => window.removeEventListener('keydown', handleKey);
  }, [isOpen, onClose]);

  return (
    <>
      {/* Scrim to catch outside clicks (only when open and not pinned) */}
      {isOpen && !pinned && (
        <div className="edge-panel__scrim" onClick={handleScrimClick} />
      )}

      {/* Collapsed tab handle — visible when panel is closed */}
      {!isOpen && onOpen && side !== 'bottom' && (
        <button
          className={`edge-panel__tab edge-panel__tab--${side}`}
          onClick={onOpen}
          title={title ? `Open ${title}` : `Open ${side} panel`}
        >
          <span className="edge-panel__tab-label">
            {title || side.toUpperCase()}
          </span>
        </button>
      )}

      <div
        className={`edge-panel edge-panel--${side}${isOpen ? ' edge-panel--open' : ''}`}
        role="complementary"
        aria-hidden={!isOpen}
      >
        {/* Pin button */}
        {onTogglePin && (
          <button
            className={`edge-panel__pin${pinned ? ' edge-panel__pin--active' : ''}`}
            onClick={onTogglePin}
            title={pinned ? 'Unpin panel' : 'Pin panel open'}
          >
            {pinned ? '\u25AA' : '\u25AB'}
          </button>
        )}

        <div className="edge-panel__content">
          {children}
        </div>
      </div>
    </>
  );
};

export default EdgePanel;
