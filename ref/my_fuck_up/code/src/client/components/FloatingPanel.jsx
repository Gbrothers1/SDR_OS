import React, { useState } from 'react';
import '../styles/FloatingPanel.css';

const FloatingPanel = ({ 
  title, 
  children, 
  defaultPosition = { x: 20, y: 20 },
  defaultSize = { width: 300, height: 400 },
  resizable = true,
  collapsible = true,
  className = ''
}) => {
  const [position, setPosition] = useState(defaultPosition);
  const [size, setSize] = useState(defaultSize);
  const [isCollapsed, setIsCollapsed] = useState(false);
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const [isResizing, setIsResizing] = useState(false);
  const [resizeStart, setResizeStart] = useState({ x: 0, y: 0, width: 0, height: 0 });

  const handleMouseDown = (e) => {
    if (e.target.closest('.panel-resize-handle')) return;
    setIsDragging(true);
    setDragStart({
      x: e.clientX - position.x,
      y: e.clientY - position.y
    });
  };

  const handleMouseMove = (e) => {
    if (isDragging) {
      setPosition({
        x: Math.max(0, Math.min(window.innerWidth - size.width, e.clientX - dragStart.x)),
        y: Math.max(0, Math.min(window.innerHeight - size.height, e.clientY - dragStart.y))
      });
    }
    if (isResizing) {
      const newWidth = Math.max(200, resizeStart.width + (e.clientX - resizeStart.x));
      const newHeight = Math.max(100, resizeStart.height + (e.clientY - resizeStart.y));
      setSize({
        width: Math.min(800, newWidth),
        height: Math.min(600, newHeight)
      });
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
    setIsResizing(false);
  };

  React.useEffect(() => {
    if (isDragging || isResizing) {
      document.addEventListener('mousemove', handleMouseMove);
      document.addEventListener('mouseup', handleMouseUp);
      return () => {
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
      };
    }
  }, [isDragging, isResizing, dragStart, resizeStart]);

  const handleResizeStart = (e) => {
    e.stopPropagation();
    setIsResizing(true);
    setResizeStart({
      x: e.clientX,
      y: e.clientY,
      width: size.width,
      height: size.height
    });
  };

  return (
    <div
      className={`floating-panel ${className} ${isCollapsed ? 'collapsed' : ''}`}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
        width: isCollapsed ? 'auto' : `${size.width}px`,
        height: isCollapsed ? 'auto' : `${size.height}px`
      }}
      onMouseDown={handleMouseDown}
    >
      <div className="panel-header">
        <span className="panel-title">{title}</span>
        <div className="panel-controls">
          {collapsible && (
            <button
              className="panel-toggle"
              onClick={() => setIsCollapsed(!isCollapsed)}
              title={isCollapsed ? 'Expand' : 'Collapse'}
            >
              {isCollapsed ? '▼' : '▲'}
            </button>
          )}
        </div>
      </div>
      {!isCollapsed && (
        <>
          <div className="panel-content">{children}</div>
          {resizable && (
            <div
              className="panel-resize-handle"
              onMouseDown={handleResizeStart}
            />
          )}
        </>
      )}
    </div>
  );
};

export default FloatingPanel;
