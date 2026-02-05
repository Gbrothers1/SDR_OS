import React from 'react';
import '../styles/HotRowKnob.css';

export default function HotRowKnob({
  label,
  value,
  subfield,
  focused = false,
  editing = false,
  overridden = false,
  onChange,
  onToggleSubfield,
  onClick,
}) {
  const formatValue = (val) => {
    if (typeof val === 'number') {
      return val.toFixed(2);
    }
    return val;
  };

  return (
    <div
      className={`hotrow-knob ${focused ? 'hotrow-knob--focused' : ''} ${editing ? 'hotrow-knob--editing' : ''} ${overridden ? 'hotrow-knob--overridden' : ''}`}
      onClick={onClick}
    >
      <span className="hotrow-knob__label">{label}</span>
      <span className="hotrow-knob__value">{formatValue(value)}</span>
      {subfield && (
        <span className="hotrow-knob__subfield">{subfield}</span>
      )}
    </div>
  );
}
