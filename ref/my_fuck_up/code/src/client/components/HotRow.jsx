import React, { useState } from 'react';
import HotRowKnob from './HotRowKnob';
import { useTrainingPanel } from '../contexts/TrainingPanelContext';
import '../styles/HotRow.css';

// Initial knob definitions (will expand to 6)
const KNOB_DEFINITIONS = [
  {
    id: 'decimation',
    label: 'Decimation',
    defaultValue: 4,
    min: 1,
    max: 20,
    step: 1,
    type: 'int',
  },
  {
    id: 'cmd_range_vx',
    label: 'Cmd Range',
    defaultValue: 1.0,
    min: 0.1,
    max: 3.0,
    step: 0.1,
    type: 'float',
    subfields: ['vx', 'vy', 'yaw'],
  },
];

export default function HotRow() {
  const { focusZone, editingParam, setEditingParam } = useTrainingPanel();
  const [focusedKnobIndex, setFocusedKnobIndex] = useState(0);
  const [values, setValues] = useState(() => {
    const initial = {};
    KNOB_DEFINITIONS.forEach(k => {
      initial[k.id] = k.defaultValue;
    });
    return initial;
  });
  const [subfieldIndex, setSubfieldIndex] = useState(0);

  const isFocused = focusZone === 'hotrow';

  const handleKnobClick = (index) => {
    setFocusedKnobIndex(index);
    const knob = KNOB_DEFINITIONS[index];
    if (editingParam === knob.id) {
      // Already editing, confirm
      setEditingParam(null);
    } else {
      // Start editing
      setEditingParam(knob.id);
    }
  };

  return (
    <div className={`hotrow ${isFocused ? 'hotrow--focused' : ''}`}>
      {KNOB_DEFINITIONS.map((knob, index) => (
        <HotRowKnob
          key={knob.id}
          label={knob.label}
          value={values[knob.id]}
          subfield={knob.subfields ? knob.subfields[subfieldIndex] : null}
          focused={isFocused && focusedKnobIndex === index}
          editing={editingParam === knob.id}
          overridden={values[knob.id] !== knob.defaultValue}
          onClick={() => handleKnobClick(index)}
        />
      ))}
    </div>
  );
}
