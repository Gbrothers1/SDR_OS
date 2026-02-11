// Skill button bindings for command triggers.
// Update this list to add new skills or change button mappings.
export const SKILL_BINDINGS = [
  {
    button: 'Y',
    skill: 'rear_stand',
    label: 'Rear Stand',
    shortLabel: 'Rear',
    trigger: 'press',
    cooldownMs: 400,
  },
  {
    button: 'X',
    skill: 'jump',
    label: 'Jump',
    shortLabel: 'Jump',
    trigger: 'press',
    cooldownMs: 1200,
  },
  {
    button: 'B',
    skill: 'stand',
    label: 'Stand',
    shortLabel: 'Stand',
    trigger: 'press',
    cooldownMs: 400,
  },
  {
    button: 'A',
    skill: 'sit',
    label: 'Sit',
    shortLabel: 'Sit',
    trigger: 'press',
    cooldownMs: 400,
  },
  // Reserved slots (leave room for more skills)
  {
    button: 'DpadUp',
    skill: null,
    label: 'Skill 5',
    shortLabel: '—',
  },
  {
    button: 'DpadRight',
    skill: null,
    label: 'Skill 6',
    shortLabel: '—',
  },
  {
    button: 'DpadDown',
    skill: null,
    label: 'Skill 7',
    shortLabel: '—',
  },
  {
    button: 'DpadLeft',
    skill: null,
    label: 'Skill 8',
    shortLabel: '—',
  },
];
