# Policy Panel + Environment Integration Design

**Date:** 2026-02-07
**Status:** Draft

## Problem

1. Policy panel shows "No policies found" because the `list_policies` command is sent but never handled by the sim runner, and no telemetry response populates `policyList` in the frontend.
2. Training environments (`Go2SkillEnv`, `Go2GaitTrainingEnv`, etc.) live in `~/dev/Genesis/SDR_OS/rl/` and need to be copied into this project.

## Changes

### 1. Copy RL environment suite into `rl/`

Copy from `~/dev/Genesis/SDR_OS/rl/` (excluding `checkpoints/`, `__pycache__/`):

```
rl/
├── envs/               # Go2SkillEnv, GaitTrainingEnv, GamepadEnv, GaitCommandManager, VecEnv
├── actions/            # ActionSpec, controllers, router, teleop bridge
├── configs/            # default.yaml
├── scripts/            # train_*, eval_*, teleop_record
├── utils/              # device utilities
└── checkpoints/        # (already exists)
```

### 2. Add `list_policies` handler to `genesis_sim_runner.py`

In `_handle_commands()`, handle `action == "list_policies"`:

- Scan `rl/checkpoints/` for directories containing `model_*.pt` files
- For each directory, gather: name, path, algorithm (from `cfgs.pkl`), checkpoint list, latest step number, total size, modified timestamp, whether it's currently loaded
- Also scan for standalone `.pt` files
- Publish result on `telemetry.policy.list` subject
- Include `policy_checkpoint` in the regular `telemetry.training.metrics` payload

### 3. Add `telemetry.policy.list` handler to `GenesisContext.jsx`

In the `handleTelemetry` function, add:

```javascript
if (subject === 'telemetry.policy.list') {
  setPolicyList(data.policies || []);
}
```

Also expand `telemetry.command.ack` to handle `list_policies` errors.

### 4. Include checkpoint name in training metrics

Add `policy_checkpoint` field to the metrics dict published every 1s so the `policyCheckpoint` derived state in GenesisContext stays populated. Currently `policyCheckpoint` reads from `trainingMetrics?.policy_checkpoint` but the sim never sends it.

## Files Modified

- `scripts/genesis_sim_runner.py` — add `list_policies` handler + checkpoint name in metrics
- `src/client/contexts/GenesisContext.jsx` — add `telemetry.policy.list` handler
- `rl/envs/` (new) — copied from ~/dev/Genesis/SDR_OS
- `rl/actions/` (new) — copied
- `rl/configs/` (new) — copied
- `rl/scripts/` (new) — copied
- `rl/utils/` (new) — copied
