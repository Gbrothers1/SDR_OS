# Raw ROS Control Path — Design

**Date:** 2026-06-12
**Status:** Approved by Ethan
**Priority:** #1 — Claude Code must control the robot by sending raw ROS commands over ros-mcp, not via scripted motion programs.

## Goal

Claude Code (via the ros-mcp MCP server → rosbridge) controls the Go2 sim live at two
levels:

1. **Velocity** — `/sim/cmd_vel` (already works), and it must keep working **while the
   web UI tab is open** (today the UI's idle 30 Hz zero-velocity stream fights the MCP
   command stream).
2. **Joints** — a new raw joint-target path so Claude can pose the robot directly
   (e.g. sit, then trace a heart with the front paws). The trajectories themselves are
   computed by Claude at command time and streamed as messages; **no canned motions are
   added to the repo**.

## Non-goals

- No scripted motion player / trick library in the sim.
- No real-robot path (sim only).
- No UI trigger buttons (later, per operator-control doctrine).
- No learned/RL skill for tricks.

## Components

### 1. Bridge: new `/sim/joint_cmd` topic (`scripts/ros/cmd_vel_to_nats_bridge.py`)

- Subscribe `/sim/joint_cmd` (`sensor_msgs/msg/JointState`, VOLATILE durability like
  the existing `/sim/cmd_vel` subscription).
- Forward each message to NATS `command.genesis.set_joint_targets` as
  `{action, cmd_seq, source: "mcp", data: {names: [...], positions: [...]}}`.
- `JointState.name[]` + `position[]` only; `velocity`/`effort` ignored. Partial joint
  sets allowed (e.g. front legs only).
- Exactly-once forwarding, no latching — same doctrine as cmd_vel: the publisher must
  stream to keep control.

### 2. Sim runner: `set_joint_targets` handler + direct-joint mode (`scripts/genesis_sim_runner.py`)

- New action `set_joint_targets`: map joint names → DOF indices (Go2 12 leg joints),
  store targets + receive time.
- **Direct-joint mode:** while the newest joint target is fresher than 500 ms, the RL
  policy is bypassed and targets are applied each frame via
  `robot.control_dofs_position(...)`. Joints not named in the latest command hold
  their previous target (initialized from current positions on mode entry).
- **Fallback:** targets stale > 500 ms → exit direct-joint mode, return to policy
  control and normal cmd_vel safety semantics.
- **Abort:** operator ESTOP, or non-zero operator gamepad input (see arbitration),
  exits direct-joint mode immediately.
- While in direct-joint mode the cmd_vel TTL safety does not ESTOP the sim (the joint
  stream is the liveness signal); safety state telemetry reports mode
  `DIRECT_JOINT` owner `mcp`.
- Unknown joint names → command rejected and logged (warning), mode unchanged.

### 3. Source arbitration for `set_cmd_vel` (runner + UI + bridge)

- All `set_cmd_vel` payloads carry `source: "ui" | "mcp"` (missing → `"ui"` for
  back-compat). Bridge tags `"mcp"`; `GenesisContext.jsx` tags `"ui"`.
- Per-source `cmd_seq` tracking replaces the single global counter (out-of-order
  protection applied per source, same reset heuristic).
- **Ownership rule:**
  - Non-zero UI (operator) input always takes ownership instantly.
  - Non-zero MCP input takes ownership only if the operator has been idle (no
    non-zero input) for ≥ 1.0 s.
  - Zero-velocity commands never take ownership; a non-owner's zeros are ignored for
    velocity but still refresh the safety TTL (the UI idle stream keeps the sim ARMED
    instead of fighting Claude).
  - Owner stream stale past TTL → normal HOLD/ESTOP, ownership released.
- Active owner exposed in `telemetry.safety.state` (`cmd_owner` field).

## Safety summary

| Event | Behavior |
|---|---|
| Operator moves gamepad | Instantly owns velocity; aborts direct-joint mode |
| Operator ESTOP | Zeroes everything, exits direct-joint mode, requires re-arm |
| MCP joint stream stops | 500 ms → back to policy control |
| MCP cmd_vel stream stops | Existing 200 ms HOLD / 2 s ESTOP unchanged |
| UI tab open, idle | Zero stream feeds TTL only; never preempts MCP |

## Testing

- Unit: joint-name→DOF mapping, partial-set hold behavior, per-source seq logic,
  ownership transitions (operator steal, idle-grace, zero-never-owns).
- Smoke: with sim running, stream cmd_vel from MCP with the tab open → robot drives;
  stream joint targets → policy bypassed; stop → clean fallback.
- Live visual: Ethan watches the stream; per project rule, scalar success without
  video confirmation counts for nothing.

## Risks

- **Sit-pretty balance:** both front paws raised shrinks the support polygon; the
  robot may tip under PD control. Mitigation: Claude adjusts pose/keyframes live;
  fallback is a one-paw heart from a three-legged stance. (Risk lives in the
  operator's commands, not this wiring.)
- **PD gains:** policy-tuned kp/kd may be soft for held poses; if so, expose optional
  per-command `kp`/`kd` in `set_joint_targets` data (deferred until observed).
