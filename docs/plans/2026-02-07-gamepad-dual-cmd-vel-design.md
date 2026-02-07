# Gamepad Dual cmd_vel Design

**Date:** 2026-02-07
**Status:** Approved

## Problem

Gamepad controller inputs are captured in ControlOverlay.jsx at 30Hz but only published to:
- Custom ROS topics (`/controller/joystick_state` as `std_msgs/String`) — not standard
- Socket.io (UI sync between tabs)
- WebRTC DataChannel (when active)

Missing:
1. Standard ROS2 `/cmd_vel` (`geometry_msgs/Twist`) — needed for any ROS2 robot
2. Genesis NATS `set_cmd_vel` — `sendVelocityCommand()` exists but is never called

## Design

### Dual-path from the same 30Hz gamepad loop

```
Gamepad (30Hz)
  ├─ ROS /cmd_vel (geometry_msgs/Twist)     ← NEW: universal ROS2
  ├─ Genesis NATS set_cmd_vel (0x04 WS)     ← NEW: sim safety stack
  ├─ Socket.io joystick_state (unchanged)   ← existing: UI sync
  └─ Custom ROS topic (unchanged)           ← existing: display/logging
```

### ROS Path (universal)
- New `ROSLIB.Topic('/cmd_vel', 'geometry_msgs/Twist')` publisher
- Publishes at 30Hz alongside existing joystick state
- Works with any ROS2 robot subscribing to `/cmd_vel`
- Topic name configurable via SettingsContext (`topics.cmdVel`)

### Genesis NATS Path (sim-specific)
- `sendVelocityCommand(linearX, linearY, angularZ)` already exists in GenesisContext
- Already has Layer 1 safety (videoHealthy gate), TTL (200ms), cmd_seq ordering
- Just needs to be passed as prop and called in the joystick send block
- Only fires when `genesisConnected` is true

### Safety
- ROS path: no gating (real robots handle their own safety)
- Genesis path: 3-layer safety stack unchanged (frontend → transport → sim)

## Code Changes

### ControlOverlay.jsx
- Add `sendVelocityCommand` prop
- Create `/cmd_vel` Twist publisher (lazy, like existing publishers)
- In 30Hz joystick block: publish Twist + call sendVelocityCommand

### App.jsx
- Pull `sendVelocityCommand` from `useGenesis()`
- Pass as prop to `<ControlOverlay>`

### No changes to:
- GenesisContext.jsx (sendVelocityCommand already correct)
- genesis_sim_runner.py (already handles set_cmd_vel via NATS)
- transport-server (already relays 0x04 commands)
- server.js (Socket.io relay unchanged)
