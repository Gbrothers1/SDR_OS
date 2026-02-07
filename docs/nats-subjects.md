# NATS Subject Schema

Binary WS protocol type bytes: `0x01` VIDEO, `0x02` TELEMETRY, `0x03` SIGNALING, `0x04` COMMAND.

## Commands (browser → transport → NATS → sim)

0x04 JSON format: `{"action": "<action>", "cmd_seq": N, "ttl_ms": N, "data": {...}}`

| Subject | Payload |
|---------|---------|
| `command.genesis.set_cmd_vel` | `{linear_x, linear_y, angular_z}` |
| `command.genesis.pause` | `{paused: bool}` |
| `command.genesis.reset` | `{}` |
| `command.genesis.load_robot` | `{robot_name}` |
| `command.genesis.unload_robot` | `{}` |
| `command.genesis.set_mode` | `{mode}` |
| `command.genesis.load_policy` | `{checkpoint_dir, model_file?}` |
| `command.genesis.set_alpha` | `{alpha}` |
| `command.genesis.estop` | `{reason?}` |
| `command.genesis.estop_clear` | `{cmd_seq}` |
| `command.genesis.camera` | `{position, lookat}` |
| `command.genesis.settings` | `{jpeg_quality, stream_fps, camera_res}` |

## Telemetry (sim → NATS → transport → browser)

Relayed as 0x02 WS messages: `[0x02][u16 subject_len][subject][JSON payload]`

| Subject | Rate | Payload |
|---------|------|---------|
| `telemetry.training.metrics` | 1 Hz | `{step, fps, policy_loaded, total_reward, episode_length}` |
| `telemetry.frame.stats` | 1 Hz | `{frame_id, encode_ms, codec, frame_bytes}` |
| `telemetry.reward.breakdown` | 1 Hz | `{per_component_rewards}` |
| `telemetry.obs.breakdown` | 1 Hz | `{per_component_observations}` |
| `telemetry.velocity.command` | 5 Hz | `{linear_x, linear_y, angular_z}` |
| `telemetry.safety.state` | 2 Hz | `{state_id, mode, reason, since_ms}` |
| `telemetry.safety.cmd_timeout` | on event | `{mode, decaying, velocity}` |
| `telemetry.command.ack` | on event | `{action, cmd_seq, status, detail}` |

## Safety (transport → NATS)

| Subject | Rate | Payload |
|---------|------|---------|
| `telemetry.safety.video_gate` | on event | `{gated, mode, stale_ms}` |

## Safety Modes

- **ARMED** — normal operation
- **HOLD** — zero velocity, system armed, auto-recoverable
- **ESTOP** — zero velocity, policy disabled, requires operator re-arm

## SHM Codec Values

- `1` = H.264
- `2` = HEVC
- `3` = JPEG
