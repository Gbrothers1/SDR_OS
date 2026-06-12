# Raw ROS Control Path Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Claude Code controls the Go2 sim live over ros-mcp at both velocity and joint level, with the web UI tab open, with no canned motions in the repo.

**Architecture:** Two new pure modules (`cmd_arbiter`, `joint_command`) hold all decision logic so they unit-test without Genesis/ROS. The ROS bridge gains a `/sim/joint_cmd` (JointState) subscription forwarded to NATS `command.genesis.set_joint_targets`. The sim runner gains a DIRECT_JOINT branch in `step_sim` that converts raw joint targets into stand-mode action space (`action = target − default_offset`, scale 1.0, kp 50) — the env keeps stepping normally, so obs history and PD plumbing stay intact. `set_cmd_vel` gains per-source seq tracking + activity-based ownership.

**Tech Stack:** Python (rclpy, nats-py, torch), pytest in `tests/unit/`, React (one-line UI tag). Spec: `docs/superpowers/specs/2026-06-12-raw-ros-control-design.md`.

**Key existing facts (verified):**
- `PositionActionManager.step(actions)` applies `actions * _scale_values + _offset_values`; `_offset_values` = default joint positions. Stand mode (`set_stand_gains`, `src/sdr_os/envs/bridge_control.py:28`) sets scale to 1.0 and kp=50.
- Joint layout (ActuatorManager regex order, `bridge_control.py:56-60`):
  `[FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf, RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]`
- URDF defaults: hip 0.0; FL/FR thigh 0.8; RL/RR thigh 1.0; calf −1.6. URDF limits clamped downstream by PositionActionManager.
- Runner: `step_sim` branches at `scripts/genesis_sim_runner.py:1061`; `set_cmd_vel` handler at `:1157`; TTL safety `_enforce_cmd_ttl` at `:1373`; safety telemetry publish at `:1488`. Runner adds project root to `sys.path` and imports `from src.sdr_os...`.
- Bridge: `scripts/ros/cmd_vel_to_nats_bridge.py`, queue-based rclpy→asyncio handoff, single shared `cmd_seq`.
- UI: `sendWsCommand` builds `{action, cmd_seq, data}` at `src/client/contexts/GenesisContext.jsx:375`.
- Live stack runs from the MAIN worktree (`~/Development/SDR_OS` → container `/workspace`); this repo's feature branch holds the canonical copies. Deploy = copy files over + restart `ros-bridge` and `genesis-sim` (NEVER touch the training container).

## File Structure

- Create: `src/sdr_os/control/__init__.py` — empty package marker
- Create: `src/sdr_os/control/joint_command.py` — joint-name→index mapping, targets→actions math (pure torch CPU)
- Create: `src/sdr_os/control/cmd_arbiter.py` — per-source seq + ownership state machine (pure python)
- Create: `scripts/ros/bridge_payloads.py` — NATS payload builders (no rclpy import, testable on host)
- Create: `tests/unit/test_joint_command.py`, `tests/unit/test_cmd_arbiter.py`, `tests/unit/test_bridge_payloads.py`
- Modify: `scripts/ros/cmd_vel_to_nats_bridge.py` — JointState subscription, payload builders, `source: "mcp"`
- Modify: `scripts/genesis_sim_runner.py` — arbiter wiring, `set_joint_targets` handler, DIRECT_JOINT branch, TTL suspension, `cmd_owner` telemetry
- Modify: `src/client/contexts/GenesisContext.jsx` — `source: 'ui'` tag

Run all tests with: `.venv/bin/python3 -m pytest tests/unit/<file> -v` from the repo root.

---

### Task 1: joint_command module (pure math)

**Files:**
- Create: `src/sdr_os/control/__init__.py`
- Create: `src/sdr_os/control/joint_command.py`
- Test: `tests/unit/test_joint_command.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/unit/test_joint_command.py
import pytest
from src.sdr_os.control.joint_command import (
    GO2_JOINT_LAYOUT, normalize_joint_name, resolve_joint_indices,
    merge_joint_targets, targets_to_actions,
)

GO2_DEFAULTS = [0.0, 0.8, -1.6, 0.0, 0.8, -1.6, 0.0, 1.0, -1.6, 0.0, 1.0, -1.6]


def test_layout_is_12_joints():
    assert len(GO2_JOINT_LAYOUT) == 12
    assert GO2_JOINT_LAYOUT[0] == "FL_hip"
    assert GO2_JOINT_LAYOUT[11] == "RR_calf"


def test_normalize_strips_joint_suffix():
    assert normalize_joint_name("FL_hip_joint") == "FL_hip"
    assert normalize_joint_name("FL_hip") == "FL_hip"


def test_resolve_known_names():
    assert resolve_joint_indices(["FR_thigh_joint", "FL_hip"]) == [4, 0]


def test_resolve_unknown_name_raises():
    with pytest.raises(ValueError, match="unknown joint"):
        resolve_joint_indices(["FL_elbow"])


def test_merge_partial_update_holds_others():
    latch = list(GO2_DEFAULTS)
    merged = merge_joint_targets(latch, ["FL_thigh"], [0.3])
    assert merged[1] == 0.3
    assert merged[0] == latch[0] and merged[2] == latch[2]
    assert latch[1] == 0.8  # input not mutated


def test_merge_length_mismatch_raises():
    with pytest.raises(ValueError, match="length"):
        merge_joint_targets(list(GO2_DEFAULTS), ["FL_hip"], [0.1, 0.2])


def test_targets_to_actions_is_target_minus_offset():
    targets = list(GO2_DEFAULTS)
    targets[4] = 1.3  # FR_thigh: 0.8 default → action 0.5
    actions = targets_to_actions(targets, GO2_DEFAULTS)
    assert actions[4] == pytest.approx(0.5)
    assert actions[0] == pytest.approx(0.0)
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `.venv/bin/python3 -m pytest tests/unit/test_joint_command.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'src.sdr_os.control'`

- [ ] **Step 3: Implement**

```python
# src/sdr_os/control/__init__.py
```
(empty file)

```python
# src/sdr_os/control/joint_command.py
"""
Raw joint-target command math for the Go2 bridge envs.

Pure functions, no Genesis/torch imports: the sim runner converts the result
to a tensor. Order matches ActuatorManager regex ordering documented in
src/sdr_os/envs/bridge_control.py:56-60. Action convention is stand mode
(scale=1.0): action = target_rad - default_offset_rad. URDF clamping happens
downstream in PositionActionManager.
"""

GO2_JOINT_LAYOUT = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
]
_INDEX_BY_NAME = {name: i for i, name in enumerate(GO2_JOINT_LAYOUT)}


def normalize_joint_name(name: str) -> str:
    """Accept both 'FL_hip' and URDF-style 'FL_hip_joint'."""
    return name[:-6] if name.endswith("_joint") else name


def resolve_joint_indices(names: list[str]) -> list[int]:
    indices = []
    for raw in names:
        name = normalize_joint_name(raw)
        if name not in _INDEX_BY_NAME:
            raise ValueError(f"unknown joint name: {raw!r}")
        indices.append(_INDEX_BY_NAME[name])
    return indices


def merge_joint_targets(
    latch: list[float], names: list[str], positions: list[float]
) -> list[float]:
    """New latch with named joints updated; unnamed joints hold. Immutable."""
    if len(names) != len(positions):
        raise ValueError(
            f"names/positions length mismatch: {len(names)} vs {len(positions)}"
        )
    merged = list(latch)
    for idx, pos in zip(resolve_joint_indices(names), positions):
        merged[idx] = float(pos)
    return merged


def targets_to_actions(targets: list[float], offsets: list[float]) -> list[float]:
    """Stand-mode actions (scale=1.0): radian offsets from default positions."""
    return [t - o for t, o in zip(targets, offsets)]
```

- [ ] **Step 4: Run tests, verify they pass**

Run: `.venv/bin/python3 -m pytest tests/unit/test_joint_command.py -v`
Expected: 7 passed

- [ ] **Step 5: Commit**

```bash
git add src/sdr_os/control/ tests/unit/test_joint_command.py
git commit -m "feat: joint_command module — raw joint-target math for direct-joint mode"
```

---

### Task 2: cmd_arbiter module (ownership state machine)

**Files:**
- Create: `src/sdr_os/control/cmd_arbiter.py`
- Test: `tests/unit/test_cmd_arbiter.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/unit/test_cmd_arbiter.py
from src.sdr_os.control.cmd_arbiter import CmdVelArbiter


def make():
    return CmdVelArbiter(operator_grace_s=1.0, owner_ttl_s=0.2)


def test_first_nonzero_ui_takes_ownership():
    a = make()
    d = a.evaluate("ui", 1, is_zero=False, now=10.0)
    assert d.accepted and d.apply_velocity and a.owner == "ui"


def test_mcp_owns_when_operator_idle():
    a = make()
    a.evaluate("ui", 1, is_zero=True, now=10.0)   # idle zeros only
    d = a.evaluate("mcp", 1, is_zero=False, now=11.5)
    assert d.apply_velocity and a.owner == "mcp"


def test_mcp_blocked_during_operator_grace():
    a = make()
    a.evaluate("ui", 1, is_zero=False, now=10.0)  # operator active
    d = a.evaluate("mcp", 1, is_zero=False, now=10.5)  # 0.5s later < 1.0s grace
    assert not d.apply_velocity and a.owner == "ui"


def test_ui_nonzero_steals_from_mcp_instantly():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    d = a.evaluate("ui", 1, is_zero=False, now=10.05)
    assert d.apply_velocity and a.owner == "ui"


def test_ui_idle_zeros_never_preempt_mcp():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    d = a.evaluate("ui", 1, is_zero=True, now=10.05)
    assert d.accepted and not d.apply_velocity and a.owner == "mcp"
    assert d.refresh_ttl  # idle stream still feeds safety TTL


def test_owner_zeros_apply():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    d = a.evaluate("mcp", 2, is_zero=True, now=10.05)
    assert d.apply_velocity  # owner's stop command goes through


def test_no_owner_zeros_apply():
    a = make()
    d = a.evaluate("ui", 1, is_zero=True, now=10.0)
    assert d.apply_velocity and a.owner is None


def test_per_source_seq_drop_stale():
    a = make()
    a.evaluate("mcp", 5, is_zero=False, now=10.0)
    d = a.evaluate("mcp", 4, is_zero=False, now=10.01)
    assert not d.accepted


def test_per_source_seq_reset_jump_accepted():
    a = make()
    a.evaluate("mcp", 500, is_zero=False, now=10.0)
    d = a.evaluate("mcp", 1, is_zero=False, now=10.01)  # new session
    assert d.accepted


def test_sources_do_not_share_seq():
    a = make()
    a.evaluate("ui", 9000, is_zero=True, now=10.0)
    d = a.evaluate("mcp", 1, is_zero=False, now=11.5)
    assert d.accepted and d.apply_velocity


def test_owner_expires_when_silent():
    a = make()
    a.evaluate("mcp", 1, is_zero=False, now=10.0)
    assert a.expire_owner(now=10.1) is False     # still fresh
    assert a.expire_owner(now=10.31) is True     # >0.2s silent → released
    assert a.owner is None
    # subsequent ui zeros now apply (functional HOLD with tab open)
    d = a.evaluate("ui", 9001, is_zero=True, now=10.32)
    assert d.apply_velocity
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `.venv/bin/python3 -m pytest tests/unit/test_cmd_arbiter.py -v`
Expected: FAIL — `ImportError: cannot import name 'CmdVelArbiter'`

- [ ] **Step 3: Implement**

```python
# src/sdr_os/control/cmd_arbiter.py
"""
Activity-based arbitration between cmd_vel sources ("ui" operator, "mcp" Claude).

Rules (spec: docs/superpowers/specs/2026-06-12-raw-ros-control-design.md):
- Non-zero UI input always takes ownership instantly (operator authority).
- Non-zero MCP input takes ownership only if the operator has sent no non-zero
  input for >= operator_grace_s.
- Zero-velocity commands never take ownership; non-owner zeros are ignored for
  velocity but still refresh the global safety TTL.
- An owner silent for > owner_ttl_s is released (caller must zero velocity) —
  otherwise a dead MCP stream would coast forever while UI idle zeros keep the
  sim ARMED.
- Per-source monotonic seq with the runner's existing reset heuristic: stale
  within 100 of the max is dropped; a larger backwards jump is a new session.
"""
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class Decision:
    accepted: bool        # passed per-source seq check
    apply_velocity: bool  # forward to env.set_velocity_from_gamepad
    refresh_ttl: bool     # counts as liveness for Layer-3 TTL


_DROP = Decision(accepted=False, apply_velocity=False, refresh_ttl=False)


class CmdVelArbiter:
    def __init__(self, operator_grace_s: float = 1.0, owner_ttl_s: float = 0.2):
        self._grace = operator_grace_s
        self._owner_ttl = owner_ttl_s
        self.owner: str | None = None
        self._last_seq: dict[str, int] = {}
        self._last_nonzero: dict[str, float] = {}
        self._last_msg: dict[str, float] = {}

    def evaluate(
        self, source: str, cmd_seq: int, is_zero: bool, now: float | None = None
    ) -> Decision:
        now = time.monotonic() if now is None else now
        last = self._last_seq.get(source, 0)
        if cmd_seq <= last and cmd_seq > last - 100:
            return _DROP  # out-of-order within the same session
        self._last_seq[source] = cmd_seq
        self._last_msg[source] = now

        if not is_zero:
            self._last_nonzero[source] = now
            if source == "ui":
                self.owner = "ui"
            elif self._operator_idle(now):
                self.owner = source

        apply_velocity = self.owner is None or self.owner == source
        return Decision(accepted=True, apply_velocity=apply_velocity, refresh_ttl=True)

    def expire_owner(self, now: float | None = None) -> bool:
        """Release a silent owner. Returns True if released (caller zeros velocity)."""
        now = time.monotonic() if now is None else now
        if self.owner is None:
            return False
        if now - self._last_msg.get(self.owner, 0.0) > self._owner_ttl:
            self.owner = None
            return True
        return False

    def release(self) -> None:
        self.owner = None

    def _operator_idle(self, now: float) -> bool:
        return now - self._last_nonzero.get("ui", float("-inf")) >= self._grace
```

- [ ] **Step 4: Run tests, verify they pass**

Run: `.venv/bin/python3 -m pytest tests/unit/test_cmd_arbiter.py -v`
Expected: 11 passed

- [ ] **Step 5: Commit**

```bash
git add src/sdr_os/control/cmd_arbiter.py tests/unit/test_cmd_arbiter.py
git commit -m "feat: CmdVelArbiter — per-source seq + activity-based ownership"
```

---

### Task 3: Bridge — payload builders + /sim/joint_cmd subscription

**Files:**
- Create: `scripts/ros/bridge_payloads.py`
- Modify: `scripts/ros/cmd_vel_to_nats_bridge.py`
- Test: `tests/unit/test_bridge_payloads.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/unit/test_bridge_payloads.py
import pytest
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "scripts", "ros"))
from bridge_payloads import cmd_vel_data, joint_targets_data, wrap_command


def test_cmd_vel_data_swaps_axes_and_clamps():
    d = cmd_vel_data(fwd=2.0, strafe=-0.5, yaw=0.25)
    assert d == {
        "linear_y": 1.0, "linear_x": -0.5, "angular_z": 0.25,
        "angular_y": 0.0, "gait_enabled": True,
    }


def test_joint_targets_data():
    d = joint_targets_data(["FL_hip_joint"], [0.2])
    assert d == {"names": ["FL_hip_joint"], "positions": [0.2]}


def test_joint_targets_length_mismatch_raises():
    with pytest.raises(ValueError):
        joint_targets_data(["FL_hip"], [0.1, 0.2])


def test_wrap_command_tags_mcp_source():
    p = wrap_command("set_cmd_vel", 7, {"linear_y": 1.0})
    assert p == {
        "action": "set_cmd_vel", "cmd_seq": 7, "source": "mcp",
        "data": {"linear_y": 1.0},
    }
```

- [ ] **Step 2: Run tests, verify they fail**

Run: `.venv/bin/python3 -m pytest tests/unit/test_bridge_payloads.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'bridge_payloads'`

- [ ] **Step 3: Implement the builders**

```python
# scripts/ros/bridge_payloads.py
"""
Pure NATS payload builders for the ROS→NATS command bridge.

No rclpy/nats imports so they unit-test on the host. The axis swap in
cmd_vel_data is the documented stick→NATS convention (see
cmd_vel_to_nats_bridge.py module docstring): Twist.linear.x (forward) →
data.linear_y, Twist.linear.y (strafe) → data.linear_x.
"""


def _clamp_stick(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def cmd_vel_data(fwd: float, strafe: float, yaw: float) -> dict:
    return {
        "linear_y": _clamp_stick(fwd),
        "linear_x": _clamp_stick(strafe),
        "angular_z": _clamp_stick(yaw),
        "angular_y": 0.0,
        "gait_enabled": True,
    }


def joint_targets_data(names: list, positions: list) -> dict:
    if len(names) != len(positions):
        raise ValueError(
            f"names/positions length mismatch: {len(names)} vs {len(positions)}"
        )
    return {"names": list(names), "positions": [float(p) for p in positions]}


def wrap_command(action: str, cmd_seq: int, data: dict) -> dict:
    return {"action": action, "cmd_seq": cmd_seq, "source": "mcp", "data": data}
```

- [ ] **Step 4: Run tests, verify they pass**

Run: `.venv/bin/python3 -m pytest tests/unit/test_bridge_payloads.py -v`
Expected: 4 passed

- [ ] **Step 5: Rewire the bridge**

In `scripts/ros/cmd_vel_to_nats_bridge.py`:

a. Imports — after `from geometry_msgs.msg import Twist` add:

```python
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bridge_payloads import cmd_vel_data, joint_targets_data, wrap_command
```

b. Add a joint subject constant next to `CMD_SUBJECT`:

```python
JOINT_SUBJECT = "command.genesis.set_joint_targets"
```

c. Replace `_clamp_stick` usage: delete the local `_clamp_stick` function (now in
bridge_payloads). In `CmdVelToNatsBridge.__init__`, add the JointState
subscription after the Twist one:

```python
        self.create_subscription(
            JointState, "/sim/joint_cmd", self._on_joint_state, _CMD_QOS
        )
        self.get_logger().info("Subscribed to /sim/joint_cmd (RELIABLE, VOLATILE)")
```

d. Queue now carries `(subject, data)` tuples. Replace `_on_twist` body and add
`_on_joint_state` (joint commands share the same rate limiter — both paths are
"drive" traffic):

```python
    def _on_twist(self, msg: Twist) -> None:
        """ROS executor thread → hand off to the asyncio/NATS thread."""
        data = cmd_vel_data(fwd=msg.linear.x, strafe=msg.linear.y, yaw=msg.angular.z)
        self._enqueue(CMD_SUBJECT, data)

    def _on_joint_state(self, msg: JointState) -> None:
        try:
            data = joint_targets_data(list(msg.name), list(msg.position))
        except ValueError as e:
            self.get_logger().warning(f"/sim/joint_cmd rejected: {e}")
            return
        self._enqueue(JOINT_SUBJECT, data)

    def _enqueue(self, subject: str, data: dict) -> None:
        now = time.monotonic()
        if now - self._last_recv < MIN_FORWARD_INTERVAL_S or self._loop is None:
            return
        self._last_recv = now
        try:
            self._loop.call_soon_threadsafe(self._queue.put_nowait, (subject, data))
        except (asyncio.QueueFull, RuntimeError):
            pass  # drop rather than block the executor; next message replaces it
```

e. In `nats_loop`, replace the forwarding block (lines 160-173) with:

```python
            try:
                subject, data = await asyncio.wait_for(node._queue.get(), timeout=1.0)
            except asyncio.TimeoutError:
                continue
            cmd_seq += 1
            action = "set_cmd_vel" if subject == CMD_SUBJECT else "set_joint_targets"
            payload = wrap_command(action, cmd_seq, data)
            await nc.publish(subject, json.dumps(payload).encode())
            forwarded += 1
            if forwarded % 100 == 1:
                logger.info(f"forwarded #{forwarded} seq={cmd_seq} {action} {data}")
```

f. Update the module docstring: add a paragraph noting `/sim/joint_cmd`
(`sensor_msgs/msg/JointState`, `name[]`+`position[]` in radians, partial sets
allowed) → `command.genesis.set_joint_targets`, and that every payload now
carries `source: "mcp"`.

- [ ] **Step 6: Syntax check (host has no rclpy — compile only)**

Run: `.venv/bin/python3 -m py_compile scripts/ros/cmd_vel_to_nats_bridge.py && echo OK`
Expected: OK

- [ ] **Step 7: Commit**

```bash
git add scripts/ros/bridge_payloads.py scripts/ros/cmd_vel_to_nats_bridge.py tests/unit/test_bridge_payloads.py
git commit -m "feat: bridge /sim/joint_cmd JointState → NATS set_joint_targets; tag source=mcp"
```

---

### Task 4: Runner — arbiter wiring + cmd_owner telemetry

**Files:**
- Modify: `scripts/genesis_sim_runner.py`

- [ ] **Step 1: Import and construct the arbiter**

After the existing `from src.sdr_os.ipc.shm_ringbuffer import ...` import (line 70):

```python
from src.sdr_os.control.cmd_arbiter import CmdVelArbiter
from src.sdr_os.control.joint_command import (
    merge_joint_targets, targets_to_actions,
)
```

In `__init__` next to the safety state (line ~734), replace `self._last_cmd_seq = 0`
with:

```python
        self._cmd_arbiter = CmdVelArbiter(operator_grace_s=1.0, owner_ttl_s=0.2)
```

`grep -n "_last_cmd_seq" scripts/genesis_sim_runner.py` — the other two uses are
the set_cmd_vel handler (replaced in Step 2) and the ESTOP reset at line 1384
(replaced in Task 5 Step 3). No reads may remain after this task.

- [ ] **Step 2: Replace the set_cmd_vel handler block**

Replace lines 1157-1193 (`if action == "set_cmd_vel": ... continue`) with:

```python
                if action == "set_cmd_vel":
                    source = data.get("source", "ui")  # back-compat: untagged = ui
                    is_zero = all(
                        abs(float(cmd_data.get(k, 0.0))) < 1e-3
                        for k in ("linear_x", "linear_y", "angular_z", "angular_y")
                    )
                    decision = self._cmd_arbiter.evaluate(source, cmd_seq, is_zero)
                    if not decision.accepted:
                        continue  # out-of-order within the same session
                    if decision.refresh_ttl:
                        self._last_cmd_vel_time = time.monotonic()
                        self._cmd_vel_received = True
                    if not decision.apply_velocity:
                        continue  # non-owner (e.g. UI idle zeros while MCP drives)

                    # Non-zero operator input aborts direct-joint mode instantly
                    if source == "ui" and not is_zero and self._joint_targets is not None:
                        logger.info("Operator input — aborting direct-joint mode")
                        self._joint_targets = None

                    self._gait_enabled = bool(cmd_data.get("gait_enabled", False))
                    self._stand_axes = [
                        cmd_data.get("linear_y", 0.0),    # pitch
                        cmd_data.get("linear_x", 0.0),    # roll
                        cmd_data.get("angular_z", 0.0),   # yaw
                        cmd_data.get("angular_y", 0.0),   # height
                    ]
                    self._cmd_log_counter += 1
                    if self._cmd_log_counter % 30 == 1:
                        logger.info(
                            f"cmd_vel: seq={cmd_seq} src={source} owner={self._cmd_arbiter.owner} "
                            f"gait={self._gait_enabled} safety={self._safety_mode} "
                            f"lx={cmd_data.get('linear_x', 0):.3f} "
                            f"ly={cmd_data.get('linear_y', 0):.3f} "
                            f"az={cmd_data.get('angular_z', 0):.3f} "
                            f"ay={cmd_data.get('angular_y', 0):.3f}"
                        )
                    # Recover from HOLD or cmd_timeout ESTOP on fresh command
                    if self._safety_mode in ("HOLD", "ESTOP") and self._safety_reason == "cmd_timeout":
                        logger.info(f"Auto-recovering from {self._safety_mode} on fresh cmd_vel")
                        self._safety_mode = "ARMED"
                        self._safety_reason = "ok"
                    if self.env:
                        self.env.set_velocity_from_gamepad(cmd_data)
                    continue
```

Note: `self._joint_targets` is introduced in Task 5 Step 1 — Tasks 4 and 5 land
as one deploy; run Task 5 before restarting the sim.

- [ ] **Step 3: Owner expiry in the main loop**

In `run()`, immediately before `self._enforce_cmd_ttl()` (line ~1424):

```python
                # Release a silent velocity owner (e.g. MCP stream died while
                # the UI tab's idle zeros keep the TTL fresh) — zero immediately.
                if self._cmd_arbiter.expire_owner():
                    logger.info("cmd_vel owner expired — zeroing velocity")
                    if self.env:
                        self.env.zero_velocity()
                    self._gait_enabled = False
                    self._stand_axes = [0.0, 0.0, 0.0, 0.0]
```

- [ ] **Step 4: cmd_owner in safety telemetry**

In the safety-state publish block (line ~1491), add one field:

```python
                        "cmd_owner": self._cmd_arbiter.owner,
```

- [ ] **Step 5: Syntax check + unit tests still green**

Run: `.venv/bin/python3 -m py_compile scripts/genesis_sim_runner.py && .venv/bin/python3 -m pytest tests/unit/test_cmd_arbiter.py -q`
Expected: compile OK, 11 passed

- [ ] **Step 6: Commit**

```bash
git add scripts/genesis_sim_runner.py
git commit -m "feat: per-source cmd_vel arbitration in sim runner — operator always wins, idle zeros never preempt"
```

---

### Task 5: Runner — set_joint_targets handler + DIRECT_JOINT branch

**Files:**
- Modify: `scripts/genesis_sim_runner.py`

- [ ] **Step 1: State in `__init__`**

Next to the arbiter (Task 4 Step 1):

```python
        # Direct-joint mode: latched 12-dim target list (None = inactive)
        self._joint_targets: list | None = None
        self._last_joint_cmd_time = 0.0
        self._joint_cmd_count = 0
```

Add a freshness helper next to `_enforce_cmd_ttl`:

```python
    JOINT_CMD_TTL_S = 0.5

    def _direct_joint_active(self) -> bool:
        return (
            self._joint_targets is not None
            and time.monotonic() - self._last_joint_cmd_time < self.JOINT_CMD_TTL_S
        )
```

- [ ] **Step 2: Handler in `_handle_commands`**

Insert after the `set_cmd_vel` block (before `try:` / `if action == "pause"`):

```python
                if action == "set_joint_targets":
                    if self._safety_mode == "ESTOP":
                        continue  # require operator re-arm first
                    if not self.env:
                        continue
                    try:
                        names = cmd_data.get("names", [])
                        positions = cmd_data.get("positions", [])
                        if self._joint_targets is None:
                            # Enter mode: latch current joint positions so
                            # unspecified joints hold where they are.
                            dofs_idx = self.env.actuator_manager.dofs_idx
                            current = self.env.robot.get_dofs_position(dofs_idx)
                            if current.dim() == 2:  # batched scene: (n_envs, 12)
                                current = current[0]
                            self._joint_targets = current.cpu().tolist()
                            logger.info("Entering DIRECT_JOINT mode")
                        self._joint_targets = merge_joint_targets(
                            self._joint_targets, names, positions
                        )
                        self._last_joint_cmd_time = time.monotonic()
                        self._joint_cmd_count += 1
                        if self._joint_cmd_count % 20 == 1:
                            logger.info(
                                f"joint_targets #{self._joint_cmd_count}: "
                                f"{dict(zip(names, positions))}"
                            )
                    except ValueError as e:
                        logger.warning(f"set_joint_targets rejected: {e}")
                    continue
```

- [ ] **Step 3: DIRECT_JOINT branch in `step_sim`**

Insert between the ESTOP branch and the WALK branch (line 1072/1073, after
`branch = "ESTOP"` block):

```python
        elif self._direct_joint_active():
            # Raw joint targets from MCP (Claude) — stand gains give scale=1.0
            # so actions are radian offsets from default; PositionActionManager
            # clamps to URDF limits.
            self._switch_gains("stand")
            # _offset_values is 1-D (num_dofs,) — verified in genesis_forge
            # actuator_manager._fill_value_buffer
            offsets = self.env.action_manager._offset_values.cpu().tolist()
            action_list = targets_to_actions(self._joint_targets, offsets)
            actions = torch.tensor([action_list], dtype=torch.float32, device=gs.device)
            branch = "DIRECT_JOINT"
```

Also clear stale mode on expiry — at the top of `step_sim`, before the branch
chain:

```python
        if self._joint_targets is not None and not self._direct_joint_active():
            logger.info("Joint targets stale — exiting DIRECT_JOINT mode")
            self._joint_targets = None
```

- [ ] **Step 4: TTL suspension while joint stream is live**

In `_enforce_cmd_ttl` (line ~1377), after the ESTOP early-return:

```python
        if self._direct_joint_active():
            return  # the joint stream is the liveness signal
```

And replace the line `self._last_cmd_seq = 0  # Reset so fresh...` (line 1384)
with:

```python
            self._cmd_arbiter.release()  # fresh sessions accepted after ESTOP
```

- [ ] **Step 5: Operator ESTOP also exits joint mode**

In the `estop` action handler (line ~1202), after `self.env.zero_velocity()`:

```python
                        self._joint_targets = None
```

- [ ] **Step 6: Safety telemetry shows the mode**

In the safety-state publish dict (next to `cmd_owner` from Task 4):

```python
                        "direct_joint": self._direct_joint_active(),
```

- [ ] **Step 7: Syntax check + full unit suite**

Run: `.venv/bin/python3 -m py_compile scripts/genesis_sim_runner.py && .venv/bin/python3 -m pytest tests/unit/test_joint_command.py tests/unit/test_cmd_arbiter.py tests/unit/test_bridge_payloads.py -q`
Expected: compile OK, 22 passed

- [ ] **Step 8: Commit**

```bash
git add scripts/genesis_sim_runner.py
git commit -m "feat: set_joint_targets handler + DIRECT_JOINT mode — raw joint control from MCP"
```

---

### Task 6: UI source tag

**Files:**
- Modify: `src/client/contexts/GenesisContext.jsx:375`

- [ ] **Step 1: Tag the source**

```javascript
    const cmd = { action, cmd_seq: cmdSeqRef.current, source: 'ui', data };
```

- [ ] **Step 2: Rebuild the bundle**

Run: `npm run build`
Expected: webpack completes without errors, `dist/bundle.js` updated.
(The arbiter defaults untagged → `"ui"`, so the old bundle stays correct; the
rebuild just makes tagging explicit.)

- [ ] **Step 3: Commit**

```bash
git add src/client/contexts/GenesisContext.jsx
git commit -m "feat: tag web UI cmd_vel commands with source=ui for arbitration"
```

---

### Task 7: Deploy to the live stack + smoke test

The live stack runs from the MAIN worktree (`~/Development/SDR_OS`, mounted at
`/workspace` in the containers). The training container is untouchable.

- [ ] **Step 1: Copy changed files to the main worktree**

```bash
cd ~/Development/SDR_OS-power-jump-v2
mkdir -p ~/Development/SDR_OS/src/sdr_os/control
cp src/sdr_os/control/__init__.py src/sdr_os/control/cmd_arbiter.py src/sdr_os/control/joint_command.py ~/Development/SDR_OS/src/sdr_os/control/
cp scripts/ros/bridge_payloads.py scripts/ros/cmd_vel_to_nats_bridge.py ~/Development/SDR_OS/scripts/ros/
cp scripts/genesis_sim_runner.py ~/Development/SDR_OS/scripts/
cp dist/bundle.js ~/Development/SDR_OS/dist/bundle.js
```

- [ ] **Step 2: Restart only the affected services**

```bash
cd ~/Development/SDR_OS
docker compose --profile sim restart ros-bridge genesis-sim
```

Expected: both containers healthy; `docker logs --tail 20` on each shows the
bridge "Subscribed to /sim/joint_cmd" and the runner reaching "Starting sim loop".
Confirm the training container (`sdr_os_training-nats`) was NOT restarted:
`docker ps --format '{{.Names}}\t{{.Status}}' | grep train` still shows its old uptime.

- [ ] **Step 3: Smoke — cmd_vel with the tab open**

With the web UI tab open in the browser, from Claude Code via ros-mcp:
`publish_for_durations(topic='/sim/cmd_vel', msg_type='geometry_msgs/msg/Twist', messages=[{'linear': {'x': 0.4}}], durations=[3.0], rate_hz=20)`
Expected: robot drives; `/odom` position changes; runner log shows
`owner=mcp`; no seq-reset spam.

- [ ] **Step 4: Smoke — joint targets**

`publish_for_durations(topic='/sim/joint_cmd', msg_type='sensor_msgs/msg/JointState', messages=[{'name': ['FL_thigh_joint','FR_thigh_joint'], 'position': [0.5, 0.5]}], durations=[2.0], rate_hz=10)`
Expected: runner log shows "Entering DIRECT_JOINT mode", branch=DIRECT_JOINT in
step_sim log; front thighs move; 0.5s after the stream stops, "Joint targets
stale — exiting DIRECT_JOINT mode" and the robot returns to stand/policy.

- [ ] **Step 5: Smoke — operator override**

While streaming joint targets, move the gamepad in the web UI tab.
Expected: runner log "Operator input — aborting direct-joint mode"; UI control
is instant.

- [ ] **Step 6: Final commit + registry note**

```bash
cd ~/Development/SDR_OS-power-jump-v2
git add -A docs/superpowers/plans/
git commit -m "docs: raw ROS control implementation plan (executed)"
```

Then live verification by the operator: Claude streams sit pose + heart trace
over `/sim/joint_cmd` while Ethan watches the stream (the trick itself is
conversation-driven, not repo code — per spec non-goals).
