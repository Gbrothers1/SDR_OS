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
