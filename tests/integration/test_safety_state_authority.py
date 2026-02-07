"""Smoke test: verify sim publishes canonical safety state with monotonic state_id.

Requires NATS server and genesis_sim_runner running.
"""
import asyncio
import json
import pytest
import nats as nats_client


@pytest.mark.asyncio
async def test_safety_state_authority():
    nc = await nats_client.connect("nats://localhost:4222")
    try:
        sub = await nc.subscribe("telemetry.safety.state")

        # Collect 3 consecutive safety state messages
        states = []
        for _ in range(3):
            msg = await asyncio.wait_for(sub.next_msg(), timeout=3.0)
            state = json.loads(msg.data.decode())
            states.append(state)

        # Verify required fields
        for state in states:
            assert "state_id" in state
            assert "mode" in state
            assert state["mode"] in ("ARMED", "HOLD", "ESTOP")

        # Verify monotonically increasing state_id
        for i in range(1, len(states)):
            assert states[i]["state_id"] > states[i - 1]["state_id"], (
                f"state_id not monotonic: {states[i-1]['state_id']} -> {states[i]['state_id']}"
            )
    finally:
        await nc.close()
