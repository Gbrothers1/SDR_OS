"""Smoke test: verify sim enters HOLD after command TTL expires.

Requires NATS server and genesis_sim_runner running.
Send a velocity command, wait >200ms without another, confirm safety
state transitions to HOLD.
"""
import asyncio
import json
import pytest
import nats as nats_client


@pytest.mark.asyncio
async def test_cmd_ttl_decay():
    nc = await nats_client.connect("nats://localhost:4222")
    try:
        sub = await nc.subscribe("telemetry.safety.state")

        # Send one velocity command to establish baseline
        cmd = {
            "action": "set_cmd_vel",
            "cmd_seq": 1000,
            "data": {"linear_x": 0.5, "linear_y": 0, "angular_z": 0},
        }
        await nc.publish("command.genesis.set_cmd_vel", json.dumps(cmd).encode())

        # Wait for safety state to transition to HOLD (~200ms TTL + 500ms publish interval)
        deadline = asyncio.get_event_loop().time() + 5.0
        saw_hold = False
        while asyncio.get_event_loop().time() < deadline:
            try:
                msg = await asyncio.wait_for(sub.next_msg(), timeout=1.0)
                state = json.loads(msg.data.decode())
                if state["mode"] == "HOLD":
                    saw_hold = True
                    break
            except asyncio.TimeoutError:
                continue

        assert saw_hold, "Expected safety state to transition to HOLD after command TTL"
    finally:
        await nc.close()
