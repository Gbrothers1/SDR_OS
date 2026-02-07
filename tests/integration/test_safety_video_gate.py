"""Smoke test: send velocity command when video is stale, confirm video gate fires.

Requires transport-server and NATS running. SHM writer must be stopped
so video appears stale and the gate triggers.
"""
import asyncio
import json
import pytest
import nats as nats_client


@pytest.mark.asyncio
async def test_safety_video_gate():
    nc = await nats_client.connect("nats://localhost:4222")
    try:
        sub = await nc.subscribe("telemetry.safety.video_gate")
        # Send a velocity command to trigger gate check (video will be stale)
        cmd = {
            "action": "set_cmd_vel",
            "cmd_seq": 999,
            "data": {"linear_x": 1.0, "linear_y": 0, "angular_z": 0},
        }
        await nc.publish("command.genesis.set_cmd_vel", json.dumps(cmd).encode())

        # The transport-server should publish video_gate within ~1s
        msg = await asyncio.wait_for(sub.next_msg(), timeout=3.0)
        gate = json.loads(msg.data.decode())
        assert gate["gated"] is True
    finally:
        await nc.close()
