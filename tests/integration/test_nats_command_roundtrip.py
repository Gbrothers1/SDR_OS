"""Smoke test: publish command.genesis.pause, confirm telemetry.command.ack.

Requires NATS server and genesis_sim_runner running.
"""
import asyncio
import json
import pytest
import nats as nats_client


@pytest.mark.asyncio
async def test_nats_command_roundtrip():
    nc = await nats_client.connect("nats://localhost:4222")
    try:
        sub = await nc.subscribe("telemetry.command.ack")
        cmd = {"action": "pause", "cmd_seq": 1, "data": {"paused": True}}
        await nc.publish("command.genesis.pause", json.dumps(cmd).encode())

        msg = await asyncio.wait_for(sub.next_msg(), timeout=2.0)
        ack = json.loads(msg.data.decode())
        assert ack["action"] == "pause"
        assert ack["status"] == "ok"
    finally:
        await nc.close()
