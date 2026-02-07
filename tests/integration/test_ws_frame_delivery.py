"""Smoke test: WS upgrade to /stream/ws, confirm 0x01 frame within 2s.

Requires transport-server running with active SHM writer.
"""
import asyncio
import pytest
from websockets.asyncio.client import connect


@pytest.mark.asyncio
async def test_ws_frame_delivery():
    ws = await connect("ws://localhost:8080/stream/ws")
    try:
        msg = await asyncio.wait_for(ws.recv(), timeout=2.0)
        assert len(msg) > 33, "Frame too short (need type + 32-byte header + payload)"
        assert msg[0] == 0x01, f"Expected VIDEO (0x01), got 0x{msg[0]:02x}"
    finally:
        await ws.close()
