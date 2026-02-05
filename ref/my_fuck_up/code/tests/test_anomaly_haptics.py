"""Tests for anomaly-triggered haptic feedback."""

import asyncio
import numpy as np
from unittest.mock import AsyncMock


def test_anomaly_autostop_triggers_vibration(bridge_server):
    """Exceeding targets (action magnitude) should trigger vibration on auto-stop."""
    async def run_test():
        mock_sio = AsyncMock()
        mock_sio.connected = True
        bridge_server.sio = mock_sio

        # Make auto-stop trigger on the first anomaly.
        bridge_server.anomaly_detector.config.consecutive_anomalies_to_stop = 1

        # Create an action that exceeds the configured magnitude threshold.
        threshold = bridge_server.anomaly_detector.config.max_action_magnitude
        action = np.array([threshold + 1.0])

        anomalies = bridge_server.anomaly_detector.check_action(action, bridge_server.config.dt)
        assert anomalies

        if bridge_server.anomaly_detector.update(anomalies):
            bridge_server._handle_anomaly_autostop(anomalies)

        # Let the scheduled emit run.
        await asyncio.sleep(0)

        assert bridge_server.auto_stopped is True
        assert bridge_server.paused is True
        mock_sio.emit.assert_called()
        event, payload = mock_sio.emit.call_args[0]
        assert event == "controller_vibration"
        assert payload.get("reason") == "anomaly_autostop"

    asyncio.run(run_test())
