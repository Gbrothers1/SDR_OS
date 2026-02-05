"""Tests for per-client stream fanout."""
import pytest
import asyncio
from unittest.mock import AsyncMock


class TestClientStream:
    @pytest.mark.asyncio
    async def test_enqueue_adds_to_queue(self):
        from genesis_bridge.stream_fanout import ClientStream

        mock_ws = AsyncMock()
        client = ClientStream(client_id="test", websocket=mock_ws, supports_h264=False)

        client.enqueue(b'frame1')
        assert client.queue.qsize() == 1

    @pytest.mark.asyncio
    async def test_enqueue_drops_oldest_when_full(self):
        from genesis_bridge.stream_fanout import ClientStream

        mock_ws = AsyncMock()
        client = ClientStream(client_id="test", websocket=mock_ws, supports_h264=False)

        client.enqueue(b'frame1')
        client.enqueue(b'frame2')
        assert client.queue.qsize() == 2

        client.enqueue(b'frame3')
        assert client.queue.qsize() == 2

        frame = client.queue.get_nowait()
        assert frame == b'frame2'

    @pytest.mark.asyncio
    async def test_send_loop_sends_frames(self):
        from genesis_bridge.stream_fanout import ClientStream

        mock_ws = AsyncMock()
        client = ClientStream(client_id="test", websocket=mock_ws, supports_h264=True)

        task = asyncio.create_task(client.start())
        await asyncio.sleep(0.01)

        client.enqueue(b'testframe')
        await asyncio.sleep(0.05)

        client.stop()
        await task

        mock_ws.send.assert_called_with(b'testframe')


class TestStreamFanout:
    @pytest.mark.asyncio
    async def test_add_client_to_correct_group(self):
        from genesis_bridge.stream_fanout import StreamFanout

        fanout = StreamFanout()

        mock_ws_h264 = AsyncMock()
        mock_ws_jpeg = AsyncMock()

        fanout.add_client("c1", mock_ws_h264, supports_h264=True, h264_codec="avc1.42E01E")
        fanout.add_client("c2", mock_ws_jpeg, supports_h264=False)

        assert "c1" in fanout.h264_clients
        assert "c2" in fanout.jpeg_clients
        assert "c1" not in fanout.jpeg_clients
        assert "c2" not in fanout.h264_clients

        await fanout.close_all()

    @pytest.mark.asyncio
    async def test_remove_client(self):
        from genesis_bridge.stream_fanout import StreamFanout

        fanout = StreamFanout()
        mock_ws = AsyncMock()

        fanout.add_client("c1", mock_ws, supports_h264=True)
        assert "c1" in fanout.h264_clients

        await fanout.remove_client("c1")
        assert "c1" not in fanout.h264_clients

    @pytest.mark.asyncio
    async def test_force_keyframe_on_new_h264_client(self):
        from genesis_bridge.stream_fanout import StreamFanout

        fanout = StreamFanout()
        mock_ws = AsyncMock()

        assert fanout.should_force_keyframe() == False

        fanout.add_client("c1", mock_ws, supports_h264=True)
        assert fanout.should_force_keyframe() == True
        assert fanout.should_force_keyframe() == False  # Cleared after read

        await fanout.close_all()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
