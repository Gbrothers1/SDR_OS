"""Integration tests for H.264 streaming pipeline."""
import pytest
import numpy as np


class TestH264Pipeline:
    def test_full_encode_pipeline(self):
        """Test encoding a frame through the full pipeline."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        from genesis_bridge.frame_protocol import unpack_frame_header, FrameType, HEADER_SIZE

        pipeline = GpuFramePipeline()

        # Create test frame
        frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)

        if pipeline.has_nvenc:
            # H.264 path
            result = None
            for i in range(5):
                result = pipeline.encode_h264_au(frame, frame_id=i)
                if result:
                    break

            if result:
                assert len(result) > HEADER_SIZE
                header = unpack_frame_header(result[:HEADER_SIZE])
                assert header['frame_type'] == FrameType.H264
                print(f"H.264 frame: {len(result)} bytes, keyframe={bool(header['flags'] & 0x01)}")

        # JPEG path (always available)
        from genesis_bridge.frame_protocol import pack_frame_header
        jpeg_bytes = pipeline.encode_jpeg_numpy(frame, quality=80)
        header = pack_frame_header(FrameType.JPEG, len(jpeg_bytes), 0, 0)
        full_frame = header + jpeg_bytes

        parsed = unpack_frame_header(full_frame[:HEADER_SIZE])
        assert parsed['frame_type'] == FrameType.JPEG
        assert parsed['payload_len'] == len(jpeg_bytes)
        print(f"JPEG frame: {len(full_frame)} bytes")

    def test_fanout_client_management(self):
        """Test adding/removing clients from fanout."""
        import asyncio
        from unittest.mock import AsyncMock
        from genesis_bridge.stream_fanout import StreamFanout

        async def run_test():
            fanout = StreamFanout()

            # Add clients
            ws1 = AsyncMock()
            ws2 = AsyncMock()

            fanout.add_client("h264_client", ws1, supports_h264=True)
            fanout.add_client("jpeg_client", ws2, supports_h264=False)

            assert fanout.has_h264_clients()
            assert fanout.has_jpeg_clients()

            # Remove clients
            await fanout.remove_client("h264_client")
            assert not fanout.has_h264_clients()
            assert fanout.has_jpeg_clients()

            await fanout.close_all()

        asyncio.run(run_test())


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
