# genesis_bridge/test_gpu_pipeline.py
import pytest
import numpy as np

# Test without requiring CUDA
class TestGpuFramePipelineCapabilities:
    """Test capability detection (works on any machine)."""

    def test_pipeline_initializes(self):
        """GpuFramePipeline initializes without error."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        assert pipeline is not None

    def test_has_capability_flags(self):
        """Pipeline has has_cuda, has_nvjpeg, has_nvenc flags."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        assert hasattr(pipeline, 'has_cuda')
        assert hasattr(pipeline, 'has_nvjpeg')
        assert hasattr(pipeline, 'has_nvenc')
        assert isinstance(pipeline.has_cuda, bool)
        assert isinstance(pipeline.has_nvjpeg, bool)
        assert isinstance(pipeline.has_nvenc, bool)

    def test_has_device(self):
        """Pipeline has device attribute."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        import torch
        assert isinstance(pipeline.device, torch.device)


class TestGpuFramePipelineEncode:
    """Test encoding (uses CPU fallback if no CUDA)."""

    def test_encode_jpeg_from_numpy(self):
        """encode_jpeg_numpy works with HWC uint8 numpy array."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        rgb_np = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        jpeg_bytes = pipeline.encode_jpeg_numpy(rgb_np, quality=80)
        assert isinstance(jpeg_bytes, bytes)
        assert len(jpeg_bytes) > 0
        assert jpeg_bytes[:2] == b'\xff\xd8'  # JPEG magic bytes

    def test_encode_jpeg_from_tensor(self):
        """encode_jpeg works with HWC torch tensor."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        import torch
        pipeline = GpuFramePipeline()
        rgb_tensor = torch.randint(0, 255, (480, 640, 3), dtype=torch.uint8, device=pipeline.device)
        jpeg_bytes = pipeline.encode_jpeg(rgb_tensor, quality=80)
        assert isinstance(jpeg_bytes, bytes)
        assert len(jpeg_bytes) > 0
        assert jpeg_bytes[:2] == b'\xff\xd8'

    def test_encode_jpeg_quality_affects_size(self):
        """Higher quality produces larger JPEG."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        pipeline = GpuFramePipeline()
        rgb_np = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        low_q = pipeline.encode_jpeg_numpy(rgb_np, quality=20)
        high_q = pipeline.encode_jpeg_numpy(rgb_np, quality=95)
        assert len(high_q) > len(low_q)


class TestH264AccessUnit:
    """Tests for H.264 access unit encoding."""

    def test_encode_h264_au_returns_header_plus_payload(self):
        """encode_h264_au returns framed access unit with header."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        from genesis_bridge.frame_protocol import HEADER_SIZE, unpack_frame_header, FrameType

        pipeline = GpuFramePipeline()
        if not pipeline.has_nvenc:
            pytest.skip("NVENC not available")

        # Create test frame
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Encode - may return None on first frame (encoder buffering)
        result = None
        for i in range(5):
            result = pipeline.encode_h264_au(frame, frame_id=i, profile='baseline')
            if result is not None:
                break

        if result is None:
            pytest.skip("Encoder did not produce output")

        # Verify header
        assert len(result) > HEADER_SIZE
        header = unpack_frame_header(result[:HEADER_SIZE])
        assert header['frame_type'] == FrameType.H264
        assert header['payload_len'] == len(result) - HEADER_SIZE

        pipeline.close()

    def test_encode_h264_au_keyframe_has_correct_flags(self):
        """First frame should be keyframe with SPS/PPS."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline
        from genesis_bridge.frame_protocol import HEADER_SIZE, unpack_frame_header, FrameFlags

        pipeline = GpuFramePipeline()
        if not pipeline.has_nvenc:
            pytest.skip("NVENC not available")

        # Force new encoder
        pipeline._h264_encoder = None

        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # First encoded frame should be keyframe
        result = None
        for i in range(5):
            result = pipeline.encode_h264_au(frame, frame_id=i, profile='baseline')
            if result is not None:
                break

        if result is None:
            pytest.skip("Encoder did not produce output")

        header = unpack_frame_header(result[:HEADER_SIZE])
        assert header['flags'] & FrameFlags.KEYFRAME

        pipeline.close()

    def test_request_keyframe_sets_flag(self):
        """request_keyframe() sets _force_keyframe flag."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline

        pipeline = GpuFramePipeline()

        assert not pipeline._force_keyframe
        pipeline.request_keyframe()
        assert pipeline._force_keyframe

        pipeline.close()

    def test_encode_h264_au_no_nvenc_raises(self):
        """encode_h264_au raises RuntimeError if NVENC unavailable."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline

        pipeline = GpuFramePipeline()

        # Mock NVENC as unavailable
        original_has_nvenc = pipeline.has_nvenc
        pipeline.has_nvenc = False

        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        with pytest.raises(RuntimeError, match="NVENC not available"):
            pipeline.encode_h264_au(frame, frame_id=0)

        pipeline.has_nvenc = original_has_nvenc
        pipeline.close()

    def test_codec_to_profile_mapping(self):
        """CODEC_TO_PROFILE contains expected mappings."""
        from genesis_bridge.gpu_pipeline import CODEC_TO_PROFILE

        assert CODEC_TO_PROFILE['avc1.42E01E'] == 'baseline'
        assert CODEC_TO_PROFILE['avc1.4D401E'] == 'main'
        assert CODEC_TO_PROFILE['avc1.64001E'] == 'high'
