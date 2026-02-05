# genesis_bridge/test_integration_gpu.py
"""Integration tests for GPU pipeline."""
import pytest
import numpy as np


class TestGpuPipelineIntegration:
    """Test GPU pipeline integration with bridge components."""

    def test_gpu_pipeline_encodes_simulated_frame(self):
        """Simulate a frame through the GPU pipeline."""
        from genesis_bridge.gpu_pipeline import GpuFramePipeline

        pipeline = GpuFramePipeline()

        # Simulate 720p RGB frame
        frame = np.random.randint(0, 255, (720, 1280, 3), dtype=np.uint8)

        jpeg = pipeline.encode_jpeg_numpy(frame, quality=80)

        assert len(jpeg) > 1000  # Reasonable JPEG size
        assert jpeg[:2] == b'\xff\xd8'  # JPEG magic

    def test_device_utility_works(self):
        """get_device returns valid torch.device."""
        from rl.utils.device import get_device
        import torch

        device = get_device("auto")
        assert isinstance(device, torch.device)

        device_cpu = get_device("cpu")
        assert device_cpu.type == "cpu"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
