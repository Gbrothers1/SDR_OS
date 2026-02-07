import torch
import pytest
from rl.utils.device import get_device


def test_get_device_auto_returns_torch_device():
    """get_device('auto') returns a torch.device."""
    device = get_device("auto")
    assert isinstance(device, torch.device)


def test_get_device_cpu_returns_cpu():
    """get_device('cpu') always returns CPU device."""
    device = get_device("cpu")
    assert device.type == "cpu"


def test_get_device_cuda_returns_cuda_or_raises():
    """get_device('cuda') returns CUDA if available."""
    if torch.cuda.is_available():
        device = get_device("cuda")
        assert device.type == "cuda"
    else:
        # On CPU-only machines, requesting cuda should still return cuda device
        # (torch will error later when actually used)
        device = get_device("cuda")
        assert device.type == "cuda"


def test_get_device_auto_prefers_cuda():
    """get_device('auto') returns CUDA when available, else CPU."""
    device = get_device("auto")
    if torch.cuda.is_available():
        assert device.type == "cuda"
    else:
        assert device.type == "cpu"
