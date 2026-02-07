"""Device utilities for RL training and inference."""
import torch


def get_device(requested: str = "auto") -> torch.device:
    """Resolve device string to torch.device.

    Args:
        requested: "auto", "cuda", or "cpu"

    Returns:
        torch.device - cuda:0 if available and requested != "cpu", else cpu
    """
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(requested)
