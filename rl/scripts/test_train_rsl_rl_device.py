# rl/scripts/test_train_rsl_rl_device.py
import subprocess
import sys


def test_train_rsl_rl_help_shows_auto_default():
    """train_rsl_rl.py --help shows device default is 'auto'."""
    result = subprocess.run(
        [sys.executable, "-m", "rl.scripts.train_rsl_rl", "--help"],
        capture_output=True,
        text=True
    )
    assert "(default: auto)" in result.stdout, f"Expected '(default: auto)' in help. Got: {result.stdout}"
