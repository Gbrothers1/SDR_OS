# rl/scripts/test_train_bc_device.py
import subprocess
import sys


def test_train_bc_help_shows_auto_default():
    """train_bc.py --help shows device default is 'auto'."""
    result = subprocess.run(
        [sys.executable, "-m", "rl.scripts.train_bc", "--help"],
        capture_output=True,
        text=True
    )
    # Check that auto is shown as the default (argparse shows "(default: auto)")
    assert "(default: auto)" in result.stdout, f"Expected '(default: auto)' in help. Got: {result.stdout}"
