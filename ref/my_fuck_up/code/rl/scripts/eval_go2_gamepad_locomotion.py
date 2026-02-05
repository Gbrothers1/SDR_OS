#!/usr/bin/env python3
"""Evaluate a trained Go2 gamepad locomotion policy."""

import os
import sys
import glob
import torch
import pickle
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

import genesis as gs

EXPERIMENT_NAME = "go2-gamepad-locomotion"

parser = argparse.ArgumentParser(description="Evaluate Go2 gamepad locomotion policy")
parser.add_argument("-d", "--device", type=str, default="gpu")
parser.add_argument("-e", "--exp_name", type=str, default=EXPERIMENT_NAME)
parser.add_argument("--log_dir", type=str, default="rl/checkpoints")
parser.add_argument("--checkpoint", type=str, default=None, help="Specific checkpoint path")
parser.add_argument("--max_lin_vel", type=float, default=3.5)
parser.add_argument("--max_lat_vel", type=float, default=0.5)
parser.add_argument("--max_ang_vel", type=float, default=5.0)
parser.add_argument("--command_resample_time", type=float, default=1.5)
args = parser.parse_args()


def get_latest_model(log_dir: str) -> str:
    model_checkpoints = glob.glob(os.path.join(log_dir, "model_*.pt"))
    if len(model_checkpoints) == 0:
        print(f"No model files found at '{log_dir}'")
        sys.exit(1)
    model_checkpoints.sort()
    return model_checkpoints[-1]


def main():
    backend = gs.gpu
    if args.device == "cpu":
        backend = gs.cpu
        torch.set_default_device("cpu")
    gs.init(logging_level="warning", backend=backend)

    # Import after gs.init() -- genesis requires initialization before entity imports
    from genesis_forge.wrappers import RslRlWrapper
    from rl.envs.go2_gamepad_env import Go2GamepadLocomotionEnv
    from rsl_rl.runners import OnPolicyRunner

    log_path = os.path.join(args.log_dir, args.exp_name)
    [cfg] = pickle.load(open(os.path.join(log_path, "cfgs.pkl"), "rb"))
    model = args.checkpoint or get_latest_model(log_path)

    env = Go2GamepadLocomotionEnv(
        num_envs=1,
        headless=False,
        max_lin_vel=args.max_lin_vel,
        max_lat_vel=args.max_lat_vel,
        max_ang_vel=args.max_ang_vel,
        command_resample_time=args.command_resample_time,
    )
    env = RslRlWrapper(env)
    env.build()

    print(f"Loading model: {model}")
    runner = OnPolicyRunner(env, cfg, log_path, device=gs.device)
    runner.load(model)
    policy = runner.get_inference_policy(device=gs.device)

    obs, _ = env.reset()
    try:
        with torch.no_grad():
            while True:
                actions = policy(obs)
                obs, _, _, _ = env.step(actions)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if "Viewer closed" not in str(e):
            raise


if __name__ == "__main__":
    main()
