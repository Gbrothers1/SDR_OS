#!/usr/bin/env python3
"""Train Go2 rear-stand skill policy."""

import os
import sys
import copy
import torch
import shutil
import pickle
import argparse

# Ensure project root is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

EXPERIMENT_NAME = "go2-rear-stand"

parser = argparse.ArgumentParser(description="Train Go2 rear-stand skill policy")
parser.add_argument("-n", "--num_envs", type=int, default=2048)
parser.add_argument("--max_iterations", type=int, default=1000)
parser.add_argument("-d", "--device", type=str, default="gpu")
parser.add_argument("--cuda_device", type=str, default=None, help="CUDA device index (e.g., 1)")
parser.add_argument("-e", "--exp_name", type=str, default=EXPERIMENT_NAME)
parser.add_argument("--log_dir", type=str, default="rl/checkpoints")
args = parser.parse_args()


def training_cfg(exp_name: str, max_iterations: int, num_envs: int):
    return {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.01,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.001,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 5,
            "num_mini_batches": 4,
            "schedule": "adaptive",
            "use_clipped_value_loss": True,
            "value_loss_coef": 1.0,
        },
        "init_member_classes": {},
        "actor": {
            "class_name": "MLPModel",
            "activation": "elu",
            "hidden_dims": [512, 256, 128],
            "stochastic": True,
            "init_noise_std": 1.0,
            "noise_std_type": "scalar",
            "state_dependent_std": False,
            "obs_normalization": False,
        },
        "critic": {
            "class_name": "MLPModel",
            "activation": "elu",
            "hidden_dims": [512, 256, 128],
            "stochastic": False,
            "obs_normalization": False,
        },
        "runner": {
            "checkpoint": -1,
            "experiment_name": exp_name,
            "load_run": -1,
            "log_interval": 1,
            "max_iterations": max_iterations,
            "record_interval": -1,
            "resume": False,
            "resume_path": None,
            "run_name": "",
        },
        "runner_class_name": "OnPolicyRunner",
        "seed": 1,
        "num_steps_per_env": round(98_304 / num_envs),
        "save_interval": 100,
        "empirical_normalization": None,
        "obs_groups": {"actor": ["policy"], "critic": ["policy", "critic"]},
        "env": {
            "env_name": "go2_rear_stand",
        },
    }


def main():
    if args.cuda_device is not None:
        os.environ["CUDA_VISIBLE_DEVICES"] = str(args.cuda_device)
        if args.device != "cpu" and torch.cuda.is_available():
            torch.cuda.set_device(0)

    import genesis as gs

    backend = gs.gpu
    if args.device == "cpu":
        backend = gs.cpu
        torch.set_default_device("cpu")
    gs.init(logging_level="warning", backend=backend, performance_mode=True)

    # Import after gs.init() -- genesis requires initialization before entity imports
    from genesis_forge.wrappers import VideoWrapper, RslRlWrapper
    from rl.envs.go2_skill_env import Go2RearStandEnv
    from rsl_rl.runners import OnPolicyRunner

    log_path = os.path.join(args.log_dir, args.exp_name)
    if os.path.exists(log_path):
        shutil.rmtree(log_path)
    os.makedirs(log_path, exist_ok=True)
    print(f"Logging to: {log_path}")

    cfg = training_cfg(args.exp_name, args.max_iterations, args.num_envs)
    pickle.dump([cfg], open(os.path.join(log_path, "cfgs.pkl"), "wb"))

    env = Go2RearStandEnv(num_envs=args.num_envs, headless=True)
    env = VideoWrapper(
        env,
        video_length_sec=8,
        out_dir=os.path.join(log_path, "videos"),
        episode_trigger=lambda episode_id: episode_id % 5 == 0,
    )
    env = RslRlWrapper(env)
    env.cfg = cfg.get("env", {})
    env.build()
    env.reset()

    print("Training model...")
    runner = OnPolicyRunner(env, copy.deepcopy(cfg), log_path, device=gs.device)
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=False)
    env.close()


if __name__ == "__main__":
    main()
