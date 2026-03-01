#!/usr/bin/env python3
"""Train Go2 jump skill policy."""

import os
import sys
import copy
import torch
import shutil
import pickle
import argparse
import glob

# Ensure project root is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

EXPERIMENT_NAME = "go2-jump"

parser = argparse.ArgumentParser(description="Train Go2 jump skill policy")
parser.add_argument("-n", "--num_envs", type=int, default=640)
parser.add_argument("--max_iterations", type=int, default=2000)
parser.add_argument("-d", "--device", type=str, default="gpu")
parser.add_argument("--cuda_device", type=str, default=None, help="CUDA device index (e.g., 1)")
parser.add_argument("--cpu_threads", type=int, default=0, help="CPU threads for PyTorch/BLAS (0=all available cores)")
parser.add_argument("--interop_threads", type=int, default=0, help="PyTorch interop threads (0=auto)")
parser.add_argument("-e", "--exp_name", type=str, default=EXPERIMENT_NAME)
parser.add_argument("--log_dir", type=str, default="rl/checkpoints")
parser.add_argument("--resume", action="store_true", help="Resume from the latest checkpoint in the log dir")
parser.add_argument("--resume_path", type=str, default=None, help="Path to a specific checkpoint to resume")
args = parser.parse_args()


def training_cfg(exp_name: str, max_iterations: int, num_envs: int):
    return {
        "algorithm": {
            "class_name": "PPO",
            "clip_param": 0.2,
            "desired_kl": 0.01,
            "entropy_coef": 0.005,
            "gamma": 0.99,
            "lam": 0.95,
            "learning_rate": 0.0003,
            "max_grad_norm": 1.0,
            "num_learning_epochs": 8,
            "num_mini_batches": 8,
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
            "init_noise_std": 0.5,
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
            "env_name": "go2_jump",
        },
    }


def get_latest_model(log_dir: str) -> str:
    model_checkpoints = glob.glob(os.path.join(log_dir, "model_*.pt"))
    if len(model_checkpoints) == 0:
        raise FileNotFoundError(f"No model checkpoints found in '{log_dir}'")
    model_checkpoints.sort()
    return model_checkpoints[-1]


def _available_cpu_cores() -> int:
    try:
        return max(1, len(os.sched_getaffinity(0)))
    except (AttributeError, OSError):
        return max(1, os.cpu_count() or 1)


def configure_cpu_threads(cpu_threads: int, interop_threads: int) -> tuple[int, int]:
    cores = _available_cpu_cores()
    torch_threads = cores if cpu_threads <= 0 else max(1, min(cpu_threads, cores))
    if interop_threads <= 0:
        torch_interop = max(1, min(4, torch_threads))
    else:
        torch_interop = max(1, min(interop_threads, torch_threads))

    # Configure BLAS/OpenMP pools to match torch thread settings.
    os.environ["OMP_NUM_THREADS"] = str(torch_threads)
    os.environ["MKL_NUM_THREADS"] = str(torch_threads)
    os.environ["OPENBLAS_NUM_THREADS"] = str(torch_threads)
    os.environ["NUMEXPR_NUM_THREADS"] = str(torch_threads)

    torch.set_num_threads(torch_threads)
    try:
        torch.set_num_interop_threads(torch_interop)
    except RuntimeError:
        # Safe fallback if interop threads were already initialized.
        pass
    return torch_threads, torch_interop


def main():
    torch_threads, torch_interop = configure_cpu_threads(args.cpu_threads, args.interop_threads)
    print(
        f"CPU threading: torch_threads={torch_threads}, "
        f"interop_threads={torch_interop}, available_cores={_available_cpu_cores()}"
    )

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
    from rl.envs.go2_skill_env import Go2JumpEnv
    from rsl_rl.runners import OnPolicyRunner

    log_path = os.path.join(args.log_dir, args.exp_name)
    cfg_path = os.path.join(log_path, "cfgs.pkl")

    resume_path = args.resume_path
    if args.resume or resume_path is not None:
        if not os.path.exists(log_path):
            raise FileNotFoundError(f"Log directory not found: {log_path}")
        if os.path.exists(cfg_path):
            [cfg] = pickle.load(open(cfg_path, "rb"))
        else:
            cfg = training_cfg(args.exp_name, args.max_iterations, args.num_envs)
        resume_path = resume_path or get_latest_model(log_path)
        cfg["runner"]["resume"] = True
        cfg["runner"]["resume_path"] = resume_path
        cfg["runner"]["max_iterations"] = args.max_iterations
        cfg["num_steps_per_env"] = round(98_304 / args.num_envs)
        print(f"Resuming from: {resume_path}")
    else:
        if os.path.exists(log_path):
            shutil.rmtree(log_path)
        os.makedirs(log_path, exist_ok=True)
        cfg = training_cfg(args.exp_name, args.max_iterations, args.num_envs)
        pickle.dump([cfg], open(cfg_path, "wb"))
    print(f"Logging to: {log_path}")

    env = Go2JumpEnv(num_envs=args.num_envs, headless=True)
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
    if resume_path is not None:
        runner.load(resume_path)
    runner.learn(num_learning_iterations=args.max_iterations, init_at_random_ep_len=False)
    env.close()


if __name__ == "__main__":
    main()
