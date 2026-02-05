#!/usr/bin/env python3
"""
Policy Evaluation Script

Evaluates a trained policy on the Genesis environment.
"""

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch
import torch.nn as nn

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from rl.envs import GenesisVecEnv


class Policy(nn.Module):
    """Policy network matching the training architecture."""
    
    def __init__(self, obs_dim, action_dim):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, action_dim),
            nn.Tanh()
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ELU(),
            nn.Linear(256, 256),
            nn.ELU(),
            nn.Linear(256, 1)
        )
        self.log_std = nn.Parameter(torch.zeros(action_dim))
    
    def forward(self, obs):
        return self.actor(obs)


def evaluate_policy(env, policy, num_episodes, device):
    """Run evaluation episodes."""
    episode_rewards = []
    episode_lengths = []
    
    obs = env.reset()
    current_rewards = torch.zeros(env.num_envs, device=device)
    current_lengths = torch.zeros(env.num_envs, device=device)
    
    episodes_completed = 0
    
    while episodes_completed < num_episodes:
        # Get action from policy
        with torch.no_grad():
            action = policy(obs)
        
        # Step environment
        obs, reward, done, info = env.step(action)
        
        current_rewards += reward
        current_lengths += 1
        
        # Check for completed episodes
        done_indices = done.nonzero(as_tuple=False).flatten()
        for idx in done_indices:
            episode_rewards.append(current_rewards[idx].item())
            episode_lengths.append(current_lengths[idx].item())
            current_rewards[idx] = 0
            current_lengths[idx] = 0
            episodes_completed += 1
            
            if episodes_completed >= num_episodes:
                break
    
    return episode_rewards, episode_lengths


def main():
    parser = argparse.ArgumentParser(description="Evaluate trained policy")
    parser.add_argument("--checkpoint", type=str, required=True, help="Path to checkpoint")
    parser.add_argument("--num-envs", type=int, default=4, help="Number of parallel environments")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"], help="Device")
    parser.add_argument("--goal-mode", type=str, default="none", help="Goal mode")
    parser.add_argument("--episodes", type=int, default=100, help="Number of evaluation episodes")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()
    
    print("=" * 60)
    print("Policy Evaluation Script")
    print("=" * 60)
    print(f"Checkpoint: {args.checkpoint}")
    print(f"Num envs: {args.num_envs}")
    print(f"Device: {args.device}")
    print(f"Episodes: {args.episodes}")
    print("=" * 60)
    
    # Set seed
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    
    # Load checkpoint
    checkpoint_path = Path(args.checkpoint)
    if not checkpoint_path.exists():
        print(f"Error: Checkpoint not found: {args.checkpoint}")
        print("Creating mock evaluation...")
        obs_dim = 48
        action_dim = 12
    else:
        checkpoint = torch.load(args.checkpoint, map_location=args.device)
        obs_dim = checkpoint.get('obs_dim', 48)
        action_dim = checkpoint.get('action_dim', 12)
    
    # Create environment
    env = GenesisVecEnv(
        env=None,
        num_envs=args.num_envs,
        num_obs=obs_dim,
        num_actions=action_dim,
        device=args.device,
        mode="policy"
    )
    env.seed(args.seed)
    
    print(f"\nEnvironment created:")
    print(f"  num_envs: {env.num_envs}")
    print(f"  num_obs: {env.num_obs}")
    print(f"  num_actions: {env.num_actions}")
    
    # Create and load policy
    policy = Policy(obs_dim, action_dim).to(args.device)
    
    if checkpoint_path.exists():
        policy.load_state_dict(checkpoint['model_state_dict'])
        print(f"\nLoaded policy from checkpoint")
    else:
        print(f"\nUsing random policy (no checkpoint)")
    
    policy.eval()
    
    # Run evaluation
    print(f"\nRunning {args.episodes} evaluation episodes...")
    start_time = time.time()
    
    episode_rewards, episode_lengths = evaluate_policy(
        env, policy, args.episodes, args.device
    )
    
    elapsed = time.time() - start_time
    
    # Compute statistics
    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)
    min_reward = np.min(episode_rewards)
    max_reward = np.max(episode_rewards)
    
    mean_length = np.mean(episode_lengths)
    std_length = np.std(episode_lengths)
    
    print(f"\n" + "=" * 60)
    print("Evaluation Results")
    print("=" * 60)
    print(f"Episodes evaluated: {len(episode_rewards)}")
    print(f"Time: {elapsed:.2f}s ({len(episode_rewards) / elapsed:.1f} episodes/s)")
    print()
    print("Rewards:")
    print(f"  Mean:   {mean_reward:.3f}")
    print(f"  Std:    {std_reward:.3f}")
    print(f"  Min:    {min_reward:.3f}")
    print(f"  Max:    {max_reward:.3f}")
    print()
    print("Episode Lengths:")
    print(f"  Mean:   {mean_length:.1f}")
    print(f"  Std:    {std_length:.1f}")
    print("=" * 60)
    
    # Cleanup
    env.close()


if __name__ == "__main__":
    main()
