#!/usr/bin/env python3
"""
PPO Training Script with rsl_rl

Trains policy using PPO algorithm with Genesis VecEnv.
"""

import argparse
import sys
import time
import yaml
from pathlib import Path
from datetime import datetime

import numpy as np
import torch

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from rl.envs import GenesisVecEnv
from rl.utils.device import get_device


def make_env(args, config, device):
    """Create Genesis VecEnv."""
    return GenesisVecEnv(
        env=None,  # Use mock mode without real Genesis
        num_envs=args.num_envs,
        num_obs=config['env']['num_observations'],
        num_actions=config['env']['num_actions'],
        device=device,
        mode="policy"
    )


def make_policy(obs_dim, action_dim, device):
    """Create a simple policy network."""
    import torch.nn as nn
    
    class Policy(nn.Module):
        def __init__(self):
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
        
        def act(self, obs):
            mean = self.actor(obs)
            std = torch.exp(self.log_std)
            dist = torch.distributions.Normal(mean, std)
            action = dist.sample()
            log_prob = dist.log_prob(action).sum(-1)
            return action, log_prob
        
        def evaluate(self, obs, action):
            mean = self.actor(obs)
            std = torch.exp(self.log_std)
            dist = torch.distributions.Normal(mean, std)
            log_prob = dist.log_prob(action).sum(-1)
            entropy = dist.entropy().sum(-1)
            value = self.critic(obs).squeeze(-1)
            return log_prob, entropy, value
    
    return Policy().to(device)


def train_ppo(env, policy, config, args, device):
    """Simple PPO training loop."""
    import torch.optim as optim

    ppo_config = config['training']['ppo']

    optimizer = optim.Adam(policy.parameters(), lr=ppo_config['learning_rate'])

    # Storage for rollouts
    num_steps = ppo_config['num_steps_per_env']
    num_envs = args.num_envs
    obs_dim = config['env']['num_observations']
    action_dim = config['env']['num_actions']

    obs_buffer = torch.zeros(num_steps, num_envs, obs_dim, device=device)
    action_buffer = torch.zeros(num_steps, num_envs, action_dim, device=device)
    log_prob_buffer = torch.zeros(num_steps, num_envs, device=device)
    reward_buffer = torch.zeros(num_steps, num_envs, device=device)
    done_buffer = torch.zeros(num_steps, num_envs, device=device)
    value_buffer = torch.zeros(num_steps, num_envs, device=device)
    
    # Initialize
    obs = env.reset()
    total_steps = 0
    num_updates = args.steps // (num_steps * num_envs)
    
    print(f"Training for {num_updates} updates ({args.steps} total steps)")
    
    episode_rewards = []
    
    for update in range(num_updates):
        # Collect rollouts
        for step in range(num_steps):
            with torch.no_grad():
                action, log_prob = policy.act(obs)
                value = policy.critic(obs).squeeze(-1)
            
            obs_buffer[step] = obs
            action_buffer[step] = action
            log_prob_buffer[step] = log_prob
            value_buffer[step] = value
            
            obs, reward, done, info = env.step(action)
            
            reward_buffer[step] = reward
            done_buffer[step] = done.float()
            
            total_steps += num_envs
            
            # Track episode rewards
            if 'terminal_rewards' in info:
                episode_rewards.extend(info['terminal_rewards'].cpu().numpy().tolist())
        
        # Compute advantages (simple GAE)
        gamma = ppo_config['gamma']
        lam = ppo_config['lam']
        
        with torch.no_grad():
            next_value = policy.critic(obs).squeeze(-1)
        
        advantages = torch.zeros_like(reward_buffer)
        lastgaelam = 0
        for t in reversed(range(num_steps)):
            if t == num_steps - 1:
                next_nonterminal = 1.0 - done_buffer[t]
                next_values = next_value
            else:
                next_nonterminal = 1.0 - done_buffer[t]
                next_values = value_buffer[t + 1]
            
            delta = reward_buffer[t] + gamma * next_values * next_nonterminal - value_buffer[t]
            advantages[t] = lastgaelam = delta + gamma * lam * next_nonterminal * lastgaelam
        
        returns = advantages + value_buffer
        
        # Flatten batches
        b_obs = obs_buffer.reshape(-1, obs_dim)
        b_actions = action_buffer.reshape(-1, action_dim)
        b_log_probs = log_prob_buffer.reshape(-1)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        
        # Normalize advantages
        b_advantages = (b_advantages - b_advantages.mean()) / (b_advantages.std() + 1e-8)
        
        # PPO update
        batch_size = num_steps * num_envs
        minibatch_size = batch_size // ppo_config['num_mini_batches']
        
        indices = np.arange(batch_size)
        np.random.shuffle(indices)
        
        for start in range(0, batch_size, minibatch_size):
            end = start + minibatch_size
            mb_indices = indices[start:end]
            
            mb_obs = b_obs[mb_indices]
            mb_actions = b_actions[mb_indices]
            mb_old_log_probs = b_log_probs[mb_indices]
            mb_advantages = b_advantages[mb_indices]
            mb_returns = b_returns[mb_indices]
            
            # Evaluate actions
            new_log_probs, entropy, values = policy.evaluate(mb_obs, mb_actions)
            
            # Policy loss (clipped)
            ratio = torch.exp(new_log_probs - mb_old_log_probs)
            clip_param = ppo_config['clip_param']
            pg_loss1 = -mb_advantages * ratio
            pg_loss2 = -mb_advantages * torch.clamp(ratio, 1 - clip_param, 1 + clip_param)
            pg_loss = torch.max(pg_loss1, pg_loss2).mean()
            
            # Value loss
            v_loss = ((values - mb_returns) ** 2).mean() * ppo_config['value_loss_coef']
            
            # Entropy loss
            entropy_loss = -entropy.mean() * ppo_config['entropy_coef']
            
            # Total loss
            loss = pg_loss + v_loss + entropy_loss
            
            optimizer.zero_grad()
            loss.backward()
            torch.nn.utils.clip_grad_norm_(policy.parameters(), ppo_config['max_grad_norm'])
            optimizer.step()
        
        # Logging
        if (update + 1) % 10 == 0 or update == num_updates - 1:
            mean_reward = np.mean(episode_rewards[-100:]) if episode_rewards else 0
            print(f"Update {update + 1}/{num_updates}, Steps: {total_steps}, Mean Reward: {mean_reward:.3f}")
    
    return policy


def main():
    parser = argparse.ArgumentParser(
        description="Train PPO policy with rsl_rl",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--config", type=str, default="rl/configs/default.yaml", help="Config file path")
    parser.add_argument("--steps", type=int, default=50000, help="Total training steps")
    parser.add_argument("--num-envs", type=int, default=8, help="Number of parallel environments")
    parser.add_argument("--device", type=str, default="auto", choices=["auto", "cpu", "cuda"], help="Device (auto-detects CUDA)")
    parser.add_argument("--alpha-init", type=float, default=0.0, help="Initial blend alpha")
    parser.add_argument("--checkpoint", type=str, default=None, help="Resume from checkpoint")
    parser.add_argument("--output-dir", type=str, default="rl/checkpoints", help="Output directory")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    args = parser.parse_args()

    device = get_device(args.device)

    print("=" * 60)
    print("PPO Training Script (rsl_rl style)")
    print("=" * 60)
    print(f"Config: {args.config}")
    print(f"Steps: {args.steps}")
    print(f"Num envs: {args.num_envs}")
    print(f"Device: {device} (requested: {args.device})")
    print(f"Seed: {args.seed}")
    print("=" * 60)
    
    # Set seed
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    
    # Load config
    with open(args.config, 'r') as f:
        config = yaml.safe_load(f)
    
    # Create environment
    env = make_env(args, config, device)
    env.seed(args.seed)
    
    print(f"\nEnvironment created:")
    print(f"  num_envs: {env.num_envs}")
    print(f"  num_obs: {env.num_obs}")
    print(f"  num_actions: {env.num_actions}")
    
    # Create policy
    policy = make_policy(env.num_obs, env.num_actions, device)
    
    print(f"\nPolicy created:")
    print(f"  Parameters: {sum(p.numel() for p in policy.parameters()):,}")
    
    # Load checkpoint if provided
    if args.checkpoint:
        print(f"\nLoading checkpoint: {args.checkpoint}")
        checkpoint = torch.load(args.checkpoint, map_location=device)
        policy.load_state_dict(checkpoint['model_state_dict'])
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Train
    print("\nStarting training...")
    start_time = time.time()
    
    policy = train_ppo(env, policy, config, args, device)
    
    elapsed = time.time() - start_time
    print(f"\nTraining complete in {elapsed:.2f}s")
    
    # Save checkpoint
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    checkpoint_path = output_dir / f"ppo_policy_{timestamp}.pt"
    
    torch.save({
        'model_state_dict': policy.state_dict(),
        'obs_dim': env.num_obs,
        'action_dim': env.num_actions,
        'steps': args.steps,
        'config': config
    }, checkpoint_path)
    
    print(f"\nCheckpoint saved to: {checkpoint_path}")
    
    # Cleanup
    env.close()
    
    print("\n" + "=" * 60)
    print("Training complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
