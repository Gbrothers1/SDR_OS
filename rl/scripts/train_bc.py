#!/usr/bin/env python3
"""
Behavior Cloning Training Script

Trains a policy network to mimic teleop demonstrations.
"""

import argparse
import sys
import time
import glob
from pathlib import Path
from datetime import datetime

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader

from rl.utils.device import get_device


# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))


class TeleopDataset(Dataset):
    """Dataset for teleop demonstrations."""
    
    def __init__(self, data_files):
        self.observations = []
        self.actions = []
        
        for filepath in data_files:
            data = np.load(filepath)
            self.observations.append(data['observations'])
            if 'actions_teleop' in data:
                self.actions.append(data['actions_teleop'])
            elif 'actions' in data:
                self.actions.append(data['actions'])
        
        self.observations = np.concatenate(self.observations, axis=0)
        self.actions = np.concatenate(self.actions, axis=0)
        
        print(f"Loaded {len(self.observations)} transitions from {len(data_files)} files")
    
    def __len__(self):
        return len(self.observations)
    
    def __getitem__(self, idx):
        return (
            torch.tensor(self.observations[idx], dtype=torch.float32),
            torch.tensor(self.actions[idx], dtype=torch.float32)
        )


class BCPolicy(nn.Module):
    """Simple MLP policy for behavior cloning."""
    
    def __init__(self, obs_dim, action_dim, hidden_dims=[256, 256]):
        super().__init__()
        
        layers = []
        in_dim = obs_dim
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(in_dim, hidden_dim))
            layers.append(nn.ELU())
            in_dim = hidden_dim
        layers.append(nn.Linear(in_dim, action_dim))
        layers.append(nn.Tanh())  # Actions in [-1, 1]
        
        self.net = nn.Sequential(*layers)
    
    def forward(self, obs):
        return self.net(obs)


def main():
    parser = argparse.ArgumentParser(
        description="Train BC policy from teleop data",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument("--data-path", type=str, default="rl/datasets/*.npz", help="Glob pattern for data files")
    parser.add_argument("--steps", type=int, default=10000, help="Training steps (batches)")
    parser.add_argument("--device", type=str, default="auto", choices=["auto", "cpu", "cuda"], help="Device (auto-detects CUDA)")
    parser.add_argument("--learning-rate", type=float, default=0.001, help="Learning rate")
    parser.add_argument("--batch-size", type=int, default=256, help="Batch size")
    parser.add_argument("--output-dir", type=str, default="rl/checkpoints", help="Output directory")
    parser.add_argument("--obs-dim", type=int, default=48, help="Observation dimension")
    parser.add_argument("--action-dim", type=int, default=12, help="Action dimension")
    args = parser.parse_args()
    
    # Resolve device
    device = get_device(args.device)

    print("=" * 60)
    print("Behavior Cloning Training Script")
    print("=" * 60)
    print(f"Data path: {args.data_path}")
    print(f"Steps: {args.steps}")
    print(f"Device: {device} (requested: {args.device})")
    print(f"Learning rate: {args.learning_rate}")
    print(f"Batch size: {args.batch_size}")
    print("=" * 60)
    
    # Find data files
    data_files = glob.glob(args.data_path)
    if not data_files:
        print(f"No data files found matching: {args.data_path}")
        print("Generating mock data for demonstration...")
        
        # Generate mock data
        mock_dir = Path("rl/datasets")
        mock_dir.mkdir(parents=True, exist_ok=True)
        mock_file = mock_dir / "mock_teleop_data.npz"
        
        np.savez(
            mock_file,
            observations=np.random.randn(1000, args.obs_dim).astype(np.float32),
            actions_teleop=np.random.randn(1000, args.action_dim).astype(np.float32) * 0.5
        )
        data_files = [str(mock_file)]
    
    # Create dataset and dataloader
    dataset = TeleopDataset(data_files)
    dataloader = DataLoader(dataset, batch_size=args.batch_size, shuffle=True)
    
    # Create model
    policy = BCPolicy(args.obs_dim, args.action_dim).to(device)
    optimizer = optim.Adam(policy.parameters(), lr=args.learning_rate)
    loss_fn = nn.MSELoss()
    
    print(f"\nPolicy architecture:")
    print(policy)
    print(f"\nTotal parameters: {sum(p.numel() for p in policy.parameters()):,}")
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Training loop
    print("\nStarting training...")
    start_time = time.time()
    
    policy.train()
    step = 0
    epoch = 0
    losses = []
    
    while step < args.steps:
        epoch += 1
        for batch_obs, batch_actions in dataloader:
            batch_obs = batch_obs.to(device)
            batch_actions = batch_actions.to(device)
            
            # Forward pass
            pred_actions = policy(batch_obs)
            loss = loss_fn(pred_actions, batch_actions)
            
            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            
            losses.append(loss.item())
            step += 1
            
            # Progress logging
            if step % 100 == 0:
                avg_loss = np.mean(losses[-100:])
                print(f"  Step {step}/{args.steps}, Loss: {avg_loss:.6f}")
            
            if step >= args.steps:
                break
    
    elapsed = time.time() - start_time
    print(f"\nTraining complete in {elapsed:.2f}s")
    print(f"Final loss: {np.mean(losses[-100:]):.6f}")
    
    # Save checkpoint
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    checkpoint_path = output_dir / f"bc_policy_{timestamp}.pt"
    
    torch.save({
        'model_state_dict': policy.state_dict(),
        'optimizer_state_dict': optimizer.state_dict(),
        'obs_dim': args.obs_dim,
        'action_dim': args.action_dim,
        'step': step,
        'loss': np.mean(losses[-100:])
    }, checkpoint_path)
    
    print(f"\nCheckpoint saved to: {checkpoint_path}")
    
    print("\n" + "=" * 60)
    print("Training complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
