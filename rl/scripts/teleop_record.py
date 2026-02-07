#!/usr/bin/env python3
"""
Teleop Recording Script

Records human teleop demonstrations for behavior cloning training.
"""

import argparse
import sys
import time
import numpy as np
from pathlib import Path
from datetime import datetime

# Add parent directories to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from rl.actions import ActionSpec, TeleopBridge


def main():
    parser = argparse.ArgumentParser(description="Record teleop demonstrations")
    parser.add_argument("--steps", type=int, default=200, help="Number of steps to record")
    parser.add_argument("--seed", type=int, default=None, help="Random seed")
    parser.add_argument("--num-envs", type=int, default=1, help="Number of parallel environments")
    parser.add_argument("--device", type=str, default="cpu", choices=["cpu", "cuda"], help="Device")
    parser.add_argument("--output-dir", type=str, default="rl/datasets", help="Output directory")
    parser.add_argument("--dt", type=float, default=0.02, help="Simulation timestep")
    args = parser.parse_args()
    
    # Set seed if provided
    if args.seed is not None:
        np.random.seed(args.seed)
    
    print("=" * 60)
    print("Teleop Recording Script")
    print("=" * 60)
    print(f"Steps: {args.steps}")
    print(f"Seed: {args.seed}")
    print(f"Num envs: {args.num_envs}")
    print(f"Device: {args.device}")
    print(f"Output: {args.output_dir}")
    print("=" * 60)
    
    # Initialize action spec and teleop bridge
    action_spec = ActionSpec()
    teleop_bridge = TeleopBridge()
    
    print(f"Action spec: {action_spec}")
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Storage for episode data
    observations = []
    actions_teleop = []
    rewards = []
    dones = []
    goals = []
    
    # Mock observation (would come from real environment)
    obs_dim = 48  # Typical observation dimension
    
    print("\nRecording...")
    print("(In real usage, connect a gamepad and it will capture your inputs)")
    print()
    
    start_time = time.time()
    total_reward = 0.0
    
    for step in range(args.steps):
        # Generate mock observation
        obs = np.random.randn(obs_dim).astype(np.float32) * 0.1
        
        # Get teleop action (mock gamepad input for demonstration)
        # In real usage, this would receive actual gamepad data
        mock_joystick = {
            'linear': {'x': np.sin(step * 0.1) * 0.5, 'y': 0, 'z': 0},
            'angular': {'x': 0, 'y': 0, 'z': np.cos(step * 0.1) * 0.3}
        }
        teleop_action = teleop_bridge.get_action(joystick_state=mock_joystick)
        
        # Apply action spec clamps
        clamped_action, safety_flags = action_spec.clamp_action(teleop_action, args.dt)
        
        # Mock reward
        reward = np.random.randn() * 0.1
        total_reward += reward
        
        # Mock done
        done = step == args.steps - 1
        
        # Store transition
        observations.append(obs)
        actions_teleop.append(clamped_action)
        rewards.append(reward)
        dones.append(done)
        goals.append(np.zeros(7))  # position + quaternion
        
        # Progress
        if (step + 1) % 50 == 0:
            print(f"  Step {step + 1}/{args.steps}, reward so far: {total_reward:.2f}")
    
    elapsed = time.time() - start_time
    print(f"\nRecording complete in {elapsed:.2f}s")
    print(f"Total reward: {total_reward:.2f}")
    print(f"Mean reward: {total_reward / args.steps:.4f}")
    
    # Convert to arrays
    data = {
        'observations': np.stack(observations),
        'actions_teleop': np.stack(actions_teleop),
        'rewards': np.array(rewards),
        'dones': np.array(dones),
        'goals': np.stack(goals)
    }
    
    # Save to file
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = output_dir / f"teleop_episode_{timestamp}.npz"
    
    np.savez_compressed(filename, **data)
    print(f"\nSaved episode to: {filename}")
    
    # Print shapes
    print("\nData shapes:")
    for key, value in data.items():
        print(f"  {key}: {value.shape}")
    
    print("\n" + "=" * 60)
    print("Recording complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
