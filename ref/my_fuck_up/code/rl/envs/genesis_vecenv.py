"""
Genesis VecEnv Wrapper for rsl_rl Compatibility

Wraps Genesis ManagedEnvironment to be compatible with rsl_rl's OnPolicyRunner.
"""

from typing import Optional, Dict, Any, Tuple
import numpy as np
import torch
import yaml
from pathlib import Path


class GenesisVecEnv:
    """
    Vectorized environment wrapper for Genesis + rsl_rl integration.
    
    This wrapper provides the interface expected by rsl_rl's OnPolicyRunner:
    - reset() -> obs
    - step(actions) -> obs, rewards, dones, infos
    - Properties: num_obs, num_actions, num_envs, device
    
    Modes:
    - teleop: Pure human control, no policy
    - policy: Pure policy control
    - blend: Blended teleop + policy with alpha ramping
    """
    
    def __init__(
        self,
        env=None,  # Genesis ManagedEnvironment instance
        num_envs: int = 4096,
        num_obs: int = 48,
        num_actions: int = 12,
        device: str = "cpu",
        config_path: Optional[str] = None,
        mode: str = "policy"
    ):
        """
        Initialize VecEnv wrapper.
        
        Args:
            env: Genesis ManagedEnvironment instance (or None for mock mode)
            num_envs: Number of parallel environments
            num_obs: Observation dimension
            num_actions: Action dimension
            device: Device for tensors ("cpu" or "cuda")
            config_path: Path to config YAML
            mode: Operating mode ("teleop", "policy", "blend")
        """
        self._env = env
        self._num_envs = num_envs
        self._num_obs = num_obs
        self._num_actions = num_actions
        self._device = device
        self._mode = mode
        
        # Load config if provided
        if config_path:
            self._load_config(config_path)
        
        # State tracking
        self._obs = None
        self._step_count = 0
        self._episode_lengths = torch.zeros(num_envs, device=device)
        self._episode_rewards = torch.zeros(num_envs, device=device)
        
        # Blending state (for blend mode)
        self._blend_alpha = 0.0
        self._actor_tag = "policy"  # or "teleop", "blend"
        
        # Goal tracking
        self._current_goals = torch.zeros(num_envs, 7, device=device)  # pos + quat
        
        # Extras info storage
        self._extras = {}
    
    def _load_config(self, config_path: str):
        """Load configuration from YAML."""
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        env_config = config.get('env', {})
        self._num_envs = env_config.get('num_envs', self._num_envs)
        self._num_obs = env_config.get('num_observations', self._num_obs)
        self._num_actions = env_config.get('num_actions', self._num_actions)
    
    @property
    def num_envs(self) -> int:
        """Number of parallel environments."""
        return self._num_envs
    
    @property
    def num_obs(self) -> int:
        """Observation dimension."""
        return self._num_obs
    
    @property
    def num_observations(self) -> int:
        """Alias for num_obs (rsl_rl compatibility)."""
        return self._num_obs
    
    @property
    def num_actions(self) -> int:
        """Action dimension."""
        return self._num_actions
    
    @property
    def device(self) -> str:
        """Device for tensors."""
        return self._device
    
    @property
    def episode_length_buf(self) -> torch.Tensor:
        """Episode lengths buffer (rsl_rl compatibility)."""
        return self._episode_lengths
    
    @property
    def max_episode_length(self) -> int:
        """Maximum episode length."""
        return 500  # Could load from config
    
    def reset(self, env_ids: Optional[torch.Tensor] = None) -> torch.Tensor:
        """
        Reset environments.
        
        Args:
            env_ids: Optional tensor of environment indices to reset.
                     If None, resets all environments.
        
        Returns:
            Initial observations tensor (num_envs, num_obs)
        """
        if env_ids is None:
            # Reset all environments
            if self._env is not None:
                self._obs = self._env.reset()
            else:
                # Mock observation
                self._obs = torch.randn(
                    self._num_envs, self._num_obs,
                    device=self._device
                ) * 0.1
            
            self._episode_lengths.zero_()
            self._episode_rewards.zero_()
        else:
            # Reset specific environments
            if self._env is not None:
                reset_obs = self._env.reset_idx(env_ids)
                self._obs[env_ids] = reset_obs
            else:
                self._obs[env_ids] = torch.randn(
                    len(env_ids), self._num_obs,
                    device=self._device
                ) * 0.1
            
            self._episode_lengths[env_ids] = 0
            self._episode_rewards[env_ids] = 0
        
        self._step_count = 0
        return self._obs
    
    def step(self, actions: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor, Dict]:
        """
        Step the environment.
        
        Args:
            actions: Action tensor (num_envs, num_actions)
        
        Returns:
            obs: Observation tensor (num_envs, num_obs)
            rewards: Reward tensor (num_envs,)
            dones: Done tensor (num_envs,)
            infos: Dictionary with extra information
        """
        self._step_count += 1
        
        if self._env is not None:
            # Step real Genesis environment
            self._obs, rewards, terminated, truncated, extras = self._env.step(actions)
            dones = terminated | truncated
            self._extras = extras
        else:
            # Mock step for testing
            self._obs = torch.randn(
                self._num_envs, self._num_obs,
                device=self._device
            ) * 0.1
            
            # Mock rewards (could use simple reward function)
            rewards = torch.randn(self._num_envs, device=self._device) * 0.1
            
            # Mock dones (episode end after max_episode_length)
            dones = self._episode_lengths >= self.max_episode_length
            
            self._extras = {}
        
        # Update episode tracking
        self._episode_lengths += 1
        self._episode_rewards += rewards
        
        # Build info dict
        infos = {
            "episode_lengths": self._episode_lengths.clone(),
            "episode_rewards": self._episode_rewards.clone(),
            "actor_tag": self._actor_tag,
            "blend_alpha": self._blend_alpha,
            "mode": self._mode,
            **self._extras
        }
        
        # Handle resets for done environments
        done_indices = dones.nonzero(as_tuple=False).flatten()
        if len(done_indices) > 0:
            infos["terminal_observation"] = self._obs[done_indices].clone()
            infos["terminal_rewards"] = self._episode_rewards[done_indices].clone()
            self.reset(done_indices)
        
        return self._obs, rewards, dones, infos
    
    def get_observations(self) -> torch.Tensor:
        """Get current observations (rsl_rl compatibility)."""
        return self._obs
    
    def set_mode(self, mode: str):
        """Set operating mode."""
        assert mode in ["teleop", "policy", "blend"], f"Unknown mode: {mode}"
        self._mode = mode
    
    def set_blend_alpha(self, alpha: float):
        """Set blend alpha for blend mode."""
        self._blend_alpha = max(0.0, min(1.0, alpha))
    
    def set_goals(self, goals: torch.Tensor):
        """Set target goals for all environments."""
        self._current_goals = goals.to(self._device)
    
    def seed(self, seed: int):
        """Set random seed."""
        torch.manual_seed(seed)
        np.random.seed(seed)
        if self._device == "cuda":
            torch.cuda.manual_seed(seed)
    
    def close(self):
        """Clean up resources."""
        if self._env is not None:
            self._env.close()


# For backwards compatibility with rsl_rl
VecEnv = GenesisVecEnv
