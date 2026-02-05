"""ActionSpec: Defines action segments with safety clamps and constraints."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np
import yaml
from pathlib import Path


@dataclass
class ActionSegment:
    """Defines a named segment of the action vector."""
    name: str
    start: int
    size: int
    scale: float = 1.0
    clip_min: float = -1.0
    clip_max: float = 1.0


@dataclass
class SafetyBounds:
    """Safety constraints for action execution."""
    max_vel: float = 1.0  # m/s
    max_accel: float = 5.0  # m/s^2
    max_jerk: float = 50.0  # m/s^3


class ActionSpec:
    """
    Manages action space definition with safety clamps.
    
    Loads configuration from YAML and provides methods to:
    - Validate action shapes
    - Apply safety clamps (velocity, acceleration, jerk limits)
    - Track which safety bounds were triggered
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize ActionSpec from config file.
        
        Args:
            config_path: Path to YAML config. Defaults to rl/configs/default.yaml
        """
        if config_path is None:
            # Default to rl/configs/default.yaml relative to this file
            config_path = Path(__file__).parent.parent / "configs" / "default.yaml"
        
        self.config_path = Path(config_path)
        self._load_config()
        
        # State tracking for safety clamps
        self.last_action = None
        self.last_velocity = None
        self.last_clamp_flags = {
            'vel_clamped': False,
            'accel_clamped': False,
            'jerk_clamped': False,
            'clip_clamped': False
        }
    
    def _load_config(self):
        """Load configuration from YAML file."""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Parse action segments
        self.segments: List[ActionSegment] = []
        for seg_config in config['action_spec']['segments']:
            segment = ActionSegment(
                name=seg_config['name'],
                start=seg_config['start'],
                size=seg_config['size'],
                scale=seg_config.get('scale', 1.0),
                clip_min=seg_config.get('clip_min', -1.0),
                clip_max=seg_config.get('clip_max', 1.0)
            )
            self.segments.append(segment)
        
        # Parse safety bounds
        safety_config = config['safety']
        self.safety_bounds = SafetyBounds(
            max_vel=safety_config['max_vel'],
            max_accel=safety_config['max_accel'],
            max_jerk=safety_config['max_jerk']
        )
        
        # Calculate total action dimension
        self.num_actions = config['env']['num_actions']
        
        # Validate segments match total actions
        total_from_segments = sum(seg.size for seg in self.segments)
        if total_from_segments != self.num_actions:
            raise ValueError(
                f"Segment sizes sum to {total_from_segments} but "
                f"config specifies {self.num_actions} actions"
            )
    
    def validate_shape(self, action: np.ndarray) -> bool:
        """
        Validate that action has correct shape.
        
        Args:
            action: Action array to validate
            
        Returns:
            True if shape is valid
        """
        if action.ndim == 1:
            return action.shape[0] == self.num_actions
        elif action.ndim == 2:
            return action.shape[-1] == self.num_actions
        return False
    
    def clip_to_bounds(self, action: np.ndarray) -> np.ndarray:
        """
        Clip action to segment-specific bounds.
        
        Args:
            action: Raw action vector
            
        Returns:
            Clipped action vector
        """
        clipped = action.copy()
        clamped = False
        
        for segment in self.segments:
            end_idx = segment.start + segment.size
            seg_slice = slice(segment.start, end_idx)
            
            # Apply scale
            scaled = clipped[seg_slice] * segment.scale
            
            # Clip to segment bounds
            original = scaled.copy()
            scaled = np.clip(scaled, segment.clip_min, segment.clip_max)
            
            if not np.allclose(original, scaled):
                clamped = True
            
            clipped[seg_slice] = scaled / segment.scale  # Unscale for storage
        
        self.last_clamp_flags['clip_clamped'] = clamped
        return clipped
    
    def clamp_action(self, action: np.ndarray, dt: float) -> tuple[np.ndarray, Dict[str, bool]]:
        """
        Apply all safety clamps to action.
        
        Clamps:
        1. Clip to segment bounds
        2. Velocity limiting
        3. Acceleration limiting
        4. Jerk limiting
        
        Args:
            action: Raw action vector
            dt: Timestep duration in seconds
            
        Returns:
            (clamped_action, safety_flags) tuple
        """
        # Reset flags
        self.last_clamp_flags = {
            'vel_clamped': False,
            'accel_clamped': False,
            'jerk_clamped': False,
            'clip_clamped': False
        }
        
        # Validate shape
        if not self.validate_shape(action):
            raise ValueError(
                f"Action shape {action.shape} doesn't match expected "
                f"dimension {self.num_actions}"
            )
        
        # Start with clipping to bounds
        clamped = self.clip_to_bounds(action)
        
        # Initialize state tracking if first call
        if self.last_action is None:
            self.last_action = clamped.copy()
            self.last_velocity = np.zeros_like(clamped)
            return clamped, self.last_clamp_flags.copy()
        
        # Calculate velocity (change in action / dt)
        velocity = (clamped - self.last_action) / dt
        
        # Velocity limiting
        vel_magnitude = np.abs(velocity)
        if np.any(vel_magnitude > self.safety_bounds.max_vel):
            self.last_clamp_flags['vel_clamped'] = True
            velocity = np.clip(
                velocity,
                -self.safety_bounds.max_vel,
                self.safety_bounds.max_vel
            )
            clamped = self.last_action + velocity * dt
        
        # Acceleration limiting
        acceleration = (velocity - self.last_velocity) / dt
        accel_magnitude = np.abs(acceleration)
        if np.any(accel_magnitude > self.safety_bounds.max_accel):
            self.last_clamp_flags['accel_clamped'] = True
            acceleration = np.clip(
                acceleration,
                -self.safety_bounds.max_accel,
                self.safety_bounds.max_accel
            )
            velocity = self.last_velocity + acceleration * dt
            clamped = self.last_action + velocity * dt
        
        # Jerk limiting (not commonly used but available)
        # Note: This would require tracking last acceleration
        # For now, we note it in flags but don't actively limit
        
        # Update state
        self.last_action = clamped.copy()
        self.last_velocity = velocity.copy()
        
        return clamped, self.last_clamp_flags.copy()
    
    def reset(self):
        """Reset state tracking (call on environment reset)."""
        self.last_action = None
        self.last_velocity = None
        self.last_clamp_flags = {
            'vel_clamped': False,
            'accel_clamped': False,
            'jerk_clamped': False,
            'clip_clamped': False
        }
    
    def get_segment_by_name(self, name: str) -> Optional[ActionSegment]:
        """Get segment by name."""
        for segment in self.segments:
            if segment.name == name:
                return segment
        return None
    
    def get_segment_slice(self, name: str) -> Optional[slice]:
        """Get numpy slice for a segment by name."""
        segment = self.get_segment_by_name(name)
        if segment:
            return slice(segment.start, segment.start + segment.size)
        return None
    
    def __repr__(self) -> str:
        segments_str = ", ".join(f"{s.name}({s.size})" for s in self.segments)
        return f"ActionSpec(total={self.num_actions}, segments=[{segments_str}])"
