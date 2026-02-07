"""TeleopBridge: Converts gamepad input to normalized action vectors."""

from typing import Dict, Optional
import numpy as np
import yaml
from pathlib import Path
from collections import deque


class TeleopBridge:
    """
    Normalizes raw gamepad state to flat action vectors.
    
    Features:
    - Maps joystick/button states to action segments
    - Low-pass filtering to smooth rapid changes
    - Rate limiting to prevent input flooding
    - Deadman switch detection (L1 button)
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize TeleopBridge from config file.
        
        Args:
            config_path: Path to YAML config. Defaults to rl/configs/default.yaml
        """
        if config_path is None:
            config_path = Path(__file__).parent.parent / "configs" / "default.yaml"
        
        self.config_path = Path(config_path)
        self._load_config()
        
        # Input state buffers
        self.last_joystick_state = None
        self.last_button_states = None
        self.deadman_active = False
        
        # Low-pass filter (simple exponential moving average)
        self.filter_alpha = 0.3  # Higher = more responsive, lower = more smooth
        self.filtered_action = None
        
        # Rate limiting
        self.min_update_interval = 0.01  # seconds (100 Hz max)
        self.last_update_time = 0.0
        
        # Input history for debugging
        self.input_history = deque(maxlen=100)
    
    def _load_config(self):
        """Load action spec from config."""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Store action dimensions
        self.num_actions = config['env']['num_actions']
        
        # Parse action segments for mapping
        self.segments = {}
        for seg in config['action_spec']['segments']:
            self.segments[seg['name']] = {
                'start': seg['start'],
                'size': seg['size'],
                'scale': seg.get('scale', 1.0),
                'clip_min': seg.get('clip_min', -1.0),
                'clip_max': seg.get('clip_max', 1.0)
            }
    
    def normalize_joystick_value(self, value: float, deadzone: float = 0.1) -> float:
        """
        Normalize joystick value with deadzone.
        
        Args:
            value: Raw joystick value (-1.0 to 1.0)
            deadzone: Deadzone threshold (0.0 to 1.0)
            
        Returns:
            Normalized value with deadzone applied
        """
        if abs(value) < deadzone:
            return 0.0
        
        # Scale beyond deadzone to full range
        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - deadzone) / (1.0 - deadzone)
        return sign * min(scaled, 1.0)
    
    def process_joystick_state(self, joystick_state: Dict) -> np.ndarray:
        """
        Convert joystick state to action vector.
        
        Expected joystick_state format:
        {
            'linear': {'x': float, 'y': float, 'z': float},
            'angular': {'x': float, 'y': float, 'z': float}
        }
        
        Args:
            joystick_state: Dictionary with linear and angular components
            
        Returns:
            Action vector matching action_spec segments
        """
        action = np.zeros(self.num_actions, dtype=np.float32)
        
        # Map linear velocity (typically from left joystick)
        if 'lin_vel' in self.segments:
            seg = self.segments['lin_vel']
            start = seg['start']
            
            linear = joystick_state.get('linear', {})
            action[start + 0] = self.normalize_joystick_value(linear.get('x', 0.0))
            action[start + 1] = self.normalize_joystick_value(linear.get('y', 0.0))
            if seg['size'] > 2:
                action[start + 2] = self.normalize_joystick_value(linear.get('z', 0.0))
        
        # Map angular velocity (typically from right joystick)
        if 'ang_vel' in self.segments:
            seg = self.segments['ang_vel']
            start = seg['start']
            
            angular = joystick_state.get('angular', {})
            action[start + 0] = self.normalize_joystick_value(angular.get('x', 0.0))
            action[start + 1] = self.normalize_joystick_value(angular.get('y', 0.0))
            if seg['size'] > 2:
                action[start + 2] = self.normalize_joystick_value(angular.get('z', 0.0))
        
        # Joint positions would be handled differently (e.g., from button combos)
        # For now, leave joint_pos segment as zeros (can be extended)
        
        return action
    
    def process_button_states(self, button_states: Dict) -> None:
        """
        Process button states (mainly for deadman switch).
        
        Expected button_states format:
        {
            'A': bool, 'B': bool, 'X': bool, 'Y': bool,
            'L1': bool, 'R1': bool, 'L2': float, 'R2': float,
            ...
        }
        
        Args:
            button_states: Dictionary of button names to states
        """
        # L1 button is the deadman switch
        self.deadman_active = button_states.get('L1', False)
        
        # Could add other button mappings here
        # e.g., mode switching, emergency stop, etc.
        
        self.last_button_states = button_states.copy()
    
    def apply_lowpass_filter(self, action: np.ndarray) -> np.ndarray:
        """
        Apply exponential moving average filter to smooth inputs.
        
        Args:
            action: Raw action vector
            
        Returns:
            Filtered action vector
        """
        if self.filtered_action is None:
            self.filtered_action = action.copy()
            return action.copy()
        
        # Exponential moving average: 
        # filtered = alpha * new + (1-alpha) * old
        self.filtered_action = (
            self.filter_alpha * action + 
            (1.0 - self.filter_alpha) * self.filtered_action
        )
        
        return self.filtered_action.copy()
    
    def get_action(
        self,
        joystick_state: Optional[Dict] = None,
        button_states: Optional[Dict] = None,
        current_time: Optional[float] = None
    ) -> np.ndarray:
        """
        Get normalized action from current gamepad state.
        
        Args:
            joystick_state: Current joystick state dict
            button_states: Current button states dict
            current_time: Current time in seconds (for rate limiting)
            
        Returns:
            Normalized action vector
        """
        # Update button states first (for deadman check)
        if button_states is not None:
            self.process_button_states(button_states)
        
        # Rate limiting check
        if current_time is not None:
            if current_time - self.last_update_time < self.min_update_interval:
                # Return last filtered action if updating too fast
                return self.filtered_action if self.filtered_action is not None else np.zeros(self.num_actions)
            self.last_update_time = current_time
        
        # Process joystick state
        if joystick_state is not None:
            raw_action = self.process_joystick_state(joystick_state)
            self.last_joystick_state = joystick_state.copy()
        elif self.last_joystick_state is not None:
            # Use last known state if no new input
            raw_action = self.process_joystick_state(self.last_joystick_state)
        else:
            # No input yet, return zeros
            raw_action = np.zeros(self.num_actions, dtype=np.float32)
        
        # Apply low-pass filter
        filtered_action = self.apply_lowpass_filter(raw_action)
        
        # Log to history
        self.input_history.append({
            'raw': raw_action.copy(),
            'filtered': filtered_action.copy(),
            'deadman': self.deadman_active
        })
        
        return filtered_action
    
    def is_deadman_active(self) -> bool:
        """Check if deadman switch is currently active."""
        return self.deadman_active
    
    def reset(self):
        """Reset bridge state."""
        self.last_joystick_state = None
        self.last_button_states = None
        self.deadman_active = False
        self.filtered_action = None
        self.last_update_time = 0.0
        self.input_history.clear()
    
    def get_diagnostics(self) -> Dict:
        """Get diagnostic info for debugging."""
        return {
            'deadman_active': self.deadman_active,
            'has_input': self.last_joystick_state is not None,
            'filtered_action_norm': (
                float(np.linalg.norm(self.filtered_action))
                if self.filtered_action is not None else 0.0
            ),
            'history_length': len(self.input_history)
        }
    
    def __repr__(self) -> str:
        deadman_str = "ON" if self.deadman_active else "OFF"
        has_input = self.last_joystick_state is not None
        return f"TeleopBridge(deadman={deadman_str}, input={'YES' if has_input else 'NO'})"
