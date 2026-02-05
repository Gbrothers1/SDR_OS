"""ActionRouter: Blends teleop and policy actions with safety constraints."""

from typing import Optional, Dict, Tuple
import numpy as np
import yaml
from pathlib import Path
from dataclasses import dataclass


@dataclass
class AlphaLimits:
    """Constraints on alpha (blend coefficient) evolution."""
    init: float = 0.0  # Initial value (start with pure teleop)
    max_ramp: float = 0.1  # Max increase per step
    min: float = 0.0
    max: float = 1.0


class ActionRouter:
    """
    Manages blending between teleop and policy actions with safety constraints.
    
    Blending formula: applied = alpha * policy + (1-alpha) * teleop
    
    Safety features:
    - Alpha ramp limiting (gradual increase only)
    - Instant drop to 0 on deadman release or override
    - Confidence gating (reduces alpha on low policy confidence)
    - Comprehensive logging of blend state
    """
    
    def __init__(self, config_path: Optional[str] = None):
        """
        Initialize ActionRouter from config file.
        
        Args:
            config_path: Path to YAML config. Defaults to rl/configs/default.yaml
        """
        if config_path is None:
            config_path = Path(__file__).parent.parent / "configs" / "default.yaml"
        
        self.config_path = Path(config_path)
        self._load_config()
        
        # Current blend state
        self._alpha = self.alpha_limits.init
        self._target_alpha = self.alpha_limits.init
        
        # History tracking for logging
        self.history = {
            'action_policy': None,
            'action_teleop': None,
            'blend_alpha': self._alpha,
            'applied_action': None,
            'actor_tag': 'teleop',  # 'teleop', 'policy', 'blend'
            'deadman_active': False,
            'override_triggered': False,
            'confidence': 1.0,
            'confidence_gate_active': False
        }
        
        # Confidence gating
        self.confidence_threshold = 0.5  # Below this, alpha is scaled down
        self.confidence_scale_min = 0.1  # Minimum scaling factor
    
    def _load_config(self):
        """Load alpha limits from config."""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        alpha_config = config['alpha']
        self.alpha_limits = AlphaLimits(
            init=alpha_config['init'],
            max_ramp=alpha_config['max_ramp'],
            min=alpha_config['min'],
            max=alpha_config['max']
        )
        
        # Store action dimension for validation
        self.num_actions = config['env']['num_actions']
    
    @property
    def alpha(self) -> float:
        """Current blend coefficient."""
        return self._alpha
    
    def set_target_alpha(self, target: float):
        """
        Set target alpha value (will ramp to it gradually).
        
        Args:
            target: Desired alpha value (0.0 to 1.0)
        """
        self._target_alpha = np.clip(target, self.alpha_limits.min, self.alpha_limits.max)
    
    def update_alpha(self):
        """
        Update alpha towards target with ramp limiting.
        
        Alpha can only increase gradually (max_ramp per step) but
        can decrease instantly for safety.
        """
        if self._target_alpha > self._alpha:
            # Gradual increase
            self._alpha = min(
                self._target_alpha,
                self._alpha + self.alpha_limits.max_ramp
            )
        else:
            # Instant decrease for safety
            self._alpha = self._target_alpha
        
        # Clamp to limits
        self._alpha = np.clip(self._alpha, self.alpha_limits.min, self.alpha_limits.max)
    
    def force_alpha_zero(self, reason: str = "override"):
        """
        Instantly drop alpha to 0 (safety override).
        
        Args:
            reason: Reason for override ('override', 'deadman', 'anomaly')
        """
        self._alpha = 0.0
        self._target_alpha = 0.0
        self.history['override_triggered'] = True
        self.history['actor_tag'] = f'override_{reason}'
    
    def apply_confidence_gate(self, confidence: float) -> float:
        """
        Apply confidence gating to alpha.
        
        If policy confidence is low, scale down alpha to reduce
        policy influence.
        
        Args:
            confidence: Policy confidence score (0.0 to 1.0)
            
        Returns:
            Effective alpha after confidence gating
        """
        self.history['confidence'] = confidence
        
        if confidence < self.confidence_threshold:
            # Scale alpha based on confidence
            scale = self.confidence_scale_min + (1.0 - self.confidence_scale_min) * (
                confidence / self.confidence_threshold
            )
            effective_alpha = self._alpha * scale
            self.history['confidence_gate_active'] = True
            return effective_alpha
        else:
            self.history['confidence_gate_active'] = False
            return self._alpha
    
    def blend(
        self,
        teleop_action: np.ndarray,
        policy_action: Optional[np.ndarray] = None,
        deadman_held: bool = True,
        confidence: Optional[float] = None
    ) -> Tuple[np.ndarray, Dict]:
        """
        Blend teleop and policy actions with safety checks.
        
        Args:
            teleop_action: Action from human teleop
            policy_action: Action from RL policy (None if no policy loaded)
            deadman_held: Whether deadman switch (e.g., L1) is held
            confidence: Optional policy confidence score
            
        Returns:
            (applied_action, log_dict) tuple
        """
        # Validate teleop action shape
        if teleop_action.shape[-1] != self.num_actions:
            raise ValueError(
                f"Teleop action shape {teleop_action.shape} doesn't match "
                f"expected {self.num_actions}"
            )
        
        # Update deadman state
        self.history['deadman_active'] = deadman_held
        
        # Deadman switch: if not held, force alpha to 0
        if not deadman_held:
            self.force_alpha_zero(reason="deadman")
        
        # If no policy, pure teleop
        if policy_action is None:
            self.history['action_teleop'] = teleop_action.copy()
            self.history['action_policy'] = None
            self.history['blend_alpha'] = 0.0
            self.history['applied_action'] = teleop_action.copy()
            self.history['actor_tag'] = 'teleop'
            return teleop_action.copy(), self.history.copy()
        
        # Validate policy action shape
        if policy_action.shape[-1] != self.num_actions:
            raise ValueError(
                f"Policy action shape {policy_action.shape} doesn't match "
                f"expected {self.num_actions}"
            )
        
        # Update alpha towards target
        self.update_alpha()
        
        # Apply confidence gating if provided
        effective_alpha = self._alpha
        if confidence is not None:
            effective_alpha = self.apply_confidence_gate(confidence)
        else:
            self.history['confidence'] = 1.0
            self.history['confidence_gate_active'] = False
        
        # Blend actions
        applied_action = (
            effective_alpha * policy_action + 
            (1.0 - effective_alpha) * teleop_action
        )
        
        # Update history
        self.history['action_teleop'] = teleop_action.copy()
        self.history['action_policy'] = policy_action.copy()
        self.history['blend_alpha'] = effective_alpha
        self.history['applied_action'] = applied_action.copy()
        
        # Determine actor tag (only if not already set by override)
        if not self.history['override_triggered']:
            if effective_alpha < 0.01:
                self.history['actor_tag'] = 'teleop'
            elif effective_alpha > 0.99:
                self.history['actor_tag'] = 'policy'
            else:
                self.history['actor_tag'] = 'blend'
            
            # Reset override flag (will be set again if needed)
            self.history['override_triggered'] = False
        
        return applied_action, self.history.copy()
    
    def reset(self):
        """Reset router state (call on environment reset)."""
        self._alpha = self.alpha_limits.init
        self._target_alpha = self.alpha_limits.init
        self.history = {
            'action_policy': None,
            'action_teleop': None,
            'blend_alpha': self._alpha,
            'applied_action': None,
            'actor_tag': 'teleop',
            'deadman_active': False,
            'override_triggered': False,
            'confidence': 1.0,
            'confidence_gate_active': False
        }
    
    def get_state(self) -> Dict:
        """Get current router state for logging."""
        return {
            'alpha': self._alpha,
            'target_alpha': self._target_alpha,
            'actor_tag': self.history['actor_tag'],
            'deadman_active': self.history['deadman_active'],
            'confidence': self.history['confidence'],
            'confidence_gate_active': self.history['confidence_gate_active']
        }
    
    def __repr__(self) -> str:
        return (
            f"ActionRouter(alpha={self._alpha:.3f}, "
            f"actor={self.history['actor_tag']}, "
            f"deadman={'ON' if self.history['deadman_active'] else 'OFF'})"
        )
