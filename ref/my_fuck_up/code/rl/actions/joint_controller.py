"""
Joint Controller for Robotic Manipulators

Provides direct joint-level control from gamepad input.
Supports multiple joint control modes and configurable mappings.
"""

import numpy as np
from typing import Dict, List, Optional
import logging

logger = logging.getLogger(__name__)


class JointController:
    """
    Direct joint control from gamepad.
    
    Control Mapping (for 7-DOF arm like Franka):
    - Left Stick X: Joint 1 (base rotation)
    - Left Stick Y: Joint 2 (shoulder)
    - Right Stick X: Joint 3 (elbow)
    - Right Stick Y: Joint 4 (forearm)
    - D-Pad Up/Down: Joint 5 (wrist 1)
    - D-Pad Left/Right: Joint 6 (wrist 2)
    - L2/R2: Joint 7 (wrist 3)
    - L1: Deadman switch
    - R1: Gripper toggle
    """
    
    def __init__(
        self,
        num_joints: int,
        joint_names: Optional[List[str]] = None,
        velocity_scale: float = 1.0,  # rad/s
        deadzone: float = 0.1,
        custom_mapping: Optional[Dict] = None
    ):
        """
        Initialize joint controller.
        
        Args:
            num_joints: Number of controlled joints (excluding gripper)
            joint_names: Optional list of joint names
            velocity_scale: Max joint velocity (rad/s)
            deadzone: Joystick deadzone threshold (0-1)
            custom_mapping: Custom gamepad-to-joint mapping
        """
        self.num_joints = num_joints
        self.joint_names = joint_names or [f"joint_{i}" for i in range(num_joints)]
        self.velocity_scale = velocity_scale
        self.deadzone = deadzone
        
        # Create default mapping
        if custom_mapping is None:
            self.joint_mapping = self._create_default_mapping()
        else:
            self.joint_mapping = custom_mapping
        
        # Gripper state
        self.gripper_state = 0.04  # Open (40mm for Franka)
        self.gripper_target = 0.04
        self.gripper_closed = False
        
        # Mode state
        self.control_mode = "normal"  # normal, fine, fast
        self.mode_scale = 1.0
        
        logger.info(f"Initialized joint controller for {num_joints} joints")
        logger.info(f"Joint names: {self.joint_names}")
    
    def _create_default_mapping(self) -> Dict:
        """
        Create default gamepad-to-joint mapping.
        
        Returns:
            Mapping dictionary
        """
        # Default mapping for 7-DOF arm
        mapping = {
            # Primary axes (sticks)
            'left_stick_x': {'joints': [0], 'scale': 1.0},
            'left_stick_y': {'joints': [1], 'scale': 1.0},
            'right_stick_x': {'joints': [2], 'scale': 1.0},
            'right_stick_y': {'joints': [3], 'scale': 1.0},
            
            # D-pad (discrete buttons)
            'dpad_up': {'joints': [4], 'value': 0.5},
            'dpad_down': {'joints': [4], 'value': -0.5},
            'dpad_right': {'joints': [5], 'value': 0.5},
            'dpad_left': {'joints': [5], 'value': -0.5},
            
            # Triggers
            'L2': {'joints': [6], 'scale': -0.8},
            'R2': {'joints': [6], 'scale': 0.8},
        }
        
        # For robots with fewer joints, remove extra mappings
        valid_mapping = {}
        for key, config in mapping.items():
            # Check if joint indices are valid
            if all(j < self.num_joints for j in config['joints']):
                valid_mapping[key] = config
        
        return valid_mapping
    
    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def set_control_mode(self, mode: str):
        """
        Set control mode.
        
        Args:
            mode: Control mode ('normal', 'fine', 'fast')
        """
        if mode == 'normal':
            self.mode_scale = 1.0
        elif mode == 'fine':
            self.mode_scale = 0.2
        elif mode == 'fast':
            self.mode_scale = 2.0
        else:
            logger.warning(f"Unknown mode: {mode}, using normal")
            self.mode_scale = 1.0
        
        self.control_mode = mode
        logger.info(f"Control mode set to: {mode} (scale: {self.mode_scale})")
    
    def gamepad_to_joint_velocities(self, gamepad_state: Dict) -> np.ndarray:
        """
        Convert gamepad state to joint velocity commands.
        
        Args:
            gamepad_state: Dictionary with gamepad inputs
                {
                    'left_stick_x': float,  # -1 to 1
                    'left_stick_y': float,
                    'right_stick_x': float,
                    'right_stick_y': float,
                    'L2': float,  # 0 to 1
                    'R2': float,
                    'dpad_up': bool,
                    'dpad_down': bool,
                    'dpad_left': bool,
                    'dpad_right': bool,
                    'L1': bool (deadman),
                    'R1': bool (gripper toggle),
                    'triangle': bool (mode change),
                    'circle': bool (mode change)
                }
        
        Returns:
            Joint velocities array (rad/s)
        """
        # Initialize joint velocities
        joint_vels = np.zeros(self.num_joints)
        
        # Check deadman
        if not gamepad_state.get('L1', False):
            return joint_vels
        
        # Check for mode changes
        if gamepad_state.get('triangle', False):
            if not hasattr(self, '_triangle_pressed'):
                self._triangle_pressed = True
                self.set_control_mode('fine')
        else:
            self._triangle_pressed = False
        
        if gamepad_state.get('circle', False):
            if not hasattr(self, '_circle_pressed'):
                self._circle_pressed = True
                self.set_control_mode('fast')
        else:
            self._circle_pressed = False
        
        if gamepad_state.get('square', False):
            if not hasattr(self, '_square_pressed'):
                self._square_pressed = True
                self.set_control_mode('normal')
        else:
            self._square_pressed = False
        
        # Process each mapping
        for input_name, config in self.joint_mapping.items():
            joints = config['joints']
            
            # Get input value
            if input_name in ['dpad_up', 'dpad_down', 'dpad_left', 'dpad_right']:
                # Digital button
                if gamepad_state.get(input_name, False):
                    value = config.get('value', 0.5)
                else:
                    value = 0.0
            else:
                # Analog axis
                raw_value = gamepad_state.get(input_name, 0.0)
                value = self.apply_deadzone(raw_value)
                
                # Apply scale from config
                value *= config.get('scale', 1.0)
            
            # Apply to joints
            for joint_idx in joints:
                if joint_idx < self.num_joints:
                    joint_vels[joint_idx] += value * self.velocity_scale * self.mode_scale
        
        # Clip to velocity limits (safety)
        joint_vels = np.clip(joint_vels, -self.velocity_scale * 2.0, self.velocity_scale * 2.0)
        
        return joint_vels
    
    def gamepad_to_joint_positions(
        self,
        gamepad_state: Dict,
        current_positions: np.ndarray,
        dt: float = 0.01
    ) -> np.ndarray:
        """
        Convert gamepad to joint position commands (by integrating velocities).
        
        Args:
            gamepad_state: Gamepad state
            current_positions: Current joint positions
            dt: Time step for integration
        
        Returns:
            Target joint positions
        """
        velocities = self.gamepad_to_joint_velocities(gamepad_state)
        return current_positions + velocities * dt
    
    def gamepad_to_joint_efforts(
        self,
        gamepad_state: Dict,
        effort_scale: float = 10.0
    ) -> np.ndarray:
        """
        Convert gamepad to joint effort (torque) commands.
        
        This is a simplified mapping - in practice, you'd want
        force feedback or impedance control.
        
        Args:
            gamepad_state: Gamepad state
            effort_scale: Scale factor for efforts (Nm)
        
        Returns:
            Joint efforts array (Nm)
        """
        # Use velocities as effort direction
        effort_direction = self.gamepad_to_joint_velocities(gamepad_state)
        
        # Normalize and scale
        norms = np.abs(effort_direction)
        efforts = effort_direction * effort_scale
        
        return efforts
    
    def set_joint_mapping(self, mapping: Dict):
        """Set custom joint mapping."""
        self.joint_mapping = mapping
        logger.info("Updated joint mapping")
    
    def get_joint_mapping(self) -> Dict:
        """Get current joint mapping."""
        return self.joint_mapping
    
    def get_state(self) -> Dict:
        """Get controller state for logging."""
        return {
            'num_joints': self.num_joints,
            'joint_names': self.joint_names,
            'control_mode': self.control_mode,
            'mode_scale': self.mode_scale,
            'velocity_scale': self.velocity_scale,
            'gripper_state': self.gripper_state,
            'gripper_closed': self.gripper_closed
        }


class AdaptiveJointController(JointController):
    """
    Adaptive joint controller that automatically adjusts mapping
    based on robot configuration.
    """
    
    def __init__(
        self,
        robot_config: Dict,
        velocity_scale: float = 1.0,
        deadzone: float = 0.1
    ):
        """
        Initialize from robot configuration.
        
        Args:
            robot_config: Robot configuration dictionary from YAML
            velocity_scale: Max joint velocity
            deadzone: Joystick deadzone
        """
        # Extract joint info from config
        robot_info = robot_config.get('robot', {})
        joint_names = robot_info.get('joints', [])
        num_joints = len(joint_names)
        
        # Get velocity limits from config
        control_config = robot_info.get('control', {})
        velocity_limits = control_config.get('velocity_limits', {})
        
        # Create per-joint scales
        joint_scales = {}
        for joint_name in joint_names:
            if joint_name in velocity_limits:
                joint_scales[joint_name] = velocity_limits[joint_name]
        
        super().__init__(
            num_joints=num_joints,
            joint_names=joint_names,
            velocity_scale=velocity_scale,
            deadzone=deadzone
        )
        
        self.joint_scales = joint_scales
        logger.info(f"Created adaptive controller for {num_joints} joints")
    
    def gamepad_to_joint_velocities(self, gamepad_state: Dict) -> np.ndarray:
        """Override to apply per-joint velocity limits."""
        # Get base velocities
        joint_vels = super().gamepad_to_joint_velocities(gamepad_state)
        
        # Apply per-joint limits
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in self.joint_scales:
                max_vel = self.joint_scales[joint_name]
                joint_vels[i] = np.clip(joint_vels[i], -max_vel, max_vel)
        
        return joint_vels
