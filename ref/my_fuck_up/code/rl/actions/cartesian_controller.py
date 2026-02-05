"""
Cartesian Controller for Robotic Manipulators

Converts gamepad input to Cartesian (end-effector) velocity commands.
Supports:
- X-Y-Z translation control
- Roll-Pitch-Yaw rotation control
- Gripper/tool actuation
- Integration with IK services
"""

import numpy as np
from typing import Dict, Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class CartesianController:
    """
    Convert gamepad input to Cartesian end-effector commands.
    
    Control Mapping:
    - Left Stick X: Move left/right (Y axis in world frame)
    - Left Stick Y: Move forward/backward (X axis in world frame)
    - Right Stick X: Rotate around Z axis (yaw)
    - Right Stick Y: Move up/down (Z axis in world frame)
    - L2 Trigger: Rotate around X axis (roll negative)
    - R2 Trigger: Rotate around X axis (roll positive)
    - Triangle/Y: Rotate around Y axis (pitch positive)
    - X/A: Rotate around Y axis (pitch negative)
    - L1: Deadman switch (must hold to enable control)
    - R1: Gripper close/open toggle
    """
    
    def __init__(
        self,
        translation_scale: float = 0.1,  # m/s
        rotation_scale: float = 0.5,  # rad/s
        gripper_scale: float = 0.02,  # m/s for finger motion
        deadzone: float = 0.1
    ):
        """
        Initialize Cartesian controller.
        
        Args:
            translation_scale: Max translation velocity (m/s)
            rotation_scale: Max rotation velocity (rad/s)
            gripper_scale: Gripper velocity (m/s)
            deadzone: Joystick deadzone threshold (0-1)
        """
        self.translation_scale = translation_scale
        self.rotation_scale = rotation_scale
        self.gripper_scale = gripper_scale
        self.deadzone = deadzone
        
        # Current end-effector state
        self.current_ee_pose = None  # [x, y, z, qw, qx, qy, qz]
        self.current_ee_velocity = None  # [vx, vy, vz, wx, wy, wz]
        
        # Gripper state
        self.gripper_state = 0.04  # Open (0.04m = 40mm for Franka)
        self.gripper_target = 0.04
        self.gripper_closed = False
        
        # IK service client (if available)
        self.ik_client = None
    
    def set_ik_client(self, ik_client):
        """Set IK service client for inverse kinematics."""
        self.ik_client = ik_client
    
    def update_ee_state(self, pose: np.ndarray, velocity: Optional[np.ndarray] = None):
        """
        Update current end-effector state.
        
        Args:
            pose: End-effector pose [x, y, z, qw, qx, qy, qz]
            velocity: End-effector velocity [vx, vy, vz, wx, wy, wz]
        """
        self.current_ee_pose = pose
        if velocity is not None:
            self.current_ee_velocity = velocity
    
    def apply_deadzone(self, value: float) -> float:
        """Apply deadzone to joystick input."""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale remaining range to 0-1
        sign = 1.0 if value > 0 else -1.0
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def gamepad_to_cartesian(self, gamepad_state: Dict) -> Dict:
        """
        Map gamepad state to Cartesian velocity command.
        
        Args:
            gamepad_state: Dictionary with gamepad inputs
                {
                    'left_stick_x': float,  # -1 to 1
                    'left_stick_y': float,
                    'right_stick_x': float,
                    'right_stick_y': float,
                    'L2': float,  # 0 to 1
                    'R2': float,
                    'triangle': bool,
                    'X': bool,
                    'L1': bool (deadman),
                    'R1': bool (gripper toggle)
                }
        
        Returns:
            Dictionary with Cartesian command:
                {
                    'linear': [vx, vy, vz],  # m/s
                    'angular': [wx, wy, wz],  # rad/s
                    'gripper': float  # m/s (finger velocity)
                }
        """
        # Check deadman
        if not gamepad_state.get('L1', False):
            return {
                'linear': [0.0, 0.0, 0.0],
                'angular': [0.0, 0.0, 0.0],
                'gripper': 0.0
            }
        
        # Get joystick values with deadzone
        left_x = self.apply_deadzone(gamepad_state.get('left_stick_x', 0.0))
        left_y = self.apply_deadzone(gamepad_state.get('left_stick_y', 0.0))
        right_x = self.apply_deadzone(gamepad_state.get('right_stick_x', 0.0))
        right_y = self.apply_deadzone(gamepad_state.get('right_stick_y', 0.0))
        
        # Translation (world frame)
        # Note: Inverted Y for intuitive forward = push up
        vel_x = left_y * self.translation_scale  # Forward/backward
        vel_y = left_x * self.translation_scale  # Left/right
        vel_z = right_y * self.translation_scale  # Up/down (inverted for intuitive control)
        
        # Rotation
        # Yaw from right stick X
        ang_z = right_x * self.rotation_scale
        
        # Roll from triggers (L2 = negative, R2 = positive)
        l2 = gamepad_state.get('L2', 0.0)
        r2 = gamepad_state.get('R2', 0.0)
        ang_x = (r2 - l2) * self.rotation_scale
        
        # Pitch from buttons (Triangle = positive, X = negative)
        triangle = 1.0 if gamepad_state.get('triangle', False) else 0.0
        x_button = 1.0 if gamepad_state.get('X', False) else 0.0
        ang_y = (triangle - x_button) * self.rotation_scale * 0.5  # Slower for pitch
        
        # Gripper toggle
        if gamepad_state.get('R1', False):
            # Toggle gripper state
            if not hasattr(self, '_r1_pressed'):
                self._r1_pressed = True
                self.gripper_closed = not self.gripper_closed
                self.gripper_target = 0.0 if self.gripper_closed else 0.04
                logger.info(f"Gripper {'closed' if self.gripper_closed else 'opened'}")
        else:
            self._r1_pressed = False
        
        # Compute gripper velocity (move towards target)
        gripper_error = self.gripper_target - self.gripper_state
        gripper_vel = np.clip(gripper_error * 2.0, -self.gripper_scale, self.gripper_scale)
        
        return {
            'linear': [vel_x, vel_y, vel_z],
            'angular': [ang_x, ang_y, ang_z],
            'gripper': gripper_vel
        }
    
    async def cartesian_to_joint_velocities(
        self,
        cart_vel: Dict,
        current_joint_state: np.ndarray
    ) -> np.ndarray:
        """
        Convert Cartesian velocity to joint velocities using Jacobian.
        
        This requires either:
        1. IK service from genesis_ros
        2. Jacobian computation from robot model
        3. Simple integration and IK
        
        Args:
            cart_vel: Cartesian velocity command
            current_joint_state: Current joint positions
        
        Returns:
            Joint velocities (rad/s for revolute, m/s for prismatic)
        """
        # Extract velocities
        linear_vel = np.array(cart_vel['linear'])
        angular_vel = np.array(cart_vel['angular'])
        twist = np.concatenate([linear_vel, angular_vel])
        
        # Method 1: Use IK service (if available)
        if self.ik_client is not None:
            # Integrate twist to get target pose
            if self.current_ee_pose is not None:
                dt = 0.01  # Assume 100 Hz
                target_pose = self.integrate_twist(self.current_ee_pose, twist, dt)
                
                # Call IK service
                try:
                    joint_positions = await self.compute_ik_service(target_pose)
                    
                    # Compute joint velocities from position difference
                    joint_vel = (joint_positions - current_joint_state) / dt
                    return joint_vel
                except Exception as e:
                    logger.error(f"IK service error: {e}")
        
        # Method 2: Use Jacobian inverse (requires robot model)
        # This would require loading URDF and computing Jacobian
        # For now, return zero velocities as fallback
        logger.warning("Jacobian-based control not implemented, returning zero velocities")
        return np.zeros_like(current_joint_state)
    
    def integrate_twist(
        self,
        pose: np.ndarray,
        twist: np.ndarray,
        dt: float
    ) -> np.ndarray:
        """
        Integrate twist (velocity) to get new pose.
        
        Args:
            pose: Current pose [x, y, z, qw, qx, qy, qz]
            twist: Twist [vx, vy, vz, wx, wy, wz]
            dt: Time step
        
        Returns:
            New pose [x, y, z, qw, qx, qy, qz]
        """
        # Extract position and orientation
        pos = pose[:3]
        quat = pose[3:]  # [qw, qx, qy, qz]
        
        # Integrate position
        new_pos = pos + twist[:3] * dt
        
        # Integrate orientation (simplified - should use proper quaternion integration)
        # Convert angular velocity to quaternion derivative
        ang_vel = twist[3:]
        # For small dt, approximate: q_new ≈ q + 0.5 * dt * [0, wx, wy, wz] * q
        omega_quat = np.array([0, ang_vel[0], ang_vel[1], ang_vel[2]])
        q_dot = 0.5 * self.quaternion_multiply(omega_quat, quat)
        new_quat = quat + q_dot * dt
        
        # Normalize quaternion
        new_quat = new_quat / np.linalg.norm(new_quat)
        
        return np.concatenate([new_pos, new_quat])
    
    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Multiply two quaternions."""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])
    
    async def compute_ik_service(self, target_pose: np.ndarray) -> np.ndarray:
        """
        Call IK service to compute joint positions for target EE pose.
        
        Args:
            target_pose: Target end-effector pose [x, y, z, qw, qx, qy, qz]
        
        Returns:
            Joint positions
        """
        if self.ik_client is None:
            raise RuntimeError("IK client not set")
        
        # This would call the actual ROS2 service
        # For now, raise not implemented
        raise NotImplementedError("IK service call not implemented")
    
    def get_state(self) -> Dict:
        """Get controller state for logging."""
        return {
            'ee_pose': self.current_ee_pose.tolist() if self.current_ee_pose is not None else None,
            'ee_velocity': self.current_ee_velocity.tolist() if self.current_ee_velocity is not None else None,
            'gripper_state': self.gripper_state,
            'gripper_target': self.gripper_target,
            'gripper_closed': self.gripper_closed
        }
