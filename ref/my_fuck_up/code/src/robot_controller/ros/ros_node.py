import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import threading
import queue
import json

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Subscribe to controller joystick and button states
        self.joystick_state_sub = self.create_subscription(
            String,
            '/controller/joystick_state',
            self.joystick_state_callback,
            10
        )
        
        self.button_state_sub = self.create_subscription(
            String,
            '/controller/button_states',
            self.button_state_callback,
            10
        )
        
        # Message queues for UI updates
        self.joint_state_queue = queue.Queue()
        self.log_queue = queue.Queue()
        
        # Control state
        self.control_state = {
            'linear_x': 0.0,
            'angular_z': 0.0,
            'joint_states': {},
        }
        
        self.get_logger().info('Robot Controller Node initialized')
    
    def joint_state_callback(self, msg):
        """Handle incoming joint state messages"""
        try:
            self.joint_state_queue.put(msg)
        except queue.Full:
            self.get_logger().warn('Joint state queue full, dropping message')
    
    def joystick_state_callback(self, msg):
        """Handle joystick state messages and convert to cmd_vel"""
        try:
            # Parse joystick state from JSON
            joystick_state = json.loads(msg.data)
            
            # Create cmd_vel message from joystick state
            twist_msg = Twist()
            
            # Map joystick axes to twist message
            if 'linear' in joystick_state:
                twist_msg.linear.x = joystick_state['linear'].get('x', 0.0)
                twist_msg.linear.y = joystick_state['linear'].get('y', 0.0)
                twist_msg.linear.z = joystick_state['linear'].get('z', 0.0)
            
            if 'angular' in joystick_state:
                twist_msg.angular.x = joystick_state['angular'].get('x', 0.0)
                twist_msg.angular.y = joystick_state['angular'].get('y', 0.0)
                twist_msg.angular.z = joystick_state['angular'].get('z', 0.0)
            
            # Publish to cmd_vel
            self.cmd_vel_pub.publish(twist_msg)
            
            # Update control state
            self.control_state['linear_x'] = twist_msg.linear.x
            self.control_state['angular_z'] = twist_msg.angular.z
                
        except Exception as e:
            self.get_logger().error(f'Error processing joystick state: {str(e)}')
    
    def button_state_callback(self, msg):
        """Handle button state messages"""
        try:
            # Parse button state from JSON
            button_states = json.loads(msg.data)
            
            # Create twist message for button mappings
            twist_msg = Twist()
            
            # Map button states to twist commands
            # D-pad
            if button_states.get('DpadUp', False):
                twist_msg.linear.x = 0.5
            elif button_states.get('DpadDown', False):
                twist_msg.linear.x = -0.5
                
            if button_states.get('DpadLeft', False):
                twist_msg.angular.z = 0.5
            elif button_states.get('DpadRight', False):
                twist_msg.angular.z = -0.5
            
            # A/B buttons for up/down
            if button_states.get('A', False):
                twist_msg.linear.z = 0.5
            elif button_states.get('B', False):
                twist_msg.linear.z = -0.5
                
            # X/Y buttons for lateral movement
            if button_states.get('X', False):
                twist_msg.linear.y = 0.5
            elif button_states.get('Y', False):
                twist_msg.linear.y = -0.5
                
            # L1/R1 for rotation
            if button_states.get('L1', False):
                twist_msg.angular.z = 0.5
            elif button_states.get('R1', False):
                twist_msg.angular.z = -0.5
            
            # Only publish if any value is non-zero
            if (abs(twist_msg.linear.x) > 0.01 or
                abs(twist_msg.linear.y) > 0.01 or
                abs(twist_msg.linear.z) > 0.01 or
                abs(twist_msg.angular.x) > 0.01 or
                abs(twist_msg.angular.y) > 0.01 or
                abs(twist_msg.angular.z) > 0.01):
                
                self.cmd_vel_pub.publish(twist_msg)
                
                # Update control state
                self.control_state['linear_x'] = twist_msg.linear.x
                self.control_state['angular_z'] = twist_msg.angular.z
            
        except Exception as e:
            self.get_logger().error(f'Error processing button state: {str(e)}')
    
    def publish_control(self, linear_x, angular_z):
        """Publish control commands to the robot"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)
        
        # Update control state
        self.control_state['linear_x'] = linear_x
        self.control_state['angular_z'] = angular_z
    
    def start_spin(self):
        """Start ROS2 spin in a separate thread"""
        self._spin_thread = threading.Thread(target=self._spin)
        self._spin_thread.daemon = True
        self._spin_thread.start()
    
    def _spin(self):
        """ROS2 spin method run in a separate thread"""
        try:
            rclpy.spin(self)
        except Exception as e:
            self.get_logger().error(f'Error in ROS spin thread: {str(e)}')
    
    def get_control_state(self):
        """Return the current control state"""
        return self.control_state
    
    def get_joint_states(self):
        """Get collected joint states from the queue"""
        states = []
        try:
            while not self.joint_state_queue.empty():
                states.append(self.joint_state_queue.get_nowait())
        except queue.Empty:
            pass
        return states
    
    def get_logs(self):
        """Get collected log messages from the queue"""
        logs = []
        try:
            while not self.log_queue.empty():
                logs.append(self.log_queue.get_nowait())
        except queue.Empty:
            pass
        return logs 