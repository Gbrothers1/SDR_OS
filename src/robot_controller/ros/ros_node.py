import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import threading
import queue

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
    
    def stop_spin(self):
        """Stop ROS2 spin thread"""
        if hasattr(self, '_spin_thread'):
            self._spin_thread.join(timeout=1.0)
    
    def _spin(self):
        """ROS2 spin loop"""
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1) 