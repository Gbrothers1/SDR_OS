#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import argparse
import threading
import sys
import termios
import tty
import time

class WebcamPublisher(Node):
    def __init__(self, device_id=0, topic='/webcam/image_raw', frame_id='webcam_link', rate=15.0):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, topic, 10)
        self.timer_period = 1.0 / rate  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.device_id = device_id
        self.bridge = CvBridge()
        self.frame_id = frame_id
        self.cap = None
        self.switching = False  # Flag to indicate camera is switching
        self.available_devices = self.discover_cameras()
        if not self.available_devices:
            self.get_logger().error("No available camera devices found. Exiting.")
            rclpy.shutdown()
            return
        # Ensure initial device is in available list, otherwise choose first
        if self.device_id not in self.available_devices:
            self.device_id = self.available_devices[0]
        
        # Initialize camera
        if not self.setup_camera(self.device_id):
            self.get_logger().error("Failed to initialize camera")
            rclpy.shutdown()
            return
            
        self.get_logger().info(f"Publishing webcam feed from device {device_id} to {topic} at {rate} Hz")
        
        # Start key listener thread
        self.running = True
        self.key_thread = threading.Thread(target=self.key_listener)
        self.key_thread.daemon = True
        self.key_thread.start()

    def validate_camera(self, device_id):
        """Check if the camera is a valid capture device"""
        try:
            cap = cv2.VideoCapture(device_id)
            if not cap.isOpened():
                return False
                
            # Try to read a frame - this is the most reliable test
            ret, frame = cap.read()
            cap.release()
            
            return ret and frame is not None
            
        except Exception as e:
            self.get_logger().error(f"Error validating camera {device_id}: {str(e)}")
            return False

    def setup_camera(self, device_id):
        """Initialize or switch camera"""
        # Release existing camera if any
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            time.sleep(0.5)  # Increased delay to ensure proper cleanup
        
        try:
            # Try to open the camera
            self.cap = cv2.VideoCapture(device_id)
            if not self.cap.isOpened():
                self.get_logger().error(f"Could not open video device {device_id}")
                return False
            
            # Try to read a test frame
            ret, _ = self.cap.read()
            if not ret:
                self.get_logger().error(f"Could not read from video device {device_id}")
                self.cap.release()
                self.cap = None
                return False
            
            self.device_id = device_id
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error setting up camera {device_id}: {str(e)}")
            if self.cap is not None:
                self.cap.release()
                self.cap = None
            return False

    def get_key(self):
        """Get a single keypress from stdin"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def key_listener(self):
        """Listen for key presses in a separate thread"""
        self.get_logger().info("Press UP ARROW (escape sequence: ^[[A) to switch cameras")
        while self.running:
            key = self.get_key()
            if key == '\x1b':  # Escape sequence
                key = self.get_key()  # Get [
                if key == '[':
                    key = self.get_key()  # Get A (up arrow)
                    if key == 'A':  # Up arrow pressed
                        self.switch_camera()

    def discover_cameras(self, max_devices: int = 10):
        """Scan /dev/video* indices up to max_devices and return list of working device IDs."""
        detected = []
        self.get_logger().info(f"Scanning for camera devices (0..{max_devices-1}) ...")
        for dev_id in range(max_devices):
            cap = cv2.VideoCapture(dev_id)
            if cap.isOpened():
                ret, _ = cap.read()
                cap.release()
                if ret:
                    detected.append(dev_id)
                    self.get_logger().info(f"  Found camera at index {dev_id}")
        return detected

    def switch_camera(self):
        """Switch to the next detected camera in self.available_devices."""
        if len(self.available_devices) <= 1:
            self.get_logger().warn("Only one camera detected; cannot switch.")
            return
        current_idx = self.available_devices.index(self.device_id)
        next_idx = (current_idx + 1) % len(self.available_devices)
        next_device = self.available_devices[next_idx]

        self.get_logger().info(f"Attempting to switch from camera {self.device_id} to camera {next_device}")
        self.switching = True

        if self.setup_camera(next_device):
            self.get_logger().info(f"Successfully switched to camera {next_device}")
        else:
            self.get_logger().warn(f"Camera {next_device} became unavailable. Rescanning devices...")
            # Remove unavailable device and rescan
            if next_device in self.available_devices:
                self.available_devices.remove(next_device)
            self.available_devices.extend(self.discover_cameras())
            self.available_devices = sorted(list(set(self.available_devices)))
            if self.available_devices:
                self.device_id = self.available_devices[0]
                self.setup_camera(self.device_id)
                self.get_logger().info(f"Switched back to available camera {self.device_id}")
            else:
                self.get_logger().error("No cameras available after rescanning. Shutting down.")
                rclpy.shutdown()
        self.switching = False

    def timer_callback(self):
        if self.cap is None or self.switching:
            return
            
        try:
            ret, frame = self.cap.read()
            if ret:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = self.frame_id
                self.publisher_.publish(img_msg)
            else:
                self.get_logger().warn("Failed to capture frame from webcam")
                # Try to reinitialize the camera if we can't read from it
                if not self.setup_camera(self.device_id):
                    self.get_logger().error("Failed to reinitialize camera")
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")

    def destroy_node(self):
        self.running = False
        self.get_logger().info("Shutting down webcam publisher.")
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='Publish webcam feed to ROS.')
    parser.add_argument('--device-id', type=int, default=0, help='Webcam device ID (default: 0)')
    parser.add_argument('--topic', type=str, default='/webcam/image_raw', help='ROS topic to publish to (default: /webcam/image_raw)')
    parser.add_argument('--frame-id', type=str, default='webcam_link', help='Frame ID for the image message header (default: webcam_link)')
    parser.add_argument('--rate', type=float, default=15.0, help='Publishing rate in Hz (default: 15.0)')
    
    # Need to parse known args in case rclpy adds its own internal args
    parsed_args, unknown = parser.parse_known_args()

    webcam_publisher = WebcamPublisher(
        device_id=parsed_args.device_id,
        topic=parsed_args.topic,
        frame_id=parsed_args.frame_id,
        rate=parsed_args.rate
    )

    try:
        rclpy.spin(webcam_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        webcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 