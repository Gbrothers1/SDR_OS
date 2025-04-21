#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import argparse

class WebcamPublisher(Node):
    def __init__(self, device_id=0, topic='/webcam/image_raw', frame_id='webcam_link', rate=15.0):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(Image, topic, 10)
        self.timer_period = 1.0 / rate  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device {device_id}")
            rclpy.shutdown() # Or raise an exception
            return
            
        self.bridge = CvBridge()
        self.frame_id = frame_id
        self.get_logger().info(f"Publishing webcam feed from device {device_id} to {topic} at {rate} Hz")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = self.frame_id
            self.publisher_.publish(img_msg)
        else:
            self.get_logger().warn("Failed to capture frame from webcam")

    def destroy_node(self):
        self.get_logger().info("Shutting down webcam publisher.")
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
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        webcam_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 