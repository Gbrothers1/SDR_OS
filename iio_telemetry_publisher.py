#!/usr/bin/env python3
"""
IIO Telemetry Publisher for Robot Controller

This script reads data from available IIO sensors (accelerometer, gyroscope, etc.)
and publishes the data to ROS topics for visualization in the Robot Controller app.

Usage:
    python3 iio_telemetry_publisher.py [--rate RATE]

Arguments:
    --rate RATE    Publishing rate in Hz (default: 10)
"""

import os
import glob
import re
import time
import argparse
import math
import json
import threading
from typing import Dict, List, Any, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Vector3, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

# Hardcoded descriptions for known device types
DEVICE_DESCRIPTIONS = {
    "gyro_3d": "Gyroscope - Measures angular velocity in 3 axes.",
    "accel_3d": "Accelerometer - Measures acceleration (including gravity) in 3 axes.",
    "magn_3d": "Magnetometer - Measures magnetic field strength in 3 axes.",
    "als": "Ambient Light Sensor - Measures ambient light intensity.",
    "incli_3d": "Inclinometer - Measures tilt (pitch and roll) relative to gravity.",
    "dev_rotation": "Rotation Sensor - Provides overall rotation data, fusing sensor inputs."
}

class IIOPublisherNode(Node):
    """ROS2 node for publishing IIO sensor data."""

    def __init__(self, rate=10):
        super().__init__('iio_telemetry_publisher')
        
        # Initialize parameters
        self.publish_rate = rate
        self.get_logger().info(f'Starting IIO Telemetry Publisher with rate: {rate} Hz')
        
        # Initialize device data storage
        self.devices = {}
        self.imu_data = {
            'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'mag': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }
        
        # Create publishers
        self.telemetry_publisher = self.create_publisher(
            String, 
            '/robot/telemetry/all', 
            10
        )
        
        self.imu_publisher = self.create_publisher(
            Imu,
            '/robot/imu',
            10
        )
        
        # Create transform broadcaster for visualizing orientation
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer for regular publishing
        self.timer = self.create_timer(1.0/rate, self.publish_data)
        
        # Start data collection thread
        self.running = True
        self.data_thread = threading.Thread(target=self.collect_data)
        self.data_thread.daemon = True
        self.data_thread.start()
        
        self.get_logger().info('IIO Telemetry Publisher is running')

    def collect_data(self):
        """Continuously collect data from IIO devices."""
        while self.running:
            try:
                device_dirs = sorted(glob.glob("/sys/bus/iio/devices/iio:device*"))
                
                if not device_dirs:
                    self.get_logger().warn_once("No IIO devices found")
                    time.sleep(1.0)
                    continue
                    
                for device_dir in device_dirs:
                    device_data = self.get_device_data(device_dir)
                    self.devices[device_data['name']] = device_data
                    
                # Update IMU data from relevant sensors
                self.update_imu_data()
                
                # Sleep to avoid excessive CPU usage
                time.sleep(0.01)
                
            except Exception as e:
                self.get_logger().error(f'Error collecting data: {str(e)}')
                time.sleep(1.0)

    def get_device_name(self, device_dir):
        """Get the name of an IIO device from its directory."""
        name_file = os.path.join(device_dir, "name")
        if os.path.exists(name_file):
            try:
                return open(name_file).read().strip()
            except Exception as e:
                return f"Error: {e}"
        return "Unknown"

    def read_value(self, file_path):
        """Read a value from a file, returning None if the file can't be read."""
        try:
            return open(file_path).read().strip()
        except Exception:
            return None

    def get_scale_for_channel(self, device_dir, raw_file):
        """Get the scale factor for a raw IIO channel."""
        candidate = raw_file.replace("_raw", "_scale")
        scale_path = os.path.join(device_dir, candidate)
        if os.path.exists(scale_path):
            return self.read_value(scale_path)
            
        # For names like in_accel_x_raw, try in_accel_scale
        m = re.match(r"(in_[a-zA-Z]+)_\w+_raw", raw_file)
        if m:
            candidate = m.group(1) + "_scale"
            scale_path = os.path.join(device_dir, candidate)
            if os.path.exists(scale_path):
                return self.read_value(scale_path)
                
        return None

    def get_device_data(self, device_dir):
        """Read data from an IIO device."""
        name = self.get_device_name(device_dir)
        description = DEVICE_DESCRIPTIONS.get(name, "No description available for this device.")
        values = {}
        
        for filename in sorted(os.listdir(device_dir)):
            if filename.endswith("_raw"):
                file_path = os.path.join(device_dir, filename)
                raw_value_str = self.read_value(file_path)
                
                # If the raw value is missing or empty, mark it as such
                if raw_value_str is None or raw_value_str == "":
                    values[filename] = {"raw": "Missing"}
                    continue
                    
                try:
                    raw_value = float(raw_value_str)
                    can_scale = True
                except ValueError:
                    raw_value = raw_value_str
                    can_scale = False
                    
                scale_str = self.get_scale_for_channel(device_dir, filename)
                if can_scale and scale_str is not None:
                    try:
                        scale = float(scale_str)
                        scaled_value = raw_value * scale
                    except ValueError:
                        scaled_value = "Error: invalid scale"
                    values[filename] = {"raw": raw_value, "scaled": scaled_value}
                else:
                    values[filename] = {"raw": raw_value}
                    
        return {
            "path": device_dir,
            "name": name,
            "description": description,
            "values": values
        }

    def update_imu_data(self):
        """Update IMU data from the available sensors."""
        # Process accelerometer data
        if 'accel_3d' in self.devices:
            accel_data = self.devices['accel_3d']['values']
            for axis in ['x', 'y', 'z']:
                key = f'in_accel_{axis}_raw'
                if key in accel_data and 'scaled' in accel_data[key]:
                    self.imu_data['accel'][axis] = accel_data[key]['scaled']
        
        # Process gyroscope data
        if 'gyro_3d' in self.devices:
            gyro_data = self.devices['gyro_3d']['values']
            for axis in ['x', 'y', 'z']:
                key = f'in_anglvel_{axis}_raw'
                if key in gyro_data and 'scaled' in gyro_data[key]:
                    self.imu_data['gyro'][axis] = gyro_data[key]['scaled']
        
        # Process magnetometer data
        if 'magn_3d' in self.devices:
            mag_data = self.devices['magn_3d']['values']
            for axis in ['x', 'y', 'z']:
                key = f'in_magn_{axis}_raw'
                if key in mag_data and 'scaled' in mag_data[key]:
                    self.imu_data['mag'][axis] = mag_data[key]['scaled']
        
        # Calculate orientation if we have accelerometer data
        if 'accel_3d' in self.devices:
            # Simple orientation calculation based on accelerometer
            ax = self.imu_data['accel']['x']
            ay = self.imu_data['accel']['y']
            az = self.imu_data['accel']['z']
            
            # Calculate roll and pitch from accelerometer
            # This is a simplified calculation for demo purposes
            roll = math.atan2(ay, az)
            pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
            
            # Yaw cannot be determined from accelerometer alone
            # In a real implementation, we would use a sensor fusion algorithm with gyro and mag
            yaw = 0.0
            
            self.imu_data['orientation']['roll'] = roll
            self.imu_data['orientation']['pitch'] = pitch
            self.imu_data['orientation']['yaw'] = yaw

    def publish_data(self):
        """Publish the collected sensor data to ROS topics."""
        try:
            # Publish all telemetry data as JSON
            telemetry_msg = String()
            telemetry_msg.data = json.dumps({
                'timestamp': self.get_clock().now().to_msg().sec,
                'devices': self.devices,
                'imu': self.imu_data
            })
            self.telemetry_publisher.publish(telemetry_msg)
            
            # Publish IMU data
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"
            
            # Linear acceleration (from accelerometer)
            imu_msg.linear_acceleration.x = self.imu_data['accel']['x']
            imu_msg.linear_acceleration.y = self.imu_data['accel']['y']
            imu_msg.linear_acceleration.z = self.imu_data['accel']['z']
            
            # Angular velocity (from gyroscope)
            imu_msg.angular_velocity.x = self.imu_data['gyro']['x']
            imu_msg.angular_velocity.y = self.imu_data['gyro']['y']
            imu_msg.angular_velocity.z = self.imu_data['gyro']['z']
            
            # Orientation as quaternion (simplified conversion)
            roll = self.imu_data['orientation']['roll']
            pitch = self.imu_data['orientation']['pitch']
            yaw = self.imu_data['orientation']['yaw']
            
            # Convert Euler angles to quaternion (simplified)
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            
            imu_msg.orientation.w = cy * cp * cr + sy * sp * sr
            imu_msg.orientation.x = cy * cp * sr - sy * sp * cr
            imu_msg.orientation.y = sy * cp * sr + cy * sp * cr
            imu_msg.orientation.z = sy * cp * cr - cy * sp * sr
            
            # Set covariance matrices (simplified - using identity)
            for i in range(9):
                imu_msg.orientation_covariance[i] = 0.01 if i % 4 == 0 else 0.0
                imu_msg.angular_velocity_covariance[i] = 0.01 if i % 4 == 0 else 0.0
                imu_msg.linear_acceleration_covariance[i] = 0.01 if i % 4 == 0 else 0.0
            
            self.imu_publisher.publish(imu_msg)
            
            # Publish transform for visualization
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "base_link"
            tf.child_frame_id = "imu_link"
            
            # Position (typically fixed relative to base)
            tf.transform.translation.x = 0.0
            tf.transform.translation.y = 0.0
            tf.transform.translation.z = 0.1  # 10cm above base
            
            # Orientation (same as IMU message)
            tf.transform.rotation.w = imu_msg.orientation.w
            tf.transform.rotation.x = imu_msg.orientation.x
            tf.transform.rotation.y = imu_msg.orientation.y
            tf.transform.rotation.z = imu_msg.orientation.z
            
            self.tf_broadcaster.sendTransform(tf)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing data: {str(e)}')

    def __del__(self):
        """Clean up resources when the node is destroyed."""
        self.running = False
        if hasattr(self, 'data_thread') and self.data_thread.is_alive():
            self.data_thread.join(timeout=1.0)

def main():
    """Main entry point for the script."""
    parser = argparse.ArgumentParser(description='IIO Telemetry Publisher for Robot Controller')
    parser.add_argument('--rate', type=float, default=10.0,
                        help='Publishing rate in Hz (default: 10)')
    args = parser.parse_args()
    
    rclpy.init()
    node = IIOPublisherNode(rate=args.rate)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 