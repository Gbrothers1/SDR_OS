#!/usr/bin/env python3

import sys
import rclpy
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from robot_controller.ui.main_window import MainWindow
from robot_controller.ros.ros_node import RobotControllerNode

def main():
    # Initialize ROS2
    rclpy.init()
    
    # Create Qt Application
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # Use Fusion style for better Steam Deck compatibility
    
    # Create ROS2 node
    ros_node = RobotControllerNode()
    
    # Create main window
    window = MainWindow(ros_node)
    window.show()
    
    # Start ROS2 spin in a separate thread
    ros_node.start_spin()
    
    # Run Qt event loop
    exit_code = app.exec()
    
    # Cleanup
    ros_node.stop_spin()
    rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main() 