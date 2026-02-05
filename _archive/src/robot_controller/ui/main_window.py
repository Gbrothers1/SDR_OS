from PyQt6.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QSplitter, QTextEdit)
from PyQt6.QtCore import Qt, QTimer
from .widgets.robot_view import RobotViewWidget
from .widgets.control_overlay import ControlOverlayWidget
from .widgets.log_view import LogViewWidget

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.init_ui()
        
        # Update timer for UI refresh
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui)
        self.update_timer.start(50)  # 20Hz update rate
    
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Steam Deck Robot Controller')
        self.setMinimumSize(1280, 800)  # Steam Deck resolution
        
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter for resizable panels
        splitter = QSplitter(Qt.Orientation.Horizontal)
        
        # Create 3D view with control overlay
        view_container = QWidget()
        view_layout = QVBoxLayout(view_container)
        view_layout.setContentsMargins(0, 0, 0, 0)
        
        self.robot_view = RobotViewWidget()
        self.control_overlay = ControlOverlayWidget(self.ros_node)
        
        view_layout.addWidget(self.robot_view)
        view_layout.addWidget(self.control_overlay)
        
        # Create right panel for logs and telemetry
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        
        self.log_view = LogViewWidget()
        right_layout.addWidget(self.log_view)
        
        # Add widgets to splitter
        splitter.addWidget(view_container)
        splitter.addWidget(right_panel)
        splitter.setStretchFactor(0, 2)  # 3D view takes more space
        splitter.setStretchFactor(1, 1)
        
        main_layout.addWidget(splitter)
    
    def update_ui(self):
        """Update UI elements with latest data"""
        # Update joint states
        try:
            while not self.ros_node.joint_state_queue.empty():
                joint_state = self.ros_node.joint_state_queue.get_nowait()
                self.robot_view.update_joint_states(joint_state)
        except queue.Empty:
            pass
        
        # Update log view
        try:
            while not self.ros_node.log_queue.empty():
                log_msg = self.ros_node.log_queue.get_nowait()
                self.log_view.append_log(log_msg)
        except queue.Empty:
            pass
        
        # Update control overlay
        self.control_overlay.update_state(self.ros_node.control_state) 