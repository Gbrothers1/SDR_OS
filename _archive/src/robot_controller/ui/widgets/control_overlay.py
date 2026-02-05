from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QPen, QFont

class ControlOverlayWidget(QWidget):
    def __init__(self, ros_node, parent=None):
        super().__init__(parent)
        self.ros_node = ros_node
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.control_state = {
            'linear_x': 0.0,
            'angular_z': 0.0,
        }
        
        # Create layout
        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        
        # Add control labels
        self.linear_label = QLabel("Linear Velocity: 0.0")
        self.angular_label = QLabel("Angular Velocity: 0.0")
        layout.addWidget(self.linear_label)
        layout.addWidget(self.angular_label)
        
        # Set up update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update)
        self.update_timer.start(50)  # 20Hz update rate
    
    def paintEvent(self, event):
        """Draw the control overlay"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Set up pen and font
        pen = QPen(QColor(255, 255, 255, 180))
        pen.setWidth(2)
        painter.setPen(pen)
        
        font = QFont("Arial", 12)
        painter.setFont(font)
        
        # Draw control indicators
        self.draw_control_indicators(painter)
    
    def draw_control_indicators(self, painter):
        """Draw visual indicators for controls"""
        # Draw left stick indicator
        center_x = 100
        center_y = 50
        radius = 40
        
        # Draw stick base
        painter.drawEllipse(center_x - radius, center_y - radius, 
                          radius * 2, radius * 2)
        
        # Draw stick position
        stick_x = center_x + self.control_state['linear_x'] * radius
        stick_y = center_y + self.control_state['angular_z'] * radius
        painter.drawEllipse(stick_x - 5, stick_y - 5, 10, 10)
        
        # Draw labels
        painter.drawText(center_x - 80, center_y - 60, "Left Stick")
        painter.drawText(center_x - 80, center_y + 60, 
                        f"X: {self.control_state['linear_x']:.2f}")
        painter.drawText(center_x + 20, center_y + 60,
                        f"Z: {self.control_state['angular_z']:.2f}")
    
    def update_state(self, state):
        """Update the control state"""
        self.control_state = state
        self.linear_label.setText(f"Linear Velocity: {state['linear_x']:.2f}")
        self.angular_label.setText(f"Angular Velocity: {state['angular_z']:.2f}")
        self.update() 