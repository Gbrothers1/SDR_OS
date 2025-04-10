from PyQt6.QtWidgets import QOpenGLWidget
from PyQt6.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np

class RobotViewWidget(QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.joint_states = {}
        self.camera_distance = 5.0
        self.camera_rotation = [45.0, 45.0]  # [x, y] rotation angles
        
        # Enable mouse tracking for camera control
        self.setMouseTracking(True)
    
    def initializeGL(self):
        """Initialize OpenGL context"""
        glClearColor(0.2, 0.2, 0.2, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        
        # Set up light
        glLightfv(GL_LIGHT0, GL_POSITION, [1, 1, 1, 0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.2, 0.2, 0.2, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1])
    
    def resizeGL(self, w, h):
        """Handle window resize"""
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w/h, 0.1, 100.0)
    
    def paintGL(self):
        """Render the scene"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # Set up camera
        gluLookAt(
            self.camera_distance * np.sin(np.radians(self.camera_rotation[1])) * np.cos(np.radians(self.camera_rotation[0])),
            self.camera_distance * np.sin(np.radians(self.camera_rotation[0])),
            self.camera_distance * np.cos(np.radians(self.camera_rotation[1])) * np.cos(np.radians(self.camera_rotation[0])),
            0, 0, 0,
            0, 1, 0
        )
        
        # Draw robot model
        self.draw_robot()
    
    def draw_robot(self):
        """Draw the robot model"""
        # Draw base
        glPushMatrix()
        glColor3f(0.5, 0.5, 0.5)
        self.draw_cylinder(0.5, 0.2)
        glPopMatrix()
        
        # Draw joints based on joint states
        for joint_name, state in self.joint_states.items():
            glPushMatrix()
            # Apply joint transformation
            glTranslatef(state.position[0], state.position[1], state.position[2])
            glRotatef(state.position[3], 0, 1, 0)
            
            # Draw joint
            glColor3f(0.8, 0.8, 0.8)
            self.draw_sphere(0.1)
            
            # Draw link
            glColor3f(0.6, 0.6, 0.6)
            self.draw_cylinder(0.3, 0.05)
            
            glPopMatrix()
    
    def draw_cylinder(self, height, radius):
        """Draw a cylinder"""
        quad = gluNewQuadric()
        gluCylinder(quad, radius, radius, height, 32, 32)
        gluDeleteQuadric(quad)
    
    def draw_sphere(self, radius):
        """Draw a sphere"""
        quad = gluNewQuadric()
        gluSphere(quad, radius, 32, 32)
        gluDeleteQuadric(quad)
    
    def update_joint_states(self, joint_state):
        """Update joint states from ROS message"""
        for i, name in enumerate(joint_state.name):
            self.joint_states[name] = type('JointState', (), {
                'position': joint_state.position[i],
                'velocity': joint_state.velocity[i] if i < len(joint_state.velocity) else 0.0,
                'effort': joint_state.effort[i] if i < len(joint_state.effort) else 0.0
            })
        self.update()
    
    def mousePressEvent(self, event):
        """Handle mouse press events for camera control"""
        if event.button() == Qt.MouseButton.LeftButton:
            self.last_pos = event.pos()
    
    def mouseMoveEvent(self, event):
        """Handle mouse move events for camera control"""
        if hasattr(self, 'last_pos'):
            dx = event.x() - self.last_pos.x()
            dy = event.y() - self.last_pos.y()
            
            self.camera_rotation[0] += dy * 0.5
            self.camera_rotation[1] += dx * 0.5
            
            self.camera_rotation[0] = max(-89, min(89, self.camera_rotation[0]))
            
            self.last_pos = event.pos()
            self.update()
    
    def wheelEvent(self, event):
        """Handle mouse wheel events for zoom"""
        delta = event.angleDelta().y()
        self.camera_distance = max(2.0, min(10.0, self.camera_distance - delta * 0.001))
        self.update() 