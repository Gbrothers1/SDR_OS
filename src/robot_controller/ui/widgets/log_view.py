from PyQt6.QtWidgets import QWidget, QVBoxLayout, QTextEdit, QPushButton, QHBoxLayout
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QTextCursor, QColor, QTextCharFormat

class LogViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
        
        # Set up text formats
        self.info_format = QTextCharFormat()
        self.info_format.setForeground(QColor(200, 200, 200))
        
        self.warn_format = QTextCharFormat()
        self.warn_format.setForeground(QColor(255, 255, 0))
        
        self.error_format = QTextCharFormat()
        self.error_format.setForeground(QColor(255, 0, 0))
    
    def init_ui(self):
        """Initialize the user interface"""
        layout = QVBoxLayout(self)
        
        # Create text edit for logs
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #ffffff;
                font-family: 'Consolas', monospace;
                font-size: 12px;
            }
        """)
        layout.addWidget(self.log_text)
        
        # Create button layout
        button_layout = QHBoxLayout()
        
        # Clear button
        clear_button = QPushButton("Clear")
        clear_button.clicked.connect(self.clear_logs)
        button_layout.addWidget(clear_button)
        
        # Save button
        save_button = QPushButton("Save")
        save_button.clicked.connect(self.save_logs)
        button_layout.addWidget(save_button)
        
        layout.addLayout(button_layout)
    
    def append_log(self, message, level="info"):
        """Append a log message with the specified level"""
        cursor = self.log_text.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.End)
        
        # Select format based on level
        format_map = {
            "info": self.info_format,
            "warn": self.warn_format,
            "error": self.error_format
        }
        cursor.setCharFormat(format_map.get(level, self.info_format))
        
        # Insert message
        cursor.insertText(f"{message}\n")
        
        # Scroll to bottom
        self.log_text.setTextCursor(cursor)
        self.log_text.ensureCursorVisible()
    
    def clear_logs(self):
        """Clear all logs"""
        self.log_text.clear()
    
    def save_logs(self):
        """Save logs to file"""
        from PyQt6.QtWidgets import QFileDialog
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Logs",
            "",
            "Text Files (*.txt);;All Files (*)"
        )
        
        if filename:
            with open(filename, 'w') as f:
                f.write(self.log_text.toPlainText()) 