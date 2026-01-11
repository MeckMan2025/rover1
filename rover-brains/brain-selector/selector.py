#!/usr/bin/env python3
"""
Brain Selector - PyQt5 Touchscreen Interface for Rover Brain Selection
Optimized for 7" touchscreen (1024x600) with dark theme
"""

import sys
import os
import yaml
import subprocess
import logging
from pathlib import Path
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QWidget, QScrollArea, QFrame
)
from PyQt5.QtCore import Qt, QSize, pyqtSignal
from PyQt5.QtGui import QFont, QPalette, QColor

class BrainButton(QPushButton):
    """Custom brain selection button with hover effects"""
    
    brain_selected = pyqtSignal(dict)
    
    def __init__(self, brain_info, parent=None):
        super().__init__(parent)
        self.brain_info = brain_info
        self.setMinimumHeight(150)
        self.setMinimumWidth(800)
        
        # Setup button layout with name and description
        self.setup_layout()
        self.setup_style()
        
        # Connect click to selection
        self.clicked.connect(lambda: self.brain_selected.emit(self.brain_info))
    
    def setup_layout(self):
        """Setup button text with name and description"""
        name = self.brain_info.get('name', 'Unknown Brain')
        desc = self.brain_info.get('description', 'No description available')
        
        # Combine name and description for button text
        button_text = f"{name}\n{desc}"
        self.setText(button_text)
        
        # Set font
        font = QFont('Arial', 18, QFont.Bold)
        self.setFont(font)
        
    def setup_style(self):
        """Setup dark theme button styling"""
        self.setStyleSheet("""
            QPushButton {
                background-color: #2d2d2d;
                color: #ffffff;
                border: 2px solid #0078d4;
                border-radius: 10px;
                padding: 20px;
                text-align: center;
                margin: 10px;
            }
            QPushButton:hover {
                background-color: #3d3d3d;
                border-color: #106ebe;
            }
            QPushButton:pressed {
                background-color: #1e1e1e;
                border-color: #0078d4;
            }
        """)

class BrainSelector(QMainWindow):
    """Main brain selector application"""
    
    def __init__(self):
        super().__init__()
        self.brains_dir = Path("/home/andrewmeckley/rover-brains")
        self.brains = []
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('/tmp/brain-selector.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # Setup UI
        self.setup_window()
        self.setup_dark_theme()
        self.scan_brains()
        self.setup_ui()
        
    def setup_window(self):
        """Configure main window for fullscreen kiosk mode"""
        self.setWindowTitle("🤖 Rover Brain Selector")
        self.setFixedSize(1024, 600)  # Hosyond 7" touchscreen resolution
        
        # Fullscreen, no decorations
        self.setWindowFlags(Qt.FramelessWindowHint | Qt.WindowStaysOnTopHint)
        self.showFullScreen()
        
    def setup_dark_theme(self):
        """Apply dark theme styling"""
        # Set application palette for dark theme
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(30, 30, 30))          # #1e1e1e
        palette.setColor(QPalette.WindowText, QColor(255, 255, 255))   # #ffffff
        palette.setColor(QPalette.Base, QColor(45, 45, 45))            # #2d2d2d
        palette.setColor(QPalette.AlternateBase, QColor(61, 61, 61))   # #3d3d3d
        palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
        palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
        palette.setColor(QPalette.Text, QColor(255, 255, 255))
        palette.setColor(QPalette.Button, QColor(45, 45, 45))
        palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
        palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
        palette.setColor(QPalette.Link, QColor(0, 120, 212))           # #0078d4
        palette.setColor(QPalette.Highlight, QColor(0, 120, 212))
        palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
        
        self.setPalette(palette)
        
        # Apply global stylesheet
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1e1e1e;
            }
            QScrollArea {
                background-color: #1e1e1e;
                border: none;
            }
            QLabel {
                color: #ffffff;
                background-color: transparent;
            }
        """)
        
    def scan_brains(self):
        """Scan for available brains with error handling"""
        self.logger.info(f"Scanning for brains in {self.brains_dir}")
        self.brains = []
        
        if not self.brains_dir.exists():
            self.logger.warning(f"Brains directory not found: {self.brains_dir}")
            return
            
        for brain_dir in self.brains_dir.iterdir():
            if brain_dir.is_dir() and brain_dir.name != "brain-selector":
                manifest_path = brain_dir / "brain.yaml"
                
                if manifest_path.exists():
                    try:
                        with open(manifest_path, 'r') as f:
                            manifest = yaml.safe_load(f)
                            
                        # Validate required fields
                        if not manifest.get('start_command'):
                            self.logger.warning(f"Missing start_command in {manifest_path}")
                            continue
                            
                        brain_info = {
                            'name': manifest.get('name', brain_dir.name),
                            'description': manifest.get('description', 'No description'),
                            'start_command': manifest['start_command'],
                            'working_directory': manifest.get('working_directory', str(brain_dir)),
                            'path': brain_dir
                        }
                        
                        self.brains.append(brain_info)
                        self.logger.info(f"Found brain: {brain_info['name']}")
                        
                    except Exception as e:
                        self.logger.error(f"Error reading {manifest_path}: {e}")
                        continue
                else:
                    self.logger.debug(f"No brain.yaml found in {brain_dir}")
                    
        self.logger.info(f"Found {len(self.brains)} valid brains")
        
    def setup_ui(self):
        """Setup the main user interface"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(40, 40, 40, 40)
        main_layout.setSpacing(20)
        
        # Header
        self.setup_header(main_layout)
        
        # Brain selection area
        self.setup_brain_area(main_layout)
        
        # Shutdown button
        self.setup_shutdown_button(main_layout)
        
    def setup_header(self, layout):
        """Setup header with title"""
        header = QLabel("🤖 SELECT ROVER BRAIN")
        header.setAlignment(Qt.AlignCenter)
        header.setFont(QFont('Arial', 28, QFont.Bold))
        header.setStyleSheet("""
            QLabel {
                color: #ffffff;
                background-color: transparent;
                padding: 20px;
                margin-bottom: 20px;
            }
        """)
        layout.addWidget(header)
        
    def setup_brain_area(self, layout):
        """Setup scrollable brain selection area"""
        # Create scroll area for brain buttons
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll_area.setMinimumHeight(300)
        
        # Scroll area content
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(15)
        scroll_layout.setContentsMargins(20, 20, 20, 20)
        
        # Add brain buttons
        if self.brains:
            for brain in self.brains:
                brain_button = BrainButton(brain)
                brain_button.brain_selected.connect(self.select_brain)
                scroll_layout.addWidget(brain_button)
        else:
            # No brains found message
            no_brains_label = QLabel("No brains found!\nCheck /home/andrewmeckley/rover-brains/")
            no_brains_label.setAlignment(Qt.AlignCenter)
            no_brains_label.setFont(QFont('Arial', 18))
            no_brains_label.setStyleSheet("""
                QLabel {
                    color: #ff6b6b;
                    background-color: #2d2d2d;
                    border: 2px solid #ff6b6b;
                    border-radius: 10px;
                    padding: 40px;
                    margin: 20px;
                }
            """)
            scroll_layout.addWidget(no_brains_label)
            
        scroll_layout.addStretch()
        scroll_area.setWidget(scroll_content)
        layout.addWidget(scroll_area)
        
    def setup_shutdown_button(self, layout):
        """Setup shutdown button at bottom"""
        shutdown_button = QPushButton("Shutdown")
        shutdown_button.setMinimumHeight(80)
        shutdown_button.setMinimumWidth(200)
        shutdown_button.setFont(QFont('Arial', 18, QFont.Bold))
        shutdown_button.setStyleSheet("""
            QPushButton {
                background-color: #d32f2f;
                color: #ffffff;
                border: 2px solid #b71c1c;
                border-radius: 10px;
                padding: 15px;
                margin: 20px;
            }
            QPushButton:hover {
                background-color: #f44336;
                border-color: #d32f2f;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)
        
        shutdown_button.clicked.connect(self.shutdown_system)
        
        # Center the shutdown button
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(shutdown_button)
        button_layout.addStretch()
        
        layout.addLayout(button_layout)
        
    def select_brain(self, brain_info):
        """Handle brain selection and launch"""
        self.logger.info(f"Selected brain: {brain_info['name']}")
        
        try:
            # Change working directory
            working_dir = brain_info['working_directory']
            self.logger.info(f"Changing to directory: {working_dir}")
            
            # Close the GUI cleanly
            self.hide()
            QApplication.processEvents()
            
            # Execute the brain's start command
            start_cmd = brain_info['start_command']
            self.logger.info(f"Executing: {start_cmd} in {working_dir}")
            
            # Use os.execvp to completely replace this process
            os.chdir(working_dir)
            
            # Parse command for execvp
            if start_cmd.startswith('./'):
                # Shell script
                os.execvp('bash', ['bash', start_cmd])
            else:
                # Split command and arguments
                cmd_parts = start_cmd.split()
                os.execvp(cmd_parts[0], cmd_parts)
                
        except Exception as e:
            self.logger.error(f"Failed to launch brain {brain_info['name']}: {e}")
            # Show error and stay in selector
            self.show()
            
    def shutdown_system(self):
        """Shutdown the system"""
        self.logger.info("Shutdown requested")
        try:
            subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=True)
        except Exception as e:
            self.logger.error(f"Shutdown failed: {e}")
            
    def keyPressEvent(self, event):
        """Handle keyboard events for emergency exit via SSH"""
        # Ctrl+C for emergency exit (useful for SSH debugging)
        if event.key() == Qt.Key_C and event.modifiers() == Qt.ControlModifier:
            self.logger.info("Emergency exit via Ctrl+C")
            self.close()
        super().keyPressEvent(event)

def main():
    """Main application entry point"""
    app = QApplication(sys.argv)
    app.setApplicationName("Brain Selector")
    
    # Create and show the brain selector
    selector = BrainSelector()
    
    # Run the application
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()