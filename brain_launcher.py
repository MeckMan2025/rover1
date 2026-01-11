#!/usr/bin/env python3
"""
Rover Brain Selector - Touchscreen Boot Launcher
Allows selection of different rover "brains" at startup
"""

import os
import sys
import yaml
import subprocess
import tkinter as tk
from tkinter import ttk
from pathlib import Path
import logging

class BrainLauncher:
    def __init__(self):
        self.brains_dir = Path("/home/andrewmeckley/rover-brains")
        self.brains = []
        
        # Setup logging
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('/tmp/brain_launcher.log'),
                logging.StreamHandler()
            ]
        )
        self.logger = logging.getLogger(__name__)
        
        # Create main window
        self.root = tk.Tk()
        self.root.title("Rover Brain Selector")
        
        # Fullscreen for 7-inch touchscreen
        self.root.attributes('-fullscreen', True)
        self.root.configure(bg='#1a1a1a')
        
        # Bind escape key to exit (for development)
        self.root.bind('<Escape>', self.quit_app)
        
        self.setup_ui()
        self.scan_brains()
        
    def setup_ui(self):
        """Setup the touch-friendly UI"""
        # Header
        header_frame = tk.Frame(self.root, bg='#2c3e50', height=80)
        header_frame.pack(fill='x', pady=(0, 20))
        header_frame.pack_propagate(False)
        
        title_label = tk.Label(
            header_frame, 
            text="🚀 Rover Brain Selector",
            font=('Arial', 28, 'bold'),
            fg='white',
            bg='#2c3e50'
        )
        title_label.pack(expand=True)
        
        # Scrollable brain list container
        self.canvas = tk.Canvas(self.root, bg='#1a1a1a', highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=self.canvas.yview)
        self.scrollable_frame = tk.Frame(self.canvas, bg='#1a1a1a')
        
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        
        self.canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=self.scrollbar.set)
        
        self.canvas.pack(side="left", fill="both", expand=True, padx=20)
        self.scrollbar.pack(side="right", fill="y")
        
        # Touch scrolling support
        self.canvas.bind("<Button-1>", self.on_touch_start)
        self.canvas.bind("<B1-Motion>", self.on_touch_move)
        self.last_y = 0
        
        # Footer with exit button (for development)
        footer_frame = tk.Frame(self.root, bg='#34495e', height=60)
        footer_frame.pack(fill='x', side='bottom')
        footer_frame.pack_propagate(False)
        
        exit_btn = tk.Button(
            footer_frame,
            text="Exit (Dev Mode)",
            font=('Arial', 14),
            command=self.quit_app,
            bg='#e74c3c',
            fg='white',
            relief='flat',
            padx=20
        )
        exit_btn.pack(side='right', padx=20, pady=10)
        
    def on_touch_start(self, event):
        """Handle touch start for scrolling"""
        self.last_y = event.y
        
    def on_touch_move(self, event):
        """Handle touch move for scrolling"""
        delta = self.last_y - event.y
        self.canvas.yview_scroll(int(delta/30), "units")
        self.last_y = event.y
        
    def scan_brains(self):
        """Scan for available brains in the brains directory"""
        self.logger.info(f"Scanning for brains in {self.brains_dir}")
        
        if not self.brains_dir.exists():
            self.logger.warning(f"Brains directory does not exist: {self.brains_dir}")
            self.show_error("Brains directory not found!")
            return
            
        self.brains = []
        
        for brain_dir in self.brains_dir.iterdir():
            if brain_dir.is_dir():
                manifest_path = brain_dir / "brain.yaml"
                if manifest_path.exists():
                    try:
                        with open(manifest_path, 'r') as f:
                            manifest = yaml.safe_load(f)
                            
                        brain_info = {
                            'path': brain_dir,
                            'name': manifest.get('name', brain_dir.name),
                            'description': manifest.get('description', 'No description'),
                            'start_command': manifest.get('start_command', '')
                        }
                        self.brains.append(brain_info)
                        self.logger.info(f"Found brain: {brain_info['name']}")
                        
                    except Exception as e:
                        self.logger.error(f"Error reading manifest {manifest_path}: {e}")
                        
        if not self.brains:
            self.show_error("No brains found!")
        else:
            self.populate_brain_list()
            
    def populate_brain_list(self):
        """Populate the UI with available brains"""
        for i, brain in enumerate(self.brains):
            # Create brain selection button
            brain_frame = tk.Frame(
                self.scrollable_frame, 
                bg='#34495e', 
                relief='raised', 
                bd=2
            )
            brain_frame.pack(fill='x', pady=10, padx=20, ipady=20)
            
            # Make the entire frame clickable
            brain_frame.bind("<Button-1>", lambda e, b=brain: self.select_brain(b))
            
            # Brain name
            name_label = tk.Label(
                brain_frame,
                text=brain['name'],
                font=('Arial', 24, 'bold'),
                fg='#ecf0f1',
                bg='#34495e'
            )
            name_label.pack(anchor='w', padx=20, pady=(10, 5))
            name_label.bind("<Button-1>", lambda e, b=brain: self.select_brain(b))
            
            # Brain description
            desc_label = tk.Label(
                brain_frame,
                text=brain['description'],
                font=('Arial', 16),
                fg='#bdc3c7',
                bg='#34495e',
                wraplength=600,
                justify='left'
            )
            desc_label.pack(anchor='w', padx=20, pady=(0, 10))
            desc_label.bind("<Button-1>", lambda e, b=brain: self.select_brain(b))
            
    def select_brain(self, brain):
        """Handle brain selection"""
        self.logger.info(f"Selected brain: {brain['name']}")
        
        if not brain['start_command']:
            self.show_error(f"No start command configured for {brain['name']}")
            return
            
        try:
            # Change to brain directory
            os.chdir(brain['path'])
            
            # Hide the launcher
            self.root.withdraw()
            
            # Execute the start command
            self.logger.info(f"Executing: {brain['start_command']}")
            result = subprocess.run(
                brain['start_command'],
                shell=True,
                cwd=brain['path']
            )
            
            self.logger.info(f"Brain {brain['name']} exited with code: {result.returncode}")
            
        except Exception as e:
            self.logger.error(f"Error launching brain {brain['name']}: {e}")
            self.show_error(f"Failed to launch {brain['name']}: {str(e)}")
            self.root.deiconify()  # Show launcher again
            
    def show_error(self, message):
        """Display error message"""
        error_window = tk.Toplevel(self.root)
        error_window.title("Error")
        error_window.configure(bg='#e74c3c')
        error_window.geometry("400x200")
        
        # Center the error window
        error_window.transient(self.root)
        error_window.grab_set()
        
        tk.Label(
            error_window,
            text=message,
            font=('Arial', 16),
            fg='white',
            bg='#e74c3c',
            wraplength=350
        ).pack(expand=True, padx=20, pady=20)
        
        tk.Button(
            error_window,
            text="OK",
            font=('Arial', 14),
            command=error_window.destroy,
            bg='#c0392b',
            fg='white',
            relief='flat',
            padx=30,
            pady=10
        ).pack(pady=10)
        
    def quit_app(self, event=None):
        """Quit the application"""
        self.logger.info("Exiting brain launcher")
        self.root.quit()
        
    def run(self):
        """Start the launcher"""
        self.logger.info("Starting brain launcher")
        self.root.mainloop()

if __name__ == "__main__":
    launcher = BrainLauncher()
    launcher.run()