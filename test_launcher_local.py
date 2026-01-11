#!/usr/bin/env python3
import sys
sys.path.append('.')
from brain_launcher import BrainLauncher
from pathlib import Path

class TestBrainLauncher(BrainLauncher):
    def __init__(self):
        super().__init__()
        # Override brains directory for testing
        self.brains_dir = Path("./test_brains")
        # Disable fullscreen for testing
        self.root.attributes('-fullscreen', False)
        self.root.geometry("800x600")

if __name__ == "__main__":
    launcher = TestBrainLauncher()
    launcher.run()
