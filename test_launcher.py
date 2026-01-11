#!/usr/bin/env python3
"""
Test script for the Brain Launcher
Creates sample brain directories and manifests for testing
"""

import os
import yaml
from pathlib import Path

def create_test_brains():
    """Create test brain directories and manifests"""
    
    # Test brains directory
    test_dir = Path("./test_brains")
    test_dir.mkdir(exist_ok=True)
    
    # Sample brains to create
    brains = [
        {
            "name": "rover1",
            "display_name": "Rover1 System Monitor",
            "description": "System monitoring and basic rover operations using btop",
            "start_command": "btop"
        },
        {
            "name": "test-brain",
            "display_name": "Test Brain",
            "description": "A simple test brain that displays a message",
            "start_command": "echo 'Test brain activated!' && sleep 3"
        },
        {
            "name": "demo-brain",
            "display_name": "Demo Navigation",
            "description": "Demonstrates rover navigation capabilities with mock data",
            "start_command": "python3 demo.py"
        }
    ]
    
    for brain in brains:
        # Create brain directory
        brain_dir = test_dir / brain["name"]
        brain_dir.mkdir(exist_ok=True)
        
        # Create brain.yaml manifest
        manifest = {
            "name": brain["display_name"],
            "description": brain["description"],
            "start_command": brain["start_command"]
        }
        
        with open(brain_dir / "brain.yaml", 'w') as f:
            yaml.dump(manifest, f, default_flow_style=False)
        
        # Create demo.py for demo-brain
        if brain["name"] == "demo-brain":
            demo_script = """#!/usr/bin/env python3
print("🚀 Demo Navigation Brain Starting...")
print("📍 Initializing GPS...")
print("🗺️  Loading waypoints...")
print("🎯 Ready for navigation!")
input("Press Enter to exit...")
"""
            with open(brain_dir / "demo.py", 'w') as f:
                f.write(demo_script)
            os.chmod(brain_dir / "demo.py", 0o755)
        
        print(f"Created test brain: {brain_dir}")

def test_launcher_locally():
    """Test the launcher with local test brains"""
    print("Creating test brains...")
    create_test_brains()
    
    print("\nTo test the launcher:")
    print("1. Edit brain_launcher.py line 26 to use ./test_brains instead of /home/andrewmeckley/rover-brains")
    print("2. Comment out the fullscreen line for windowed testing")
    print("3. Run: python3 brain_launcher.py")
    print("\nOr run the modified version below:")
    
    # Create a test version of the launcher
    test_launcher_code = '''#!/usr/bin/env python3
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
'''
    
    with open("test_launcher_local.py", 'w') as f:
        f.write(test_launcher_code)
    
    print("Created test_launcher_local.py for local testing")

if __name__ == "__main__":
    test_launcher_locally()