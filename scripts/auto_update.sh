#!/bin/bash
# Rover1 Auto-Updater
# Checks for updates on GitHub and pulls them.
# Can be run via Cron or Systemd.

WORKSPACE_DIR="$HOME/ros2_ws/src/rover1"

# Check if we have network
wget -q --spider http://github.com

if [ $? -eq 0 ]; then
    cd $WORKSPACE_DIR
    echo "Checking for updates..."
    git fetch
    
    HEADHASH=$(git rev-parse HEAD)
    UPSTREAMHASH=$(git rev-parse @{u})
    
    if [ "$HEADHASH" != "$UPSTREAMHASH" ]; then
        echo "Update found. Pulling..."
        git pull
        
        # Optional: Revert local changes to tracked files to ensure clean pull
        # git reset --hard origin/main
        
        # Rebuild if package.xml changed? 
        # For now, just pull. User might need to rebuild manually or we can trigger it.
        echo "Update applied."
        
        # Optional: Restart rover service if running
        # sudo systemctl restart rover1.service
    else
        echo "Already up to date."
    fi
else
    echo "No internet connection."
fi
