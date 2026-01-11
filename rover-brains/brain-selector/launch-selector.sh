#!/bin/bash
# Launch wrapper for Brain Selector with Cage compositor
# This script is executed by the shell profile on tty1

export DISPLAY=:0
export XDG_RUNTIME_DIR=/run/user/1000
export QT_QPA_PLATFORM=wayland

# Log startup
echo "$(date): Starting Brain Selector with Cage" >> /tmp/brain-selector.log

# Launch selector with Cage compositor
exec cage /home/andrewmeckley/rover-brains/brain-selector/selector.py