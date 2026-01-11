#!/bin/bash
# Launch wrapper for Brain Selector with Cage compositor
# This script is executed by the shell profile on tty1

export XDG_RUNTIME_DIR=/run/user/1000
export QT_QPA_PLATFORM=wayland

# Log startup
echo "$(date): Starting Brain Selector with Cage" >> /tmp/brain-selector.log

# Launch selector with Cage compositor (capture stderr, allow script to continue)
cage /home/andrewmeckley/rover-brains/brain-selector/selector.py 2>> /tmp/brain-selector.log

# Log cage exit status
echo "$(date): Cage exited with code $?" >> /tmp/brain-selector.log

# Prevent rapid restart loops
sleep 2