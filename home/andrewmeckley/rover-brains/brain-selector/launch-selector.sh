#!/bin/bash
# Launch wrapper for Brain Selector with X11
# Used by VNC xstartup.custom

export QT_QPA_PLATFORM=xcb

# Log startup
echo "$(date): Starting Brain Selector on X11" >> /tmp/brain-selector.log

# Launch selector
exec python3 /home/andrewmeckley/rover-brains/brain-selector/selector.py