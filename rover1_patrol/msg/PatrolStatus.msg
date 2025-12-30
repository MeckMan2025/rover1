# Patrol system status message

# State: idle, recording, pending_save, patrolling, paused, error
string state

# Currently loaded/recording path name
string path_name

# Waypoint progress (0-indexed current, total count)
int32 current_waypoint
int32 total_waypoints

# Loop progress
int32 current_loop
int32 total_loops  # 0 = infinite

# Speed setting (0.0-1.0)
float32 speed_percent

# Error message (empty if no error)
string error_message

# Recording-specific: count of waypoints recorded so far
int32 recorded_waypoint_count
