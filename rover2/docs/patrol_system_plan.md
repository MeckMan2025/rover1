# Teach & Repeat Patrol System - Implementation Plan

**Date:** December 29, 2025
**Status:** Planning Complete - Ready for Implementation

## Overview

A system that allows the user to:
1. Drive a path manually while recording waypoints
2. Save the path with a name and map preview
3. Load and patrol saved paths autonomously
4. Control patrol via web dashboard

## Architecture

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         PATROL SYSTEM ARCHITECTURE                       │
│                                                                          │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────┐  │
│  │   Dashboard   │───▶│  Web Server  │───▶│  ROS 2 Service Calls     │  │
│  │   (Browser)   │◀───│  (Flask/WS)  │◀───│  & Action Clients        │  │
│  └──────────────┘    └──────────────┘    └──────────────────────────┘  │
│                                                    │                     │
│         ┌──────────────────────────────────────────┼─────────────────┐  │
│         │                                          │                 │  │
│         ▼                                          ▼                 ▼  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐   │  │
│  │  Waypoint    │    │   Patrol     │    │    Map Snapshot      │   │  │
│  │  Recorder    │    │   Manager    │    │    Utility           │   │  │
│  └──────┬───────┘    └──────┬───────┘    └──────────┬───────────┘   │  │
│         │                   │                       │               │  │
│         ▼                   ▼                       ▼               │  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────┐   │  │
│  │    TF2       │    │ /follow_     │    │      /map            │   │  │
│  │ map→base_link│    │  waypoints   │    │  (OccupancyGrid)     │   │  │
│  └──────────────┘    │   (Action)   │    └──────────────────────┘   │  │
│                      └──────────────┘                               │  │
│                             │                                       │  │
│                             ▼                                       │  │
│                      ┌──────────────┐                               │  │
│                      │   Nav2       │                               │  │
│                      │  Controller  │───▶ /cmd_vel ───▶ Motors     │  │
│                      └──────────────┘                               │  │
└─────────────────────────────────────────────────────────────────────────┘
```

## Component Specifications

### 1. waypoint_recorder.py

**Package:** `rover1_patrol` (new package)

**Subscriptions:**
- TF: `map` → `base_link` transform

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/patrol/start_recording` | `std_srvs/Trigger` | Begin recording waypoints |
| `/patrol/stop_recording` | `std_srvs/Trigger` | Stop recording (keeps in memory) |
| `/patrol/save_path` | `rover1_patrol/SavePath` | Save with name, triggers map snapshot |
| `/patrol/discard_recording` | `std_srvs/Trigger` | Discard current recording |

**Publishers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/patrol/recording_status` | `std_msgs/String` | "idle" / "recording" / "pending_save" |
| `/patrol/waypoint_count` | `std_msgs/Int32` | Current count while recording |

**Logic:**
```python
class WaypointRecorder:
    def __init__(self):
        self.recording = False
        self.waypoints = []
        self.last_position = None
        self.min_distance = 0.30  # 30cm = rover length

    def tf_callback(self):
        if not self.recording:
            return

        current_pose = lookup_transform('map', 'base_link')

        if self.last_position is None:
            self.waypoints.append(current_pose)
            self.last_position = current_pose
            return

        distance = euclidean_distance(current_pose, self.last_position)
        if distance >= self.min_distance:
            self.waypoints.append(current_pose)
            self.last_position = current_pose
```

**File Format (YAML):**
```yaml
name: kitchen_loop
recorded: 2025-12-29T20:45:00
waypoint_spacing_m: 0.30
waypoints:
  - x: 0.0
    y: 0.0
    theta: 0.0
  - x: 0.28
    y: 0.05
    theta: 0.1
  # ... more waypoints
```

---

### 2. patrol_manager.py

**Package:** `rover1_patrol`

**Action Client:**
- `/follow_waypoints` (nav2_msgs/FollowWaypoints)

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/patrol/list_paths` | `rover1_patrol/ListPaths` | Returns list of saved paths |
| `/patrol/start_patrol` | `rover1_patrol/StartPatrol` | Start with path name + options |
| `/patrol/stop_patrol` | `std_srvs/Trigger` | Cancel navigation immediately |
| `/patrol/pause_patrol` | `std_srvs/Trigger` | Pause at current position |
| `/patrol/resume_patrol` | `std_srvs/Trigger` | Resume from paused state |

**Publishers:**
| Topic | Type | Description |
|-------|------|-------------|
| `/patrol/status` | `rover1_patrol/PatrolStatus` | Full status message |

**PatrolStatus.msg:**
```
string state          # idle, patrolling, paused, error
string path_name      # Currently loaded path
int32 current_waypoint
int32 total_waypoints
int32 current_loop
int32 total_loops     # 0 = infinite
float32 speed_percent # 0.0-1.0
string error_message  # Empty if no error
```

**StartPatrol.srv:**
```
string path_name
int32 loop_count      # 0 = infinite, >0 = specific count
bool reverse_mode     # Traverse forward then backward
float32 speed_percent # 0.0-1.0 (multiplier for max speed)
---
bool success
string message
```

**Loop Modes:**
1. **Continuous (loop_count=0):** Repeat forever until stopped
2. **N times (loop_count>0):** Complete N full loops, then stop
3. **Reverse mode:** At end of path, rotate 180°, traverse in reverse

**Speed Control:**
- Publish to `/cmd_vel` with velocity scaled by speed_percent
- OR: Dynamically update Nav2 controller params (more complex)
- Recommendation: Use velocity scaling in a wrapper

**Error Handling:**
```python
def feedback_callback(self, feedback):
    # If stuck on same waypoint for too long
    if self.stuck_timeout_exceeded():
        self.cancel_goal()
        self.publish_error("Cannot reach waypoint {n} - path blocked")
        self.state = "error"
```

---

### 3. map_snapshot.py

**Package:** `rover1_patrol`

**Subscriptions:**
- `/map` (nav_msgs/OccupancyGrid)

**Services:**
| Service | Type | Description |
|---------|------|-------------|
| `/patrol/save_map_snapshot` | `rover1_patrol/SaveMapSnapshot` | Save PNG with waypoints overlay |

**SaveMapSnapshot.srv:**
```
string path_name
float64[] waypoints_x
float64[] waypoints_y
---
bool success
string image_path
```

**Image Generation:**
```python
def generate_map_image(occupancy_grid, waypoints):
    # Convert OccupancyGrid to numpy array
    # -1 (unknown) → gray (128)
    # 0 (free) → white (255)
    # 100 (occupied) → black (0)

    img = np.zeros((height, width, 3), dtype=np.uint8)

    for y in range(height):
        for x in range(width):
            val = grid.data[y * width + x]
            if val == -1:
                img[y, x] = [128, 128, 128]  # Unknown = gray
            elif val == 0:
                img[y, x] = [255, 255, 255]  # Free = white
            else:
                img[y, x] = [0, 0, 0]  # Occupied = black

    # Draw waypoints as connected path
    for i, (wx, wy) in enumerate(waypoints):
        px, py = world_to_pixel(wx, wy, grid.info)
        cv2.circle(img, (px, py), 3, (0, 255, 0), -1)  # Green dots
        if i > 0:
            prev_px, prev_py = world_to_pixel(waypoints[i-1])
            cv2.line(img, (prev_px, prev_py), (px, py), (0, 200, 0), 1)

    # Mark start (blue) and end (red)
    cv2.circle(img, start_pixel, 6, (255, 0, 0), -1)
    cv2.circle(img, end_pixel, 6, (0, 0, 255), -1)

    cv2.imwrite(f'~/paths/{path_name}.png', img)
```

---

### 4. Web Dashboard Additions

**File:** `gnss_web_dashboard/static/index.html`

**Changes:**
1. Rename header: "Rover1 Dashboard" (remove UFO emoji)
2. Add new "Patrol" card

**Patrol Card Layout:**
```
┌─────────────────────────────────────────────────────────────┐
│  Patrol Control                                              │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────┐  ┌─────────────────────────────┐  │
│  │  Saved Paths        │  │  Map Preview                │  │
│  │  ┌───────────────┐  │  │  ┌───────────────────────┐  │  │
│  │  │▼ Select Path  │  │  │  │                       │  │  │
│  │  ├───────────────┤  │  │  │    [Map Image]        │  │  │
│  │  │ kitchen_loop  │  │  │  │    with waypoints     │  │  │
│  │  │ basement      │  │  │  │                       │  │  │
│  │  │ full_house    │  │  │  └───────────────────────┘  │  │
│  │  └───────────────┘  │  │  12 waypoints | Dec 29     │  │
│  └─────────────────────┘  └─────────────────────────────┘  │
│                                                              │
│  ── Recording ──────────────────────────────────────────    │
│  [Start Recording]  [Stop]  [Save...]  [Discard]            │
│  Status: Idle | Waypoints: 0                                 │
│                                                              │
│  ── Patrol ─────────────────────────────────────────────    │
│  Loop: [Continuous ▼]  [N times: ___]  [✓ Reverse mode]     │
│  Speed: [━━━━━●━━━] 70%                                      │
│                                                              │
│  [▶ Start Patrol]  [⏸ Pause]  [■ Stop]                      │
│                                                              │
│  Status: Patrolling kitchen_loop                             │
│  Progress: Waypoint 5 of 23 | Loop 2 of 5                    │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

**Backend Additions (web_dashboard.py):**

```python
# New ROS service clients
self.start_recording_client = self.create_client(Trigger, '/patrol/start_recording')
self.stop_recording_client = self.create_client(Trigger, '/patrol/stop_recording')
self.save_path_client = self.create_client(SavePath, '/patrol/save_path')
self.discard_client = self.create_client(Trigger, '/patrol/discard_recording')
self.list_paths_client = self.create_client(ListPaths, '/patrol/list_paths')
self.start_patrol_client = self.create_client(StartPatrol, '/patrol/start_patrol')
self.stop_patrol_client = self.create_client(Trigger, '/patrol/stop_patrol')
self.pause_patrol_client = self.create_client(Trigger, '/patrol/pause_patrol')
self.resume_patrol_client = self.create_client(Trigger, '/patrol/resume_patrol')

# Subscribe to status topics
self.patrol_status_sub = self.create_subscription(
    PatrolStatus, '/patrol/status', self.patrol_status_callback, 10)
self.recording_status_sub = self.create_subscription(
    String, '/patrol/recording_status', self.recording_status_callback, 10)

# HTTP endpoints for map images
@app.route('/paths/<path_name>.png')
def serve_path_image(path_name):
    return send_file(f'/home/andrewmeckley/paths/{path_name}.png')
```

**WebSocket Message Types:**
```javascript
// From server to client
{
    type: 'patrol_status',
    data: {
        state: 'patrolling',
        path_name: 'kitchen_loop',
        current_waypoint: 5,
        total_waypoints: 23,
        current_loop: 2,
        total_loops: 5,
        speed_percent: 0.7,
        error_message: ''
    }
}

{
    type: 'recording_status',
    data: {
        state: 'recording',
        waypoint_count: 15
    }
}

{
    type: 'path_list',
    data: {
        paths: [
            {name: 'kitchen_loop', waypoints: 23, recorded: '2025-12-29'},
            {name: 'basement', waypoints: 45, recorded: '2025-12-28'}
        ]
    }
}

// From client to server
{type: 'start_recording'}
{type: 'stop_recording'}
{type: 'save_path', name: 'kitchen_loop'}
{type: 'discard_recording'}
{type: 'start_patrol', path: 'kitchen_loop', loops: 0, reverse: false, speed: 0.7}
{type: 'pause_patrol'}
{type: 'resume_patrol'}
{type: 'stop_patrol'}
{type: 'list_paths'}
{type: 'get_path_preview', path: 'kitchen_loop'}
```

---

## Package Structure

```
rover1_patrol/
├── rover1_patrol/
│   ├── __init__.py
│   ├── waypoint_recorder.py
│   ├── patrol_manager.py
│   └── map_snapshot.py
├── msg/
│   └── PatrolStatus.msg
├── srv/
│   ├── SavePath.srv
│   ├── ListPaths.srv
│   ├── StartPatrol.srv
│   └── SaveMapSnapshot.srv
├── launch/
│   └── patrol.launch.py
├── package.xml
├── setup.py
└── resource/
    └── rover1_patrol
```

---

## Implementation Order

1. **Create rover1_patrol package** with message/service definitions
2. **Build waypoint_recorder.py** - Test recording manually via CLI
3. **Build map_snapshot.py** - Test saving PNG manually
4. **Build patrol_manager.py** - Test patrol via CLI
5. **Update web_dashboard.py** - Add service clients and WS handlers
6. **Update index.html** - Add Patrol card UI
7. **Integration testing** - Full end-to-end test

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| TF lookup fails | Add retry logic, check TF tree at startup |
| waypoint_follower doesn't reach goal | Timeout + skip or stop with alert |
| Map snapshot service slow | Run async, show "generating..." in UI |
| Path file corruption | Validate YAML on load, keep backups |
| Speed scaling doesn't work | Test velocity_smoother integration first |

---

## Testing Checklist

- [ ] Record 5 waypoints walking rover by hand
- [ ] Save path, verify YAML + PNG created
- [ ] Load path list in dashboard
- [ ] Preview map image in dashboard
- [ ] Start patrol, verify rover moves
- [ ] Pause mid-patrol, resume
- [ ] Stop patrol, verify rover stops
- [ ] Test loop modes (continuous, N times, reverse)
- [ ] Test speed slider
- [ ] Test error handling (block path, watch for alert)
