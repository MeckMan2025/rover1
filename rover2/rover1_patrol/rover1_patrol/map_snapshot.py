#!/usr/bin/env python3
"""
Map Snapshot Node for Teach & Repeat Patrol System

Subscribes to /map (OccupancyGrid) and generates PNG images
with waypoint path overlays for dashboard visualization.

Services:
  /patrol/save_map_snapshot - Save current map with waypoints as PNG
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import os
from pathlib import Path

from rover1_patrol.srv import SaveMapSnapshot


class MapSnapshot(Node):
    """Generates map PNG images with waypoint overlays."""

    def __init__(self):
        super().__init__('map_snapshot')

        # Parameters
        self.declare_parameter('paths_directory', os.path.expanduser('~/paths'))
        self.paths_dir = Path(self.get_parameter('paths_directory').value)

        # Ensure paths directory exists
        self.paths_dir.mkdir(parents=True, exist_ok=True)

        # Latest map data
        self.latest_map = None
        self.map_lock = False

        # Subscribe to map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Callback group for service
        self.cb_group = ReentrantCallbackGroup()

        # Service
        self.snapshot_srv = self.create_service(
            SaveMapSnapshot,
            '/patrol/save_map_snapshot',
            self.save_snapshot_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info(f'Map Snapshot ready. Paths dir: {self.paths_dir}')

    def map_callback(self, msg: OccupancyGrid):
        """Store latest map data."""
        self.latest_map = msg

    def world_to_pixel(self, wx: float, wy: float, info) -> tuple:
        """Convert world coordinates to pixel coordinates."""
        # Map origin is the world position of the (0,0) pixel
        px = int((wx - info.origin.position.x) / info.resolution)
        py = int((wy - info.origin.position.y) / info.resolution)
        return px, py

    def generate_map_image(self, occupancy_grid: OccupancyGrid,
                           waypoints_x: list, waypoints_y: list) -> np.ndarray:
        """Generate a map image with waypoint overlay."""
        info = occupancy_grid.info
        width = info.width
        height = info.height

        # Convert occupancy grid to image
        # OccupancyGrid values: -1 = unknown, 0 = free, 100 = occupied
        data = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))

        # Create RGB image
        img = np.zeros((height, width, 3), dtype=np.uint8)

        # Color mapping
        # Unknown (-1) -> gray
        # Free (0) -> white
        # Occupied (1-100) -> black gradient
        unknown_mask = data == -1
        free_mask = data == 0
        occupied_mask = data > 0

        img[unknown_mask] = [128, 128, 128]  # Gray
        img[free_mask] = [255, 255, 255]  # White
        img[occupied_mask] = [0, 0, 0]  # Black

        # Draw waypoints if provided
        if len(waypoints_x) > 0 and len(waypoints_y) > 0:
            # Convert waypoints to pixels
            pixels = []
            for wx, wy in zip(waypoints_x, waypoints_y):
                px, py = self.world_to_pixel(wx, wy, info)
                # Flip Y because image origin is top-left
                py = height - 1 - py
                pixels.append((px, py))

            # Draw path lines (green)
            for i in range(1, len(pixels)):
                cv2.line(img, pixels[i-1], pixels[i], (0, 200, 0), 2)

            # Draw waypoint dots (green, larger)
            for i, (px, py) in enumerate(pixels):
                if 0 <= px < width and 0 <= py < height:
                    cv2.circle(img, (px, py), 4, (0, 255, 0), -1)

            # Mark start (blue) and end (red) with larger circles
            if len(pixels) >= 1:
                start = pixels[0]
                if 0 <= start[0] < width and 0 <= start[1] < height:
                    cv2.circle(img, start, 8, (255, 0, 0), -1)  # Blue = start
                    cv2.circle(img, start, 8, (255, 255, 255), 2)  # White outline

            if len(pixels) >= 2:
                end = pixels[-1]
                if 0 <= end[0] < width and 0 <= end[1] < height:
                    cv2.circle(img, end, 8, (0, 0, 255), -1)  # Red = end
                    cv2.circle(img, end, 8, (255, 255, 255), 2)  # White outline

        return img

    def save_snapshot_callback(self, request, response):
        """Save a map snapshot with waypoints."""
        if self.latest_map is None:
            response.success = False
            response.message = 'No map data available'
            response.image_path = ''
            return response

        path_name = request.path_name.strip()
        if not path_name:
            response.success = False
            response.message = 'Path name cannot be empty'
            response.image_path = ''
            return response

        # Sanitize filename
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        if not safe_name:
            response.success = False
            response.message = 'Invalid path name'
            response.image_path = ''
            return response

        try:
            # Generate image
            img = self.generate_map_image(
                self.latest_map,
                list(request.waypoints_x),
                list(request.waypoints_y)
            )

            # Save image
            image_path = self.paths_dir / f'{safe_name}.png'
            cv2.imwrite(str(image_path), img)

            self.get_logger().info(f'Saved map snapshot to {image_path}')

            response.success = True
            response.message = f'Map snapshot saved'
            response.image_path = str(image_path)

        except Exception as e:
            self.get_logger().error(f'Failed to save map snapshot: {e}')
            response.success = False
            response.message = f'Failed to save: {e}'
            response.image_path = ''

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapSnapshot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
