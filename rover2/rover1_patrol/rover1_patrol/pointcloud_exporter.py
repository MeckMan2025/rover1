#!/usr/bin/env python3
"""
Point Cloud Exporter Node for Teach & Repeat Patrol System

Subscribes to /rtabmap/cloud_map and exports point clouds to PLY files
when patrol paths are saved. Downsamples for efficient web viewing.

Services:
  /patrol/save_pointcloud - Save current point cloud as PLY file
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct
import os
from pathlib import Path

from rover1_patrol.srv import SavePointCloud


class PointCloudExporter(Node):
    """Exports RTAB-Map point clouds to PLY files for web visualization."""

    def __init__(self):
        super().__init__('pointcloud_exporter')

        # Parameters
        self.declare_parameter('paths_directory', os.path.expanduser('~/paths'))
        self.declare_parameter('voxel_size', 0.05)  # 5cm voxel grid
        self.declare_parameter('max_points', 200000)  # Limit for browser performance

        self.paths_dir = Path(self.get_parameter('paths_directory').value)
        self.voxel_size = self.get_parameter('voxel_size').value
        self.max_points = self.get_parameter('max_points').value

        # Ensure paths directory exists
        self.paths_dir.mkdir(parents=True, exist_ok=True)

        # Latest point cloud data
        self.latest_cloud = None

        # Subscribe to RTAB-Map point cloud
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/rtabmap/cloud_map',
            self.cloud_callback,
            10
        )

        # Callback group for service
        self.cb_group = ReentrantCallbackGroup()

        # Service
        self.export_srv = self.create_service(
            SavePointCloud,
            '/patrol/save_pointcloud',
            self.save_pointcloud_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info(
            f'Point Cloud Exporter ready. Voxel size: {self.voxel_size}m, '
            f'Max points: {self.max_points}, Paths dir: {self.paths_dir}'
        )

    def cloud_callback(self, msg: PointCloud2):
        """Store latest point cloud."""
        self.latest_cloud = msg

    def parse_pointcloud2(self, cloud_msg: PointCloud2) -> np.ndarray:
        """
        Parse PointCloud2 message to numpy array.
        Returns array of shape (N, 6) with columns [x, y, z, r, g, b]
        """
        # Get field offsets
        field_names = [f.name for f in cloud_msg.fields]
        field_offsets = {f.name: f.offset for f in cloud_msg.fields}

        # Check for required fields
        has_xyz = all(f in field_names for f in ['x', 'y', 'z'])
        has_rgb = 'rgb' in field_names or 'rgba' in field_names

        if not has_xyz:
            self.get_logger().error('Point cloud missing XYZ fields')
            return np.array([])

        # Parse points
        points = []
        point_step = cloud_msg.point_step
        data = cloud_msg.data

        for i in range(cloud_msg.width * cloud_msg.height):
            offset = i * point_step

            # Extract XYZ
            x = struct.unpack_from('f', data, offset + field_offsets['x'])[0]
            y = struct.unpack_from('f', data, offset + field_offsets['y'])[0]
            z = struct.unpack_from('f', data, offset + field_offsets['z'])[0]

            # Skip invalid points (NaN)
            if np.isnan(x) or np.isnan(y) or np.isnan(z):
                continue

            # Extract RGB if available
            if has_rgb:
                rgb_field = 'rgb' if 'rgb' in field_names else 'rgba'
                rgb_offset = field_offsets[rgb_field]
                # RGB is packed as float32 containing 3 bytes
                rgb_bytes = struct.unpack_from('I', data, offset + rgb_offset)[0]
                r = (rgb_bytes >> 16) & 0xFF
                g = (rgb_bytes >> 8) & 0xFF
                b = rgb_bytes & 0xFF
            else:
                # Default gray color
                r, g, b = 128, 128, 128

            points.append([x, y, z, r, g, b])

        return np.array(points, dtype=np.float32)

    def voxel_downsample(self, points: np.ndarray) -> np.ndarray:
        """
        Downsample point cloud using voxel grid filter.
        Keeps one point per voxel and limits total points.
        """
        if len(points) == 0:
            return points

        # Quantize XYZ to voxel grid indices
        xyz = points[:, :3]
        voxel_indices = (xyz / self.voxel_size).astype(np.int32)

        # Find unique voxels (keeps first occurrence)
        _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
        sampled = points[unique_idx]

        # If still too many points, random sample
        if len(sampled) > self.max_points:
            idx = np.random.choice(len(sampled), self.max_points, replace=False)
            sampled = sampled[idx]

        return sampled

    def write_ply(self, points: np.ndarray, filepath: Path) -> bool:
        """Write points to PLY file (ASCII format with RGB)."""
        try:
            with open(filepath, 'w') as f:
                # PLY header
                f.write('ply\n')
                f.write('format ascii 1.0\n')
                f.write(f'element vertex {len(points)}\n')
                f.write('property float x\n')
                f.write('property float y\n')
                f.write('property float z\n')
                f.write('property uchar red\n')
                f.write('property uchar green\n')
                f.write('property uchar blue\n')
                f.write('end_header\n')

                # Write points
                for p in points:
                    x, y, z, r, g, b = p
                    f.write(f'{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n')

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to write PLY: {e}')
            return False

    def save_pointcloud_callback(self, request, response):
        """Save point cloud to PLY file."""
        if self.latest_cloud is None:
            response.success = False
            response.message = 'No point cloud data available'
            response.file_path = ''
            response.point_count = 0
            return response

        path_name = request.path_name.strip()
        if not path_name:
            response.success = False
            response.message = 'Path name cannot be empty'
            response.file_path = ''
            response.point_count = 0
            return response

        # Sanitize filename
        safe_name = "".join(c for c in path_name if c.isalnum() or c in ('_', '-')).lower()
        if not safe_name:
            response.success = False
            response.message = 'Invalid path name'
            response.file_path = ''
            response.point_count = 0
            return response

        try:
            # Parse point cloud
            self.get_logger().info('Parsing point cloud...')
            points = self.parse_pointcloud2(self.latest_cloud)

            if len(points) == 0:
                response.success = False
                response.message = 'Point cloud is empty'
                response.file_path = ''
                response.point_count = 0
                return response

            original_count = len(points)
            self.get_logger().info(f'Parsed {original_count} points')

            # Downsample
            self.get_logger().info('Downsampling...')
            points = self.voxel_downsample(points)
            self.get_logger().info(f'Downsampled to {len(points)} points')

            # Write PLY file
            ply_path = self.paths_dir / f'{safe_name}_cloud.ply'
            if not self.write_ply(points, ply_path):
                response.success = False
                response.message = 'Failed to write PLY file'
                response.file_path = ''
                response.point_count = 0
                return response

            self.get_logger().info(f'Saved point cloud to {ply_path}')

            response.success = True
            response.message = f'Saved {len(points)} points (from {original_count})'
            response.file_path = str(ply_path)
            response.point_count = len(points)

        except Exception as e:
            self.get_logger().error(f'Failed to export point cloud: {e}')
            response.success = False
            response.message = f'Export failed: {e}'
            response.file_path = ''
            response.point_count = 0

        return response


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudExporter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
