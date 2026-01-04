#!/usr/bin/env python3
"""
Velocity Scaler Node for Teach & Repeat Patrol System

Scales velocity commands based on patrol speed setting.
Acts as a relay between Nav2 output and motor driver.

Subscriptions:
  /cmd_vel_nav - Raw velocity from Nav2 (remapped from velocity_smoother)
  /patrol/speed_scale - Speed multiplier (0.0-1.0)

Publishers:
  /cmd_vel - Scaled velocity to motor driver
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class VelocityScaler(Node):
    """Scales velocity commands based on patrol speed setting."""

    def __init__(self):
        super().__init__('velocity_scaler')

        # Parameters
        self.declare_parameter('min_scale', 0.1)  # Minimum 10% speed
        self.declare_parameter('default_scale', 1.0)  # Default full speed

        self.min_scale = self.get_parameter('min_scale').value
        self.speed_scale = self.get_parameter('default_scale').value

        # Subscriptions
        self.vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.velocity_callback,
            10
        )

        self.scale_sub = self.create_subscription(
            Float32,
            '/patrol/speed_scale',
            self.scale_callback,
            10
        )

        # Publisher
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(
            f'Velocity Scaler ready. Min scale: {self.min_scale}, '
            f'Default scale: {self.speed_scale}'
        )

    def scale_callback(self, msg: Float32):
        """Update speed scale from patrol manager."""
        # Clamp to valid range
        self.speed_scale = max(self.min_scale, min(1.0, msg.data))

    def velocity_callback(self, msg: Twist):
        """Scale and forward velocity command."""
        scaled_msg = Twist()

        # Scale linear velocities
        scaled_msg.linear.x = msg.linear.x * self.speed_scale
        scaled_msg.linear.y = msg.linear.y * self.speed_scale
        scaled_msg.linear.z = msg.linear.z * self.speed_scale

        # Scale angular velocities
        scaled_msg.angular.x = msg.angular.x * self.speed_scale
        scaled_msg.angular.y = msg.angular.y * self.speed_scale
        scaled_msg.angular.z = msg.angular.z * self.speed_scale

        self.vel_pub.publish(scaled_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityScaler()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
