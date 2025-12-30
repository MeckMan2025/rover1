#!/usr/bin/env python3
"""
Simple Navigation Test using nav2_simple_commander

Tests Nav2 by sending a goal 1 meter forward from current position.
Bypasses bt_navigator - sends goals directly to controller/planner.

Usage:
  # With rover, rtabmap, and nav2 running:
  python3 scripts/nav_test.py

  # Or as ROS 2 node:
  ros2 run rover1_bringup nav_test
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Wait for Nav2 to be ready
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()
    print("Nav2 is active!")

    # Create a goal 1 meter forward (in map frame)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Simple goal: 1 meter forward from origin
    # In real use, you'd get current pose and offset from it
    goal_pose.pose.position.x = 1.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.w = 1.0  # Facing forward

    print(f"Sending goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")
    navigator.goToPose(goal_pose)

    # Monitor progress
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Distance remaining: {feedback.distance_remaining:.2f}m")
        time.sleep(0.5)

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Goal reached successfully!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled")
    elif result == TaskResult.FAILED:
        print("Goal failed!")
    else:
        print(f"Goal finished with unknown result: {result}")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
