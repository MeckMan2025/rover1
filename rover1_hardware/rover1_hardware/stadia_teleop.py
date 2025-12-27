#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class StadiaTeleop(Node):
    def __init__(self):
        super().__init__('stadia_teleop')

        # Parameters (adjustable at launch)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('deadman_threshold', 0.0)  # L2 starts at 1.0, pressed goes to -1.0
        self.declare_parameter('joystick_deadzone', 0.1)  # Ignore axis values below this threshold
        self.declare_parameter('debug_axes', False)       # Log raw axis values for calibration

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadman_thresh = self.get_parameter('deadman_threshold').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.debug_axes = self.get_parameter('debug_axes').value

        # Stadia Mapping indices (Calibrated Dec 23, 2025)
        self.AXIS_LEFT_X = 0       # Rotate (left +, right -)
        self.AXIS_LEFT_Y = 1       # Forward/Back (fwd +, back -)
        self.AXIS_RIGHT_X = 2      # Strafe (left +, right -)
        self.AXIS_L2 = 5           # Deadman Switch (released=1.0, pressed=-1.0)

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # State tracking for safety
        self.deadman_active = False
        self.is_initialized = False  # Stadia triggers start at 1.0; ignore until we see that.
        self.msg_count = 0

        # Per-axis activation tracking: Stadia sends 1.0 on axes at first connect.
        # Each axis becomes "activated" when it enters the deadzone (user touched & released).
        # Unactivated axes output 0.0 to prevent erratic motion from stale 1.0 values.
        self.axis_activated = {
            self.AXIS_LEFT_X: False,
            self.AXIS_LEFT_Y: False,
            self.AXIS_RIGHT_X: False,
        }
        self.axis_names = {
            self.AXIS_LEFT_X: 'Left X (rotate)',
            self.AXIS_LEFT_Y: 'Left Y (fwd/back)',
            self.AXIS_RIGHT_X: 'Right X (strafe)',
        }

        self.get_logger().info('=== Stadia Teleop Node Started ===')
        self.get_logger().info(f'Max Linear: {self.max_linear} m/s | Max Angular: {self.max_angular} rad/s')
        self.get_logger().info(f'Joystick Deadzone: {self.joystick_deadzone}')
        self.get_logger().info('Controls: Hold L2 to enable. LS=Fwd/Turn, RS=Strafe')
        if self.debug_axes:
            self.get_logger().info('DEBUG MODE: Logging raw axis values')

    def get_axis_value(self, axis_index: int, raw_value: float) -> float:
        """
        Get processed axis value with activation check and deadzone filtering.

        - If axis not yet activated and value enters deadzone, activate it
        - If axis not activated, return 0.0 (ignore stale values)
        - If axis activated, apply deadzone filtering and return value
        """
        in_deadzone = abs(raw_value) < self.joystick_deadzone

        # Check for activation: axis enters deadzone means user has touched & released
        if not self.axis_activated[axis_index] and in_deadzone:
            self.axis_activated[axis_index] = True
            self.get_logger().info(f'{self.axis_names[axis_index]} axis activated')

        # If not activated, ignore this axis entirely
        if not self.axis_activated[axis_index]:
            return 0.0

        # Activated: apply deadzone filtering
        if in_deadzone:
            return 0.0
        return raw_value

    def joy_callback(self, msg):
        self.msg_count += 1
        twist = Twist()

        # Safety check: ensure we have enough axes
        if len(msg.axes) <= self.AXIS_L2:
            if self.msg_count == 1:
                self.get_logger().error(f'Joy msg has only {len(msg.axes)} axes, need at least {self.AXIS_L2 + 1}')
            self.publisher.publish(twist)  # Stop
            return

        l2_value = msg.axes[self.AXIS_L2]
        
        # Initialization Logic: Stadia controllers send 0.0 on all axes until first input.
        # L2 is actually 1.0 when released. We MUST see it hit 1.0 before we trust it.
        if not self.is_initialized:
            if l2_value == 1.0:
                self.is_initialized = True
                self.get_logger().info('Controller Initialized (L2 identity seen at 1.0)')
            else:
                # Still waiting for a valid identity packet
                return

        deadman_pressed = l2_value < self.deadman_thresh

        # Debug logging (throttled to every 20 messages ~1Hz at 20Hz rate)
        if self.debug_axes and self.msg_count % 20 == 0:
            act = self.axis_activated
            self.get_logger().info(
                f'Axes: LX={msg.axes[self.AXIS_LEFT_X]:.2f} LY={msg.axes[self.AXIS_LEFT_Y]:.2f} '
                f'RX={msg.axes[self.AXIS_RIGHT_X]:.2f} L2={l2_value:.2f} | '
                f'Deadman={deadman_pressed} Active=[LX:{act[0]} LY:{act[1]} RX:{act[2]}]'
            )

        # Log deadman state changes
        if deadman_pressed != self.deadman_active:
            self.deadman_active = deadman_pressed
            if deadman_pressed:
                self.get_logger().info('System Armed - move joysticks to enable axes')
            else:
                self.get_logger().info('Deadman RELEASED - stopping')

        if deadman_pressed:
            # Get axis values with per-axis activation check and deadzone filtering
            # Unactivated axes return 0.0, activated axes return filtered value
            left_y = self.get_axis_value(self.AXIS_LEFT_Y, msg.axes[self.AXIS_LEFT_Y])
            left_x = self.get_axis_value(self.AXIS_LEFT_X, msg.axes[self.AXIS_LEFT_X])
            right_x = self.get_axis_value(self.AXIS_RIGHT_X, msg.axes[self.AXIS_RIGHT_X])

            # LS Forward/Back -> linear.x
            twist.linear.x = left_y * self.max_linear

            # RS Left/Right -> linear.y (Strafe)
            twist.linear.y = right_x * self.max_linear

            # LS Left/Right -> angular.z (Rotate)
            twist.angular.z = left_x * self.max_angular

        # Always publish (zero twist when deadman released = immediate stop)
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = StadiaTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
