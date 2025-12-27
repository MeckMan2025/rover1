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
        self.declare_parameter('activation_threshold', 0.3)  # Must move stick past this to prove intent
        self.declare_parameter('debug_axes', False)       # Log raw axis values for calibration

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.deadman_thresh = self.get_parameter('deadman_threshold').value
        self.joystick_deadzone = self.get_parameter('joystick_deadzone').value
        self.activation_threshold = self.get_parameter('activation_threshold').value
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

        # Two-phase axis activation: Stadia axes drift 1.0 → 0.0 after connection.
        # To prevent false activation, we require:
        #   1. axis_was_moved: User pushed stick past activation_threshold (proves intent)
        #   2. axis_activated: User then released stick to deadzone (confirms centering)
        # Only after both phases does the axis respond to input.
        self.axis_was_moved = {
            self.AXIS_LEFT_X: False,
            self.AXIS_LEFT_Y: False,
            self.AXIS_RIGHT_X: False,
        }
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
        Get processed axis value with two-phase activation and deadzone filtering.

        Phase 1: Detect movement - axis must exceed activation_threshold (proves user intent)
        Phase 2: Detect release - axis must enter deadzone (confirms centering)
        Only after both phases does the axis respond to input.

        This prevents false activation from Stadia's natural 1.0 → 0.0 drift.
        """
        in_deadzone = abs(raw_value) < self.joystick_deadzone
        outside_threshold = abs(raw_value) > self.activation_threshold

        # If already activated, just apply deadzone filtering
        if self.axis_activated[axis_index]:
            if in_deadzone:
                return 0.0
            return raw_value

        # Phase 1: Detect intentional movement (stick pushed past threshold)
        if not self.axis_was_moved[axis_index] and outside_threshold:
            self.axis_was_moved[axis_index] = True
            self.get_logger().info(f'{self.axis_names[axis_index]} moved - release to activate')

        # Phase 2: Detect release to center (only if was_moved is True)
        if self.axis_was_moved[axis_index] and in_deadzone:
            self.axis_activated[axis_index] = True
            self.get_logger().info(f'{self.axis_names[axis_index]} activated')

        # Not yet activated - return 0.0
        return 0.0

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
            mv = self.axis_was_moved
            act = self.axis_activated
            self.get_logger().info(
                f'Axes: LX={msg.axes[self.AXIS_LEFT_X]:.2f} LY={msg.axes[self.AXIS_LEFT_Y]:.2f} '
                f'RX={msg.axes[self.AXIS_RIGHT_X]:.2f} L2={l2_value:.2f} | '
                f'Moved=[{mv[0]},{mv[1]},{mv[2]}] Active=[{act[0]},{act[1]},{act[2]}]'
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
