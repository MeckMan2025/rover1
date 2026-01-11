import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RobotSubscriber(Node):
    def __init__(self, name):
        super().__init__(name)
        # 1. 配置与发布者兼容的QoS策略
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # 2. 创建订阅者：话题名/robot_cmd，回调函数，QoS策略
        self.subscription = self.create_subscription(
            String,
            "/robot_cmd",
            self.cmd_callback,  # 接收到数据后执行的回调函数
            self.qos_profile
        )

    def cmd_callback(self, msg):
        # 回调函数：处理接收到的指令
        self.get_logger().info(f"接收控制指令：{msg.data} → 执行对应动作")


def main(args=None):
    rclpy.init(args=args)
    node = RobotSubscriber("robot_subscriber")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()