import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# 导入QoS相关类
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ControllerPublisher(Node):
    def __init__(self, name):
        super().__init__(name)
        # 1. 配置QoS策略：可靠传输，保留最后1条历史数据
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 可靠传输（重传丢失数据）
            history=QoSHistoryPolicy.KEEP_LAST,         # 保留最后N条数据
            depth=1                                     # 保留1条历史数据
        )
        # 2. 创建发布者：话题名/robot_cmd，消息类型String，QoS策略
        self.publisher = self.create_publisher(
            String, 
            "/robot_cmd", 
            self.qos_profile
        )
        # 3. 创建定时器：每秒发送一次指令
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cmd_list = ["forward", "backward", "stop"]  # 指令列表
        self.cmd_index = 0  # 指令索引，循环切换


    def timer_callback(self):
        # 循环切换指令（前进→后退→停止→前进...）
        current_cmd = self.cmd_list[self.cmd_index % 3]
        # 创建消息并填充数据
        msg = String()
        msg.data = current_cmd
        # 发布消息
        self.publisher.publish(msg)
        # 打印日志（显示发布的指令）
        self.get_logger().info(f"发布控制指令：{msg.data}")
        # 更新指令索引
        self.cmd_index += 1


def main(args=None):
    # 初始化ROS2
    rclpy.init(args=args)
    # 创建发布者节点
    node = ControllerPublisher("robot_controller_pub")
    # 循环运行节点
    rclpy.spin(node)
    # 销毁节点并关闭ROS2
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()