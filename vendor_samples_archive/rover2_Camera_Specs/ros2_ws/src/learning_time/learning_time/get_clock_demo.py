import rclpy
from rclpy.node import Node
from rclpy.time import Time

class TimeExampleNode(Node):
    def __init__(self):
        super().__init__("time_example_node")
        
        # 获取节点的时钟对象（默认使用系统时钟）
        self.clock = self.get_clock()
        
        # 获取当前时间（返回Time对象）
        current_time = self.clock.now()
        self.get_logger().info(f"当前时间：{current_time}")

def main(args=None):
    rclpy.init(args=args)
    node = TimeExampleNode()
    rclpy.spin_once(node)  # 运行一次节点
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()