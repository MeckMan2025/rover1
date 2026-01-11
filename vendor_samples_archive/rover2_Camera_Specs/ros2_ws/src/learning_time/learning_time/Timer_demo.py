import rclpy
from rclpy.node import Node

class TimerDemoNode(Node):
    def __init__(self):
        super().__init__('timer_demo_node')
        
        # 计数器，用于演示定时器执行次数
        self.counter = 0
        
        # 创建定时器：每1秒执行一次callback函数
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # 创建一个更快的定时器：每0.5秒执行一次
        self.fast_timer = self.create_timer(0.5, self.fast_timer_callback)
        
        self.get_logger().info("定时器节点已启动")

    def timer_callback(self):
        """1秒定时器回调函数"""
        self.counter += 1
        current_time = self.get_clock().now()
        
        # 打印当前时间和计数器值
        self.get_logger().info(
            f"[1秒定时器] 第 {self.counter} 次执行，当前时间: {current_time.seconds_nanoseconds()}"
        )

    def fast_timer_callback(self):
        """0.5秒定时器回调函数"""
        # 打印当前时间戳（纳秒）
        self.get_logger().info(
            f"[0.5秒定时器] 当前时间戳: {self.get_clock().now().nanoseconds}"
        )

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建节点
    node = TimerDemoNode()
    
    # 运行节点
    rclpy.spin(node)
    
    # 关闭ROS 2
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()