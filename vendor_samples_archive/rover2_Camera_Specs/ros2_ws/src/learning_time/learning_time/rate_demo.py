import rclpy
from rclpy.node import Node
import threading

class RateExampleNode(Node):
    def __init__(self):
        super().__init__("rate_example_node")
        self.get_logger().info("Rate 示例节点启动")

    def run_loop(self):
        # 使用节点的create_rate()创建2Hz的Rate
        rate = self.create_rate(2)
        
        count = 0
        try:
            while rclpy.ok():
                self.get_logger().info(f"循环执行 {count} 次")
                count += 1
                rate.sleep()  # 休眠到下一个周期（0.5秒）
        except KeyboardInterrupt:
            self.get_logger().info("循环被中断")

def main(args=None):
    rclpy.init(args=args)
    node = RateExampleNode()
    
    # 创建线程运行循环（避免阻塞主线程）
    loop_thread = threading.Thread(target=node.run_loop)
    loop_thread.start()
    
    # 主线程执行spin，维持ROS 2节点运行
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        loop_thread.join()  # 等待线程结束
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()