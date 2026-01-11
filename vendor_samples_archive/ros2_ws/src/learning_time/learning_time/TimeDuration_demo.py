import rclpy
from rclpy.time import Time
from rclpy.duration import Duration

def main():
    rclpy.init()
    node = rclpy.create_node("time_opt_node")
   
    #time类的使用方法，创建‘时间点、时刻’
    time1 = Time(seconds=10)
    time2 = Time(seconds=4)
    #Duration类使用方法，创建‘持续时间、一段时间’
    duration1 = Duration(seconds=3)
    duration2 = Duration(seconds=5)
          
    # 时刻可以进行比较
    node.get_logger().info("time1 >= time2 ? %d" % (time1 >= time2))
    node.get_logger().info("time1 < time2 ? %d" % (time1 < time2))
    
    # 时间段与时刻可以数学运算
    t3 = time1 + duration1
    t4 = time1 - time2    
    t5 = time1 - duration1

    node.get_logger().info("t3 = %d" % t3.nanoseconds)
    node.get_logger().info("t4 = %d" % t4.nanoseconds)
    node.get_logger().info("t5 = %d" % t5.nanoseconds)

    # 时间段可以进行比较
    node.get_logger().info("-" * 80)
    node.get_logger().info("duration1 >= duration2 ? %d" % (duration1 >= duration2))
    node.get_logger().info("duration1 < duration2 ? %d" % (duration1 < duration2))

    rclpy.shutdown()

if __name__ == "__main__":
    main()

