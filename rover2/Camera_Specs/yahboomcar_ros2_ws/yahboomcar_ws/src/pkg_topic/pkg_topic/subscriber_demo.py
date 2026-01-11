#导入相关的库
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Topic_Sub(Node):
	def __init__(self,name):
		super().__init__(name)	
        #创建订阅者使用的是create_subscription，传入的参数分别是：话题数据类型，话题名称，回调函数名称，队列长度
		self.sub = self.create_subscription(String,"/topic_demo",self.sub_callback,1) 
    #回调函数执行程序：打印接收的到信息
	def sub_callback(self,msg):
		print(msg.data)
def main():
	rclpy.init() #ROS2 Python接口初始化
	sub_demo = Topic_Sub("subscriber_node") # 创建对象并进行初始化
	rclpy.spin(sub_demo)
	sub_demo.destroy_node()  #销毁节点对象
	rclpy.shutdown()         #关闭ROS2 Python接口