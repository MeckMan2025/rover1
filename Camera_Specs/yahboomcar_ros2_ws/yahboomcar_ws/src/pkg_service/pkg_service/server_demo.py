#导入相关的库文件
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service_Server(Node):
	def __init__(self,name):
		super().__init__(name)
        #创建一个服务端，使用的是create_service函数，传入的参数分别是：
        #服务数据的数据类型、服务的名称，服务回调函数（也就是服务的内容）
		self.srv = self.create_service(AddTwoInts, '/add_two_ints', self.Add2Ints_callback)
	#这里的服务回调函数的内容是把两个整型数相加，然后返回相加的结果	
	def Add2Ints_callback(self,request,response):
		response.sum = request.a + request.b
		print("response.sum = ",response.sum)
		return response
def main():
    rclpy.init()
    server_demo = Service_Server("publisher_node")
    rclpy.spin(server_demo)
    server_demo.destroy_node()                     # 销毁节点对象
    rclpy.shutdown()                               # 关闭ROS2 Python接口