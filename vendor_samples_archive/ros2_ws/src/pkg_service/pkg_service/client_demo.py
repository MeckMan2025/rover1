#导入相关的库
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service_Client(Node):
	def __init__(self,name):
		super().__init__(name)
        #创建客户端，使用的是create_client函数，传入的参数是服务数据的数据类型、服务的话题名称
		self.client = self.create_client(AddTwoInts,'/add_two_ints')
        # 循环等待服务器端成功启动
		while not self.client.wait_for_service(timeout_sec=1.0):
			print("service not available, waiting again...")
        # 创建服务请求的数据对象
		self.request = AddTwoInts.Request()
		
	def send_request(self): 
		self.request.a = 10
		self.request.b = 90
        #发送服务请求
		self.future = self.client.call_async(self.request)
		
def main():
    rclpy.init() #节点初始化
    service_client = Service_Client("client_node") #创建对象
    service_client.send_request() #发送服务请求
    while rclpy.ok():
        rclpy.spin_once(service_client)
        #判断数据是否处理完成
        if service_client.future.done():
            try:
                #获得服务反馈的信息并且打印
                response = service_client.future.result()
                print("service_client.request.a = ",service_client.request.a)
                print("service_client.request.b = ",service_client.request.b)
                print("Result = ",response.sum)
            except Exception as e:
                service_client.get_logger().info('Service call failed %r' % (e,))
        break
    service_client.destroy_node()                    
    rclpy.shutdown()