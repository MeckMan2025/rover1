#导入rclpy库
import rclpy
from rclpy.node import Node
#导入String字符串消息
from std_msgs.msg import String 
#创建一个继承于Node基类的Topic_Pub节点子类 传入一个参数name
class Topic_Pub(Node):
    def __init__(self,name):
        super().__init__(name)
        #创建一个发布者，使用create_publisher的函数，传入的参数分别是：
        #话题数据类型、话题名称、保存消息的队列长度
        self.pub = self.create_publisher(String,"/topic_demo",1) 
        #创建一个定时器，间隔1s进入中断处理函数，传入的参数分别是：
        #中断函数执行的间隔时间，中断处理函数
        self.timer = self.create_timer(1,self.pub_msg)
    #定义中断处理函数
    def pub_msg(self):
        msg = String()	#创建一个String类型的变量msg
        msg.data = "Hi,I send a message." #给msg里边的data赋值
        self.pub.publish(msg) #发布话题数据
        
#主函数
def main():
    rclpy.init() #初始化
    pub_demo = Topic_Pub("publisher_node") #创建Topic_Pub类对象，传入的参数就是节点的名字
    rclpy.spin(pub_demo)     #执行rclpy.spin函数，里边传入一个参数，参数是刚才创建好的Topic_Pub类对象
    pub_demo.destroy_node()  #销毁节点对象
    rclpy.shutdown()         #关闭ROS2 Python接口