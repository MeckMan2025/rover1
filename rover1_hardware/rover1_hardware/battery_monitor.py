import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import smbus2 as smbus
import time

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        
        # Parameters
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x34)
        self.declare_parameter('publish_rate', 1.0) # Hz
        
        bus_num = self.get_parameter('i2c_bus').value
        self.addr = self.get_parameter('i2c_address').value
        rate = self.get_parameter('publish_rate').value
        
        # Initialize I2C
        try:
            self.bus = smbus.SMBus(bus_num)
            self.get_logger().info(f"Battery Monitor started on I2C bus {bus_num} at address {hex(self.addr)}")
        except Exception as e:
            self.get_logger().error(f"Failed to open I2C bus {bus_num}: {e}")
            raise e
            
        # Publisher
        self.publisher_ = self.create_publisher(Float32, 'battery_voltage', 10)
        
        # Timer
        self.timer = self.create_timer(1.0/rate, self.timer_callback)
        
    def timer_callback(self):
        try:
            # Register 0x00: Battery voltage in mV (2 bytes, little endian)
            data = self.bus.read_i2c_block_data(self.addr, 0x00, 2)
            
            if len(data) >= 2:
                voltage_mv = data[0] + (data[1] << 8)
                voltage = float(voltage_mv) / 1000.0
                
                msg = Float32()
                msg.data = voltage
                self.publisher_.publish(msg)
                
                # Periodic log for confirmation
                self.get_logger().debug(f"Published voltage: {voltage:.2f}V")
        except Exception as e:
            self.get_logger().warn(f"Failed to read battery voltage: {e}")

def main(args=None):
    rclpy.init(args=args)
    try:
        node = BatteryMonitor()
        rclpy.spin(node)
    except Exception as e:
        print(f"Node initialization failed: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
