import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class FixToNmeaBridge(Node):
    def __init__(self):
        super().__init__('fix_to_nmea')
        
        # QoS setup for U-blox /fix (best effort)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            qos)
            
        self.publisher = self.create_publisher(String, '/nmea', 10)
        self.get_logger().info('NMEA Bridge (Fix -> GPGGA) started.')

    def fix_callback(self, msg):
        if msg.latitude == 0.0 or msg.longitude == 0.0:
            return

        # Generate GPGGA Sentence
        # Format: $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
        
        now = time.strftime("%H%M%S.00", time.gmtime())
        
        # Latitude: DDMM.MMMM
        lat = abs(msg.latitude)
        lat_deg = int(lat)
        lat_min = (lat - lat_deg) * 60.0
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_dir = 'N' if msg.latitude >= 0 else 'S'
        
        # Longitude: DDDMM.MMMM
        lon = abs(msg.longitude)
        lon_deg = int(lon)
        lon_min = (lon - lon_deg) * 60.0
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_dir = 'E' if msg.longitude >= 0 else 'W'
        
        # RTK Status (1=GPS fix, 2=DGPS fix, 4=RTK Fixed, 5=RTK Float)
        # Note: We just send '1' to the caster to jumpstart the VRS.
        fix_quality = 1 
        
        # Assemble sentence (truncating altitude to keep length < 82)
        alt = round(msg.altitude, 2)
        
        sentence = f"GPGGA,{now},{lat_str},{lat_dir},{lon_str},{lon_dir},{fix_quality},12,1.0,{alt},M,0.0,M,,"
        
        # Calculate Checksum
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        
        full_message = f"${sentence}*{checksum:02X}"
        
        nmea_msg = String()
        nmea_msg.data = full_message
        self.publisher.publish(nmea_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixToNmeaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
