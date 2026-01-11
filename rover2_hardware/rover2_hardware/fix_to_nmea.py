import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nmea_msgs.msg import Sentence
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class FixToNmeaBridge(Node):
    def __init__(self):
        super().__init__('fix_to_nmea')

        # QoS setup for U-blox /fix (best effort to match ublox_dgnss publisher)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for /nmea publisher (reliable to match ntrip_client subscriber)
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.fix_callback,
            sub_qos)

        self.publisher = self.create_publisher(Sentence, '/nmea', pub_qos)
        self.get_logger().info('NMEA Bridge v2.0 (Fix -> GPGGA Sentence) started.')

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

        nmea_msg = Sentence()
        nmea_msg.header.stamp = self.get_clock().now().to_msg()
        nmea_msg.header.frame_id = 'gps_link'
        nmea_msg.sentence = full_message
        self.publisher.publish(nmea_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixToNmeaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
