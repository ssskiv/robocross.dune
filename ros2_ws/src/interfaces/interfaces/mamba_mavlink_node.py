# mamba_mavlink_node.py
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import String

class MambaMAVLinkNode(Node):
    def __init__(self):
        super().__init__('mamba_mavlink_node')

        # Connect to MAVLink
        self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        self.publisher_ = self.create_publisher(String, 'mavlink_heartbeat', 10)
        self.create_timer(1.0, self.read_heartbeat)

        # self.master.wait_heartbeat()
        # # Get some information !
        # while True:
        #     try:
        #         altitude = self.master.messages['GPS_RAW_INT'].alt  # Note, you can access message fields as attributes!
        #         timestamp = self.master.time_since('GPS_RAW_INT')
        #         print(altitude)
        #     except:
        #         print('No GPS_RAW_INT message received')

    def read_heartbeat(self):
        # self.master.write()
        # lat = self.master.field('GLOBAL_POSITION_INT', 'lat', 0) * 1.0e-7
        msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
        # msg = flightmode_list()
        if msg :
            self.publisher_.publish(String(data=str(msg)))
            self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = MambaMAVLinkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
