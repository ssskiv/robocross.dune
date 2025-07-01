# mamba_mavlink_node.py
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import String

class MambaMAVLinkNode(Node):
    def __init__(self):
        super().__init__('mamba_mavlink_node')

        # Connect to MAVLink
        self.master = mavutil.mavlink_connection('/dev/ttyUSB1', baud=115200)
        self.publisher_ = self.create_publisher(String, 'mavlink_heartbeat', 10)
        self.create_timer(1.0, self.read_heartbeat)

    def read_heartbeat(self):
        msg = self.master.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            self.publisher_.publish(String(data=str(msg)))

def main(args=None):
    rclpy.init(args=args)
    node = MambaMAVLinkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
