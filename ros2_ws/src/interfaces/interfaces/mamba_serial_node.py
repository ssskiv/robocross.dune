# mamba_serial_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class MambaSerialNode(Node):
    def __init__(self):
        super().__init__('mamba_serial_node')
        self.serial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.publisher_ = self.create_publisher(String, 'mamba_raw', 10)
        self.subscription = self.create_subscription(String, 'mamba_cmd', self.send_command, 10)
        self.create_timer(0.1, self.read_serial)

    def read_serial(self):
        if self.serial.in_waiting:
            data = self.serial.readline().decode('utf-8').strip()
            self.publisher_.publish(String(data=data))

    def send_command(self, msg):
        self.serial.write((msg.data + '\n').encode())

def main(args=None):
    rclpy.init(args=args)
    node = MambaSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
