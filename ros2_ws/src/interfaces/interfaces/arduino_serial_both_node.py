# arduino_serial_both_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        self.publisher_ = self.create_publisher(String, 'arduino_data', 10)
        self.subscription = self.create_subscription(String, 'arduino_cmd', self.send_command, 10)

        self.ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        self.create_timer(0.1, self.read_from_arduino)

    def read_from_arduino(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            msg = String()
            msg.data = line
            self.publisher_.publish(msg)

    def send_command(self, msg):
        self.ser.write((msg.data + '\n').encode())

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
