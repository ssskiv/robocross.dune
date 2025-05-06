# uart_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class UARTNode(Node):
    def __init__(self):
        super().__init__('uart_node')
        self.publisher = self.create_publisher(PoseStamped, '/odom_raw', 10)
        self.status_sub = self.create_subscription(String, '/goal_status', self.on_status, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.on_cmd, 10)
        self.get_logger().info('Launched')

    
    def on_status(self, msg):
        pass
    def on_cmd(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = UARTNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
