# indicator_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped 

class IndicatorNode(Node):
    def __init__(self):
        super().__init__('indicator_node')
        self.publisher = self.create_publisher(String, '/dummy_indicator', 10)
        self.status_sub = self.create_subscription(String, '/goal_status', self.on_status, 10)
        self.get_logger().info('Launched')

    
    def on_status(self, msg):
        match msg.split()[0]:#предполагается разделение по пробелам, решение временное
            case 'moving':
                # publisher.publish('moving')
                self.get_logger().info('Got moving')
            case 'stop':
                self.get_logger().info('Got stop')
            case 'lights_on':
                self.get_logger().info('Got lights_on')
            case 'lights_off':
                self.get_logger().info('Got lights_off')

def main(args=None):
    rclpy.init(args=args)
    node = IndicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
