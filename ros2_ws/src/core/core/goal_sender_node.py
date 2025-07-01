# goal_sender_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped 

class GoalSenderNode(Node):
    def __init__(self):
        super().__init__('goal_sender_node')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.status_sub = self.create_subscription(String, '/goal_status', self.on_status, 10)
        self.get_logger().info('Launched')

    
    def on_status(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
