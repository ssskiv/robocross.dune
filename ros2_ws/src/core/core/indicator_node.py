# indicator_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist

class IndicatorNode(Node):
    def __init__(self):
        super().__init__('indicator_node')
        self.publisher = self.create_publisher(String, '/indication', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel',self.on_cmd, 10)
        self.status_sub = self.create_subscription(String, '/goal_status', self.on_status, 10)
        self.log  = self.get_logger()
        self.vx = 0.0
        self.log.info('Launched')

    
    def on_status(self, msg):
        match msg.split()[0]:#предполагается разделение по пробелам, решение временное
            case 'moving_forward':
                # publisher.publish('moving')
                self.log.info('Got moving')
            case 'moving_backward':
                self.log.info('Got moving')
            case 'local':
                self.log.info('Got decelerating')    
            case 'stop':
                self.log.info('Got stop')
            case 'lights_on':
                self.log.info('Got lights_on')
            case 'lights_off':
                self.log.info('Got lights_off')
            case 'pause':
                self.log.info('Got pause')
    def on_cmd(self, msg):
        if self.vx - msg.linear.x < 0:
            self.log.info('deccelerating')


def main(args=None):
    rclpy.init(args=args)
    node = IndicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
