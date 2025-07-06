# indicator_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import PoseStamped, Twist

class SpeedManagerNode(Node):
    def __init__(self):
        super().__init__('speed_manager_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel_planned',self.on_cmd, 10)
        self.status_sub = self.create_subscription(String, '/goal_status', self.on_status, 10)
        self.log  = self.get_logger()
        self.vx = 0.0
        self.log.info('Launched')
        self.state = True


    def on_status(self, msg):
        match msg.data.split()[0]:#предполагается разделение по пробелам, решение временное
            case 'moving':
                self.log.info('Got moving')
                self.state = True
            case 'stop':
                self.log.info('Got stop')
                self.state = False
            case 'pause':
                self.log.info('Got pause')
                self.state = False
    def on_cmd(self, msg):
        # если скорость уменьшается
        if self.state:
            self.publisher.publish(msg)
            self.log.info('moving')
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.log.info('stopping')



def main(args=None):
    rclpy.init(args=args)
    node = SpeedManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
