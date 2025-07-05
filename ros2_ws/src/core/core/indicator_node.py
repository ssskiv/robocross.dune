# indicator_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseStamped, Twist

class IndicatorNode(Node):
    def __init__(self):
        super().__init__('indicator_node')
        self.publisher = self.create_publisher(UInt8, '/indication', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel',self.on_cmd, 10)
        self.status_sub = self.create_subscription(String, '/goal_status', self.on_status, 10)
        self.timer = self.create_timer(0.5, self.lamplighter)
        self.log  = self.get_logger()
        self.vx = 0.0
        self.log.info('Launched')

    """
        0bXXXXXXXX - число, которое отправляется на stm

        0bX------- - всё, что хотите
        0b-X------ - мигание передних фар
        0b--X----- - мигание задних фар
        0b---Х---- - ПЕРЕДНИЕ ФАРЫ
        0b----Х--- - ЗАДНИЕ ФАРЫ БЕЛЫЕ
        0b-----X-- - МАЯК (возможно, 0 будет означать вкл. красных фар)
        0b------Х- - СИРЕНА
        0b-------Х - ЗАДНИЕ ФАРЫ КРАСНЫЙ
        
    """

    def on_status(self, msg):
        match msg.split()[0]:#предполагается разделение по пробелам, решение временное
            case 'moving_forward':
                self.publisher.publish('0b00010110')
                self.log.info('Got moving')
            case 'moving_backward':
                self.publisher.publish('0b00111110')
                self.log.info('Got moving')
            case 'local':
                self.publisher.publish('0b01111110')
                self.log.info('Got decelerating')    
            case 'stop':
                self.publisher.publish('0b00000001')
                self.log.info('Got stop')
            case 'lights_on':
                self.log.info('Got lights_on')
            case 'lights_off':
                self.log.info('Got lights_off')
            case 'pause':
                self.publisher.publish('0b00000100')
                self.log.info('Got pause')
    def on_cmd(self, msg):
        # если скорость уменьшается
        if self.vx - msg.linear.x > 0:
            self.publisher.publish('0b01010110')
            self.log.info('deccelerating')

    def lamplighter(self, msg):
        # мигание передних фар
        if (msg >> 7 & 1):
            msg ^= (1 << 4) 
            self.publisher.publish(msg)
        # мигание задних фар
        if (msg >> 6 & 1):
            msg ^= (1 << 3) 
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IndicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
