# indicator_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import UInt8, String
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
        self.msg = UInt8() 
        self.msg.data = int('0b00000000',2)

    """
        0bXXXXXXXX - число, которое отправляется на stm

        0bX------- - всё, что хотите
        0b-X------ - мигание передних фар
        0b--X----- - мигание задних фар
        0b---Х---- - Mayak      МАЯК
        0b----Х--- - Buzzer     СИРЕНА
        0b-----X-- - Reverse    ЗАДНИЕ ФАРЫ
        0b------Х- - Stop       СТОПЫ
        0b-------Х - HeadLight  ПЕРЕДНИЕ ФАРЫ
    """

    def on_status(self, msg):
        e = self.msg
        match msg.data.split()[0]:#предполагается разделение по пробелам, решение временное
            # case 'moving_forward':
            #     self.publisher.publish('0b00010110')
            #     self.log.info('Got moving')
            # case 'moving_backward':
            #     self.publisher.publish('0b00111110')
            #     self.log.info('Got moving')
            case 'local':
                e = int('0b01111101',2)
                # self.publisher.publish('0b01111101')
                self.log.info('Got decelerating')    
            case 'stop':
                e = int('0b00000010',2)
                # self.publisher.publish('0b00000010')
                self.log.info('Got stop')
            # case 'lights_on':
            #     self.log.info('Got lights_on')
            # case 'lights_off':
            #     self.log.info('Got lights_off')
            case 'pause':
                # self.publisher.publish('0b00010000')
                e = int('0b00010000',2)
                self.log.info('Got pause')
        self.msg.data = e
        self.publisher.publish(self.msg)
    def on_cmd(self, msg):
        # если движемся вперёд
        e = int('0b00011001',2)
        if msg.linear.x > 0:
            # если тормозим
            if self.vx - msg.linear.x > 0:
                e = int('0b00011001',2)
                # self.publisher.publish('0b00011001')
                self.log.info('deccelerating')
            # ускор. или пост. скорость
            else:
                e = int('0b00111101',2)
                # self.publisher.publish('0b00111101')
                self.log.info('Got moving')
        # если скорость уменьшается
        if msg.linear.x < 0:
            e = int('0b00111101',2)
            # self.publisher.publish('0b00111101')
            self.log.info('Got moving')
        self.msg.data = e
        self.publisher.publish(self.msg)
        self.vx = msg.linear.x
        

    def lamplighter(self):
        # мигание передних фар
        if (self.msg.data >> 7 & 1):
            self.msg.data ^= (1 << 0) 
            self.publisher.publish(self.msg)
        # мигание задних фар
        if (self.msg.data >> 6 & 1):
            self.msg.data ^= (1 << 2) 
            self.publisher.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = IndicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
