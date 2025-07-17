# indicator_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import PoseStamped, Twist

class IndicatorNode(Node):
    def __init__(self):
        super().__init__('indicator_node')
        self.publisher = self.create_publisher(UInt8, '/indication_planned', 10)
        #self.cmd_sub = self.create_subscription(Twist, '/cmd_vel',self.on_cmd, 10)
        #self.status_sub = self.create_subscription(String, '/goal_status', self.on_status, 10)
        self.timer = self.create_timer(0.5, self.lamplighter)
        self.log  = self.get_logger()
        self.vx = 0.0
        self.log.info('Launched')
        self.msg = UInt8()
        self.msg.data = 1 #int('0b00000001',2)
        self.publisher.publish(self.msg)
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
            case 'moving':
                e = int('0b00010000', 2)
                self.log.info('Got moving')
            case 'local':
                e = int('0b01110101',2)
                self.log.info('Got decelerating')    
            case 'stop':
                e = int('0b00000010',2)
                self.log.info('Got stop')
            case 'pause':
                e = int('0b00010000',2)
                self.log.info('Got pause')
            case 'off':
                e = int('0b00000000',2)
                self.log.info('Got off')
        self.msg.data = e
        self._pub(self.msg)
    def on_cmd(self, msg):
        # если движемся вперёд
        e = int('0b00010001',2)
        if msg.linear.x > 0:
            # если тормозим
            if self.vx - msg.linear.x > 0:
                e = int('0b00010001',2)
                self.log.info('deccelerating')
            # ускор. или пост. скорость
            else:
                e = int('0b00110101',2)
                self.log.info('Got moving')
        # если скорость уменьшается
        if msg.linear.x < 0:
            e = int('0b00110101',2)
            self.log.info('Got moving')
        if msg.linear.x == 0:
            e = int('0b00000011',2)#TODO: FIX TO TURN ON SIREN (0b00010000)
            self.log.info('Got pause')
        self.msg.data = e
        self._pub(self.msg)
        self.vx = msg.linear.x
        

    def lamplighter(self):
        # мигание передних фар
        if (self.msg.data >> 6 & 1):
            self.msg.data ^= (1 << 0) 
            self._pub(self.msg)
        # мигание задних фар
        if (self.msg.data >> 5 & 1):
            self.msg.data ^= (1 << 2) 
            self._pub(self.msg)

    def _pub(self, msg):
        if msg.data:
            if msg.data ==0:
                self.publisher.publish(msg)
            else:
                s = bin(msg.data)
                print(s)
                k = '0b000'
                u = True
                
                for i in range(4,len(s)):
                    print(s[i])
                    
                    if i == 6 and u:
                        k+='0'
                        print('SAS')
                    else:
                        k+=s[i]
                
                print(k)
                msg.data = int(s,2)
                self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IndicatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
