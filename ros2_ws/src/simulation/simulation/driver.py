import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

HALF_DISTANCE_BETWEEN_WHEELS = 0.5 + 0.075
WHEEL_RADIUS = 0.19

class MyRobotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        
        self.last_pos = 0.0
        
        # self.create_subscription(GPS, 'left_sensor', self.__left_sensor_callback, 1)
        
        for i in ['f','m','b']:

            self.__left_motor = self.__robot.getDevice(f'{i}l wheel motor')
            self.__right_motor = self.__robot.getDevice(f'{i}r wheel motor')

            self.__left_motor.setPosition(float('inf'))
            self.__left_motor.setVelocity(0)

            self.__right_motor.setPosition(float('inf'))
            self.__right_motor.setVelocity(0)
            
            self.__left_pos = self.__robot.getDevice(f'{i}l wheel ps')
            self.__right_pos = self.__robot.getDevice(f'{i}r wheel ps')
            self.__left_pos.enable(10)
            self.__right_pos.enable(10)


        self.__target_twist = Twist()

        rclpy.init(args=None)
        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)
        self.odom_pub = self.__node.create_publisher(Odometry, 'stm_odom', 1)
        self.__node.create_timer(0.1, self.__timer_callback)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist
        self.last_pos = twist.linear.x
        # self.__node._logger.info(f'{self.last_pos}')
        
    def __timer_callback(self):
        
        self.__left_pos = self.__robot.getDevice(f'ml wheel ps')
        left = self.__left_pos.getValue() * WHEEL_RADIUS
        self.__right_pos = self.__robot.getDevice(f'mr wheel ps')
        right = self.__right_pos.getValue() * WHEEL_RADIUS
        
        msg = Odometry()
        msg.header.stamp = self.__node.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = (left + right)/2
        msg.twist.twist.linear.x = self.last_pos #((left + right)/2 - self.last_pos)/0.1
        #self.last_pos = (left + right)/2
        
        self.odom_pub.publish(msg=msg)
        
    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        for i in ['f','m','b']:

            self.__left_motor = self.__robot.getDevice(f'{i}l wheel motor')
            self.__right_motor = self.__robot.getDevice(f'{i}r wheel motor')

            self.__left_motor.setVelocity(command_motor_left)
            self.__right_motor.setVelocity(command_motor_right)
            
            