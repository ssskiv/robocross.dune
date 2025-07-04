# mamba_mavlink_node.py
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import time

class MambaMAVLinkNode(Node):
    def __init__(self):
        super().__init__('mamba_mavlink_node')

        # Connect to MAVLink
        self.master = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_MambaF405v2_4C0044001451363333393534-if00', baud=115200)
        # self.create_timer(3.0, self.slow_data)
        self.create_timer(0.1, self.frequent_data)
        self.imu_publisher=self.create_publisher(Imu, '/mamba_odom', 10)
        self.imu_coef = 100.0
        self.init_imu()

    def init_imu(self):
        self.master.mav.request_data_stream_send(self.master.target_system, self.master.target_component,
                                                 mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
        while True:
            dmsg = self.read_mamba_msg('RAW_IMU')
            try:
                self.xacc = float(dmsg['xacc'])/self.imu_coef
                self.yacc = float(dmsg['yacc'])/self.imu_coef
                self.zacc = float(dmsg['zacc'])/self.imu_coef
                self.get_logger().info(f'Imu initialized with these zeroes: ax: {self.xacc} ay: {self.yacc} az: {self.zacc}')
                break
            except:
                self.get_logger().warn('No Imu data available, retrying')
                time.sleep(1)
        

    def frequent_data(self):
        dmsg = self.read_mamba_msg('RAW_IMU')
        try:
            xacc = float(dmsg['xacc'])/self.imu_coef# - self.xacc
            yacc = float(dmsg['yacc'])/self.imu_coef# - self.yacc
            zacc = float(dmsg['zacc'])/self.imu_coef# - self.zacc
            xgyro = float(dmsg['xgyro'])
            ygyro = float(dmsg['ygyro'])
            zgyro = float(dmsg['zgyro'])
            self.get_logger().info(f'ax: {xacc}, ay: {yacc}, az: {zacc}')

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'

            msg.linear_acceleration.x = xacc
            msg.linear_acceleration.y = yacc
            msg.linear_acceleration.z = zacc

            msg.angular_velocity.x = xgyro
            msg.angular_velocity.y = ygyro
            msg.angular_velocity.z = zgyro

            self.imu_publisher.publish(msg)
        except:
            self.get_logger().warn('No Imu data available')
        

    def slow_data(self):
        dmsg = self.read_mamba_msg('GPS_RAW_INT')
        try:
            lat = float(dmsg['lat'])
            lon = float(dmsg['lon'])
            self.get_logger().info(f'lat: {lat}, lon: {lon}')
        except:
            self.get_logger().warn('No GPS data available')

        
        
    def read_mamba_msg(self,msg_type):
        time.sleep(0.1)
        msg = self.master.recv_match(type=msg_type, blocking=False)
        if msg:
            s = str(msg)
            s = s[s.index('{')+1:-1]
            dmsg = {i.split(' : ')[0]: i.split(' : ')[1] for i in s.split(', ')} 
            return dmsg
        return {} 

def main(args=None):
    rclpy.init(args=args)
    node = MambaMAVLinkNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    master = mavutil.mavlink_connection('/dev/mamba', baud=115200)
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                                 mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 0)
