# uart_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, PointCloud2
from nav_msgs.msg import Odometry


class REALNode(Node):
    def __init__(self):
        super().__init__('real_node')
        self.p1 = self.create_publisher(PointCloud2, '/pc1', 10)
        self.p2 = self.create_publisher(PointCloud2, '/pc2', 10)
        self.p1 = self.create_subscription(PointCloud2, '/rs1/camera/depth/color/points', self.on_pc1)
        self.p2 = self.create_subscription(PointCloud2, '/rs2/camera/depth/color/points', self.on_pc2)
        self.get_logger().info('Launched')
        ###ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true camera_namespace:=rs1 base_frame_id:=rs1
        #ros2 launch realsense2_camera rs_launch.py enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true camera_namespace:=rs1 base_frame_id:=rs1
    
    def on_status(self, msg):
        pass
    def on_cmd(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = REALNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
