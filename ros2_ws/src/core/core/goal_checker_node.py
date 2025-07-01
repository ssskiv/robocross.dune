# goal_checker_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist


class GoalCheckerNode(Node):
    def __init__(self):
        super().__init__('goal_checker_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.publisher_ = self.create_publisher(String, '/goal_status', 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image', self.on_image, 10)
        self.dcam_sub = self.create_subscription(Image, '/depth_camera/image', self.on_image, 10)
        self.depth_sub = self.create_subscription(PointCloud2, '/depth_camera/image_points', self.on_depth, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.get_logger().info('Launched')


    def on_image(self, msg):
        pass
    def on_depth(self, msg):
        pass
    def on_scan(self, msg):
        pass
    def on_odom(self, msg):
        pass
    def on_goal(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GoalCheckerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
