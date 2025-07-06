# goal_checker_node.py
import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from main.srv import CheckGoal
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import time
import math


class GoalCheckerNode(Node):
    def __init__(self):
        super().__init__('goal_checker_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.publisher_ = self.create_publisher(String, '/goal_status', 10)
        self.cam_sub = self.create_subscription(Image, '/camera/image', self.on_image, 10)
        self.dcam_sub = self.create_subscription(Image, '/depth_camera/image', self.on_image, 10)
        self.depth_sub = self.create_subscription(PointCloud2, '/depth_camera/image_points', self.on_depth, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)

        self.yolo_point_sub = self.create_subscription(PoseStamped, '/checkpoint/point', self.on_yolo_point, 10)
        
        self.goal_pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.srv = self.create_service(CheckGoal, 'checkGoal', self.check_goal)


        # Declare parameter for nav2_params.yaml path
        self.declare_parameter("nav2_params_path", os.path.join(get_package_share_directory(
            "main"), "config", "nav2_params.yaml"))

        # Read xy_goal_tolerance from nav2_params.yaml
        nav2_params_path = self.get_parameter("nav2_params_path").value
        try:
            with open(nav2_params_path, 'r') as file:
                nav2_params = yaml.safe_load(file)
                # Access controller_server.goal_checker.xy_goal_tolerance
                self.default_tolerance = nav2_params.get('controller_server', {}).get('ros__parameters', {}).get('goal_checker', {}).get('xy_goal_tolerance', 4.0)
                self.get_logger().info(f'Loaded default xy_goal_tolerance: {self.default_tolerance}m from nav2_params.yaml')
        except Exception as e:
            self.get_logger().error(f'Failed to load nav2_params.yaml: {e}. Using default xy_goal_tolerance: 4.0m')
            self.default_tolerance = 4.0


        # Инициализация переменных
        self.flagg = False
        self.odom = None
        self.yolo_pose = None
        self.xy_goal_tolerance = self.default_tolerance
        #self.xy_goal_tolerance = 4.0  # Начальная толерантность (для внутренней логики)
        #self.default_tolerance = 4.0
        self.target_tolerance = 0.5
        self.last_yolo_time = None
        self.yolo_timeout = 10.0  # Таймаут для YOLO в секундах
        self.navigator = BasicNavigator("basic_navigator")

        # Клиент для изменения параметров Nav2
        self.param_client = self.create_client(SetParameters, '/controller_server/set_parameters')
        while not self.param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for controller_server/set_parameters service...')
        
        self.get_logger().info('Launched')

    def set_nav2_xy_goal_tolerance(self, tolerance):
        """Set xy_goal_tolerance parameter in controller_server"""
        if not self.param_client.service_is_ready():
            self.get_logger().error('Parameter service /controller_server/set_parameters is not available')
            return False

        param = Parameter()
        param.name = 'goal_checker.xy_goal_tolerance'
        param.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=tolerance)

        request = SetParameters.Request()
        request.parameters = [param]

        # Store future and tolerance for async processing
        future = self.param_client.call_async(request)
        future.tolerance = tolerance  # Attach tolerance to future for logging
        self.pending_param_future = future
        self.get_logger().debug(f'Requested to set xy_goal_tolerance to {tolerance}m')
        return True


    def check_goal(self, request, response):
        #self.get_logger().warn('Checking goal...')
        

        
        self.last_yolo_time = request.time
        self.get_logger().warn('Last YOLO time is None, setting to request time')

        # Check odometry availability
        if self.odom is None:
            self.get_logger().error('No odometry data available')
            response.status = False
            return response

        # Check for valid YOLO data
        
        current_time = self.get_clock().now().to_msg().sec
        if (current_time - self.last_yolo_time > self.yolo_timeout):
            
            # time.sleep(1.0)  # Wait for a second before retrying
            #TEST NAVIGATOR
            if (not self.flagg) and self.navigator.isTaskComplete():

                
            
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'cam_link'  # Use camera frame for YOLO pose
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose.position.x = 3.0
                goal_pose.pose.position.y = 3.0
                goal_pose.pose.position.z = 0.0
                self.get_logger().info('Sending YOLO pose to navigator from check_goal')

                self.set_nav2_xy_goal_tolerance(self.target_tolerance)
                self.xy_goal_tolerance = self.target_tolerance
                self.get_logger().info(f'Set goal tolerance to {self.xy_goal_tolerance}m')

                self.flagg = True
                self.navigator.goToPose(goal_pose)
                

            if not self.navigator.isTaskComplete():
                self.get_logger().info('Navigation to YOLO waypoint in progress...')
                response.status = False
                return response 

            if self.flagg and self.navigator.isTaskComplete():
                self.get_logger().info('YAAAAPYYYuwu')
                #Reset tolerance to default
                self.set_nav2_xy_goal_tolerance(self.default_tolerance)
                self.xy_goal_tolerance = self.default_tolerance
                self.get_logger().info(f'Reset goal tolerance to {self.xy_goal_tolerance}m')
                self.flagg = False
                response.status = True
                return response  
                
            
            self.get_logger().warn('CHDEM')  
            response.status = False
            return response 

                
        elif (current_time - self.last_yolo_time < self.yolo_timeout):
            self.get_logger().warn('SELFKILLED TIME: ' + str(current_time - self.last_yolo_time)) 
            response.status = False
            return response



        # # Verify YOLO pose frame_id
        # if self.yolo_pose.header.frame_id != 'map':
        #     self.get_logger().error(f'YOLO pose frame_id is {self.yolo_pose.header.frame_id}, expected "map"')
        #     response.status = False
        #     return response

       
        

        # Send YOLO pose to navigator
        
        # goal_pose = PoseStamped()
        # goal_pose.header.frame_id = 'map'
        # goal_pose.header.stamp = self.get_clock().now().to_msg()
        # goal_pose.pose.position = self.yolo_pose.pose.position
        # goal_pose.pose.orientation = self.yolo_pose.pose.orientation
        # self.get_logger().info('Sending YOLO pose to navigator from check_goal')
        # self.navigator.goToPose(goal_pose)

        # # Check if navigation task is complete
        # if self.navigator.isTaskComplete():
        #     self.get_logger().info('Goal reached!')
        #     response.status = True
        #     # Reset tolerance to default
        #     self.set_nav2_xy_goal_tolerance(self.default_tolerance)
        #     self.xy_goal_tolerance = self.default_tolerance
        #     self.get_logger().info(f'Reset goal tolerance to {self.xy_goal_tolerance}m')
        # else:
        #     self.get_logger().info('Navigation to YOLO waypoint in progress...')
        #     response.status = False
        
        response.status = False # <<---- На чем держится весь проект 
        return response

    def on_yolo_point(self, msg):
        self.yolo_pose = msg
        self.last_yolo_time = self.get_clock().now().to_msg().sec
        self.get_logger().info('Received YOLO pose data')
    def on_image(self, msg):
        pass
    def on_depth(self, msg):
        pass
    def on_scan(self, msg):
        pass
    def on_odom(self, msg):
        self.odom = msg
        #self.get_logger().debug(f'Received odometry: position ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})')
    def on_goal(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = GoalCheckerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
