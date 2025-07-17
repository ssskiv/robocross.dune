import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped, PoseStamped
from core.utils.gps_utils import latLonYaw2Geopose
from robot_localization.srv import FromLL
from main.srv import CheckGoal
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import time

class LoggedGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")
        self.declare_parameter("wps_file_path", os.path.join(get_package_share_directory(
        "core"), "config", "gps_waypoints.yaml"))
        self.flag = False
        self.string_subscription = self.create_subscription(
            String,
            '/start_topic',
            self.start_flag,
            5
        )

        self.wps_file_path = self.get_parameter("wps_file_path").value
        self.wp_parser = YamlWaypointParser(self.wps_file_path)
 
        self.localizer = self.create_client(FromLL,  '/fromLL')
        self.checker = self.create_client(CheckGoal, 'checkGoal')
        
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.client_futures = []

        self.get_logger().info('Ready for waypoints...')
        self.parse_wp_cb()
        self.creq = CheckGoal.Request()

    def start_flag(self, msg):
        """
        Callback to set the flag for starting the waypoint following
        """
        if msg.data == "sosal":
            #self.get_logger().info("Starting waypoint following...")
            self.flag = True
        else:
            #self.get_logger().warn("Received unexpected message, not starting waypoint following.")
            self.flag = False

    def parse_wp_cb(self):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """
    
        wps = self.wp_parser.get_wps()

        for wp in wps:
            self.req = FromLL.Request()
            self.req.ll_point.longitude = wp.position.longitude
            self.req.ll_point.latitude = wp.position.latitude
            self.req.ll_point.altitude = wp.position.altitude

            self.client_futures.append([self.localizer.call_async(self.req), wp.orientation, wp.type])
        self.get_logger().info(f"Got {len(wps)} waypoints from file...")

    def command_send_cb(self, future):
        self.resp = PoseStamped()
        self.resp.header.frame_id = 'map'
        self.resp.header.stamp = self.get_clock().now().to_msg()
        self.resp.pose.position = future[0].result().map_point
        self.resp.pose.orientation = future[1]
        
        # Log waypoint type
        wp_type = future[2]
        self.get_logger().info(f"Processing waypoint with type: {wp_type}")
        
        # Perform different actions based on waypoint type
        
        
        self.navigator.goToPose(self.resp)
        while not self.navigator.isTaskComplete():
            time.sleep(0.1)
        self.get_logger().info("Completed navigation to waypoint")

        if wp_type == 0:
            self.get_logger().info("Waypoint Type 0: Standard navigation point")
        elif wp_type == 1:
            self.get_logger().info("Waypoint Type 1: Inspection point")

            self.check_goal(self.get_clock().now().to_msg().sec)  # Call check_goal 
        elif wp_type == 2:
            self.get_logger().info("Waypoint Type 2: Charging station")
            
            #self.check_goal(self.get_clock().now().to_msg().sec)  # Call check_goal 
            #finall
        elif wp_type == 3:
            self.get_logger().info("Waypoint Type 3: Drop-off point")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)

            if self.flag == False:
                self.get_logger().warn("Waiting for start flag...")
                continue 
            incomplete_futures = []
            for f in self.client_futures:      
                if f[0].done():
                    self.get_logger().info("Following converted waypoint...")
                    self.command_send_cb(f)
                    self.get_logger().warn('END FOR THIS POINT')
                    #self.check_goal()
                else:
                    incomplete_futures.append(f)
                    
            self.client_futures = incomplete_futures
            if len(self.client_futures) == 0:
                self.get_logger().warn("DONE")
                return 0
            
    def check_goal(self,time):
        self.creq.time = time
        self.creq.enabled = True
        self.future = self.checker.call_async(self.creq)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(str(self.future.result().status))
        while not self.future.result().status:
           #self.get_logger().warn("I m v while fro check_goal")
            #self.get_logger().warn("Waiting for goal check to complete...")
            rclpy.spin_once(self)
            self.future = self.checker.call_async(self.creq)
            rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
                    
def main():
    rclpy.init()
    gps_wpf = LoggedGpsWpCommander()
    gps_wpf.spin()

if __name__ == "__main__":
    main()
    
class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file with type
        """
        from collections import namedtuple
        GeoPoseWithType = namedtuple('GeoPoseWithType', ['position', 'orientation', 'type'])
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            wp_type = wp.get("type", 0)  # Default to 0 if type is not specified
            gepose = latLonYaw2Geopose(latitude, longitude, yaw)
            gepose_wps.append(GeoPoseWithType(gepose.position, gepose.orientation, wp_type))
        return gepose_wps