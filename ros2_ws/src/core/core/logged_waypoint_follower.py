#https://github.com/ros-navigation/navigation2_tutorials/blob/rolling/nav2_gps_waypoint_follower_demo/nav2_gps_waypoint_follower_demo/interactive_waypoint_follower.py#L15

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped, PoseStamped
from core.utils.gps_utils import latLonYaw2Geopose
from robot_localization.srv import FromLL
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
        self.wps_file_path = self.get_parameter("wps_file_path").value
        self.wp_parser = YamlWaypointParser(self.wps_file_path)
 
        self.localizer = self.create_client(FromLL,  '/fromLL')
        
        while not self.localizer.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.client_futures = []

        self.get_logger().info('Ready for waypoints...')
        self.parse_wp_cb()

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

            
            self.client_futures.append([self.localizer.call_async(self.req),wp.orientation])
        self.get_logger().info(f"Got {len(wps)} waypoints from file...")

    def command_send_cb(self, future):
        self.resp = PoseStamped()
        self.resp.header.frame_id = 'map'
        self.resp.header.stamp = self.get_clock().now().to_msg()
        self.resp.pose.position = future[0].result().map_point
        self.resp.pose.orientation = future[1]
        # self.get_logger().info("Waypoint added to conversion queue...")
        self.navigator.goToPose(self.resp)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)
        print("Following")

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            incomplete_futures = []
            for f in self.client_futures:      
                if f[0].done():
                    self.get_logger().info("Following converted waypoint...")
                    self.command_send_cb(f)
                else:
                    incomplete_futures.append(f)
                    
            self.client_futures = incomplete_futures
            if len(self.client_futures) == 0:
                self.get_logger().warn("DONE")
                return 0
            
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
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            gepose_wps.append(latLonYaw2Geopose(latitude, longitude, yaw))
        return gepose_wps


