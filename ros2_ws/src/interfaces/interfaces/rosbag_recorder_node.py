#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import subprocess
import os
from ament_index_python.packages import get_package_share_directory
from datetime import datetime
import yaml

class RosbagRecorderNode(Node):
    def __init__(self):
        super().__init__('rosbag_recorder_node')

        # Declare parameters
        self.declare_parameter('topics_yaml', '')
        self.declare_parameter('output_dir', '~/ros2_ws/rosbags')

        # Get parameters
        yaml_path = self.get_parameter('topics_yaml').get_parameter_value().string_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        output_dir = os.path.expanduser(output_dir)

        if not yaml_path or not os.path.isfile(yaml_path):
            self.get_logger().error("Parameter 'topics_yaml' is missing or invalid. Loading default")
            yaml_path= os.path.join(
                get_package_share_directory('interfaces'),
                'config',
                'topics.yaml'
                )    
            # rclpy.shutdown()
            # return
        

        topics = self.load_topics_from_yaml(yaml_path)
        if not topics:
            self.get_logger().error("No topics found in YAML file.")
            rclpy.shutdown()
            return

        self.record_process = None
        self.record_rosbag(topics, output_dir)

    def load_topics_from_yaml(self, path):
        try:
            with open(path, 'r') as f:
                config = yaml.safe_load(f)
            return config.get('topics', [])
        except Exception as e:
            self.get_logger().error(f"Error reading YAML: {e}")
            return []

    def record_rosbag(self, topics, output_dir):
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_path = os.path.join(output_dir, f"rosbag_{timestamp}")
        os.makedirs(bag_path, exist_ok=True)

        cmd = ['ros2', 'bag', 'record', '-o', bag_path] + topics
        self.get_logger().info(f"Recording topics: {topics}")
        self.get_logger().info(f"Saving to: {bag_path}")

        self.record_process = subprocess.Popen(cmd)

    def destroy_node(self):
        if self.record_process:
            self.get_logger().info("Stopping rosbag recording...")
            self.record_process.terminate()
            self.record_process.wait()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RosbagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
