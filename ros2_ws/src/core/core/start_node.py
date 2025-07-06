#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringPublisher(Node):
    def __init__(self):
        super().__init__('start_node')
        self.publisher_ = self.create_publisher(String, '/start_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('StringPublisher node started, publishing to /start_topic')

    def timer_callback(self):
        try:
            msg = String()
            msg.data = 'sosal'
            self.publisher_.publish(msg)
            self.get_logger().debug('Published message: "sosal" to /start_topic')
        except Exception as e:
            self.get_logger().error(f'Failed to publish message: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down StringPublisher node')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = StringPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received shutdown request (Ctrl+C)')
    except Exception as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            node.get_logger().info('ROS2 shutdown complete')

if __name__ == '__main__':
    main()