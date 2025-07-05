#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class StringPublisher(Node):
    def __init__(self):
        super().__init__('start_node')  
        self.publisher_ = self.create_publisher(String, '/start_topic', 10)
        self.count = 0
        self.max_messages = 10
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Starting sosasat')

    def timer_callback(self):
        if self.count < self.max_messages:
            msg = String()
            msg.data = 'sosal'
            self.publisher_.publish(msg)
            self.count += 1
        else:
            self.get_logger().info('Published 10 messages, shutting down...')
            self.timer.cancel()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StringPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()