# imu_publisher.py
import rclpy
from rclpy.node import Node
import urllib.parse
import rclpy.node
from collections import deque
import pandas as pd
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
import time
import threading
import signal
import sys
from zeroconf import ServiceBrowser, ServiceListener, Zeroconf
import websocket
import json
import socket
from math import pi
from tf_transformations import quaternion_from_euler

class GPSPublisherNode(Node):
    
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('target_socket','ws://192.168.0.104:45123')
        self.target_socket = self.get_parameter('target_socket').value
        self.ws = None
        self.rate = self.create_rate(10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.url = f"{self.target_socket}/gps"
        self.ws = websocket.WebSocketApp(self.url,
                              on_open=self.on_open,
                              on_message=self.on_message,
                              on_error=self.on_error,
                              on_close=self.on_close)

        self.ws.run_forever()

    def on_error(self, ws, error):
        print(f"Error occurred: {error}")

    def on_close(self, ws, close_code, reason):
        self.rate.sleep()
        self.connect()

    def on_open(self, ws):
        print("Connected to the WebSocket server")

    def on_message(self, ws, message):
        # self.get_logger().info('Got message')
        try:
            data = json.loads(message)
            sensor_type = data.get("type", "")
            accuracy = data.get("accuracy", None)
            timestamp = data.get("timestamp", None)
            values = data.get("values", [])

            self.get_logger().info(str(data))
                    
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON: {e}")

    def connect(self):
        self.ws = websocket.WebSocketApp(
            self.url,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        self.ws.run_forever()

    def shutdown(self):
        if self.ws:
            self.ws.close()
        self.shutdown("Shutting down due to interrupt")


def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


