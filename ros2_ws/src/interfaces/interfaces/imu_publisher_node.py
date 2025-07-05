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

class ImuPublisherNode(Node):
    
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('target_socket','ws://192.168.0.104:45123')
        self.target_socket = self.get_parameter('target_socket').value
        self.ws = None
        self.rate = self.create_rate(10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.sensor_types = ["android.sensor.accelerometer", "android.sensor.gyroscope", "android.sensor.orientation"]
        self.encoded_type = json.dumps(self.sensor_types)
        self.encoded_types = urllib.parse.quote(self.encoded_type)
        self.url = f"{self.target_socket}/sensors/connect?types={self.encoded_types}"
        self.imu_buffer = {
            'android.sensor.accelerometer': deque(maxlen=1),
            'android.sensor.gyroscope': deque(maxlen=1),
            "android.sensor.orientation": deque(maxlen=1),
        }
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

    def android_timestamp_to_ros_time(self, android_timestamp_ms):
        """Convert Android timestamp in milliseconds to rospy.Time."""
        seconds = android_timestamp_ms / 1000.0
        return self.get_clock().from_sec(seconds)

    def synchronize_data(self):
        df_accel = pd.DataFrame(list(self.imu_buffer['android.sensor.accelerometer']))
        df_gyro = pd.DataFrame(list(self.imu_buffer['android.sensor.gyroscope']))
        df_orient = pd.DataFrame(list(self.imu_buffer['android.sensor.orientation']))

        if not df_accel.empty and not df_gyro.empty:
            df_accel['timestamp'] = df_accel['timestamp'].astype(float)
            df_gyro['timestamp'] = df_gyro['timestamp'].astype(float)
            
            # df_merged = pd.merge_asof(
            #     df_gyro.sort_values('timestamp'),
            #     df_accel.sort_values('timestamp'),
            #     on='timestamp',
            #     direction='nearest',
            #     suffixes=('_gyro', '_accel')
            # )

            imu_msg = Imu()
            imu_msg.header.frame_id = "imu_link" 

            # latest_timestamp_ms = df_merged['timestamp'].iloc[-1]
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            quaternion = quaternion_from_euler(df_orient['y'].iloc[-1]/180*pi,
            df_orient['z'].iloc[-1]/180*pi,
            df_orient['x'].iloc[-1]/180*pi)
            imu_msg.orientation = Quaternion(
                x=quaternion[0],
                y=quaternion[1],
                z=quaternion[2],
                w=quaternion[3])  # Placeholder values

            # self.get_logger().warn(str(df_gyro))
            
            imu_msg.angular_velocity = Vector3(
                x=0.0,
                y=0.0,
                z=df_gyro['z'].iloc[-1]
            )
            #/180*pi
            imu_msg.linear_acceleration = Vector3(
                x=df_accel['x'].iloc[-1],
                y=df_accel['y'].iloc[-1],
                z=df_accel['z'].iloc[-1]
            )
            
            # imu_msg.angular_velocity_covariance = [0.0] * 9
            # imu_msg.linear_acceleration_covariance = [0.0] * 9
            
            try:
                self.imu_pub.publish(imu_msg)
            except self.ROSException as e:
                print(f"Error publishing IMU message: {e}")

    def on_message(self, ws, message):
        # self.get_logger().info('Got message')
        try:
            data = json.loads(message)
            sensor_type = data.get("type", "")
            accuracy = data.get("accuracy", None)
            timestamp = data.get("timestamp", None)
            values = data.get("values", [])

            if sensor_type in self.imu_buffer:
                if len(values) == 3:
                    x, y, z = values
                    self.imu_buffer[sensor_type].append({
                        'timestamp': timestamp,
                        'x': x,
                        'y': y,
                        'z': z
                    })
                    
                    self.synchronize_data()
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
    node = ImuPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


