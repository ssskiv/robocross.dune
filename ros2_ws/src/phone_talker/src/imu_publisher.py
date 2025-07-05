#!/usr/bin/env python3
# indicator_node.py
import rclpy
from rclpy.node import Node
import urllib.parse
import rclpy.node
import websocket
import json
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


class IMUpublisherNode(Node):
    
    def __init__(self):
        super().__init__('imu_node')
        # self.declare_parameter('target_socket','ws://localhost:8085')
        self.ws = None
        self.imu_pub = self.create_publisher('/imu', Imu, 10)
        self.sensor_types = ["android.sensor.accelerometer", "android.sensor.gyroscope"]
        self.encoded_type = json.dumps(self.sensor_types)
        self.encoded_types = urllib.parse.quote(self.encoded_type)
        # self.url = f"{target_socket}/sensors/connect?types={self.encoded_types}"


        self.imu_buffer = {
            'android.sensor.accelerometer': deque(maxlen=1000),
            'android.sensor.gyroscope': deque(maxlen=1000)
        }

        self.rate = self.create_rate(10)

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
        return rclpy.time.from_sec(seconds)

    def synchronize_data(self):
        df_accel = pd.DataFrame(list(self.imu_buffer['android.sensor.accelerometer']))
        df_gyro = pd.DataFrame(list(self.imu_buffer['android.sensor.gyroscope']))

        if not df_accel.empty and not df_gyro.empty:
            df_accel['timestamp'] = df_accel['timestamp'].astype(float)
            df_gyro['timestamp'] = df_gyro['timestamp'].astype(float)
            
            df_merged = pd.merge_asof(
                df_gyro.sort_values('timestamp'),
                df_accel.sort_values('timestamp'),
                on='timestamp',
                direction='nearest',
                suffixes=('_gyro', '_accel')
            )

            imu_msg = Imu()
            imu_msg.header.frame_id = "imu_link" 

            latest_timestamp_ms = df_merged['timestamp'].iloc[-1]
            imu_msg.header.stamp = self.time.now()
            
            imu_msg.orientation = Quaternion(0, 0, 0, 1)  # Placeholder values
            
            imu_msg.angular_velocity = Vector3(
                df_merged['x_gyro'].iloc[-1],
                df_merged['y_gyro'].iloc[-1],
                df_merged['z_gyro'].iloc[-1]
            )
            
            imu_msg.linear_acceleration = Vector3(
                df_merged['x_accel'].iloc[-1],
                df_merged['y_accel'].iloc[-1],
                df_merged['z_accel'].iloc[-1]
            )
            
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            
            try:
                self.imu_pub.publish(imu_msg)
            except self.ROSException as e:
                print(f"Error publishing IMU message: {e}")

    def on_message(self, ws, message):
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
    node = IMUpublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



class MyServiceListener(ServiceListener):

    def update_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        print(f"Service {name} updated")

    def remove_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        print(f"Service {name} removed")

    def add_service(self, zc: Zeroconf, type_: str, name: str) -> None:
        info = zc.get_service_info(type_, name)
        if info:
            addresses = [socket.inet_ntoa(addr) for addr in info.addresses]
            print(f"\nService Added:")
            print(f"  Name: {name}")
            print(f"  Addresses: {addresses}")
            print(f"  Port: {info.port}")

            if len(addresses) != 0:
                address = addresses[0]
                portNo = info.port
                print("connecting...")
                # print(f"ws://{address}:{portNo}/sensor/connect?type=android.sensor.accelerometer")
                connect(f'ws://{address}:{portNo}/gps')
                # connect(f'ws://{address}:{portNo}/sensors/connect?types=["android.sensor.accelerometer","android.sensor.gyroscope","android.sensor.light","android.sensor.magnetic_field"]')
