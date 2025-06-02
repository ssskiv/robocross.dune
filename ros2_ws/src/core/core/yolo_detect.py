# ros2 run core yolo_detect стартуем ноду

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

class WebotsYoloProcessor(Node):
    def __init__(self):
        super().__init__('webots_yolo_processor')

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_color',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Image, '/yolo_detect_image', 10)
        self.bridge = CvBridge()
        
        # Yolov8

        self.model = YOLO('/bmstu/ros2_ws/src/core/train/weights/best.pt')  
        self.get_logger().info('YOLOv8 модель загружена, нода запущена')

    def image_callback(self, msg):
       
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        results = self.model(cv_image)
        
       
        annotated_image = results[0].plot()  # Метод plot() делает bounding boxes
        
        
        processed_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        processed_image_msg.header = msg.header
        
        
        self.publisher_.publish(processed_image_msg)
        self.get_logger().info('Изображение обработано YOLOv8')

def main(args=None):
    rclpy.init(args=args)
    node = WebotsYoloProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()