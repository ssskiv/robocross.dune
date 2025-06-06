# ros2 run core yolo_detect стартуем ноду

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image , LaserScan
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import message_filters
import math

 # config_ekf2= os.path.join(get_package_share_directory(package_name),'config','ekf_params2.yaml') РАБОТАЕТ ТВАРЬ ПЖ

class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')

        lidar_qos = QoSProfile(
            depth=10,  # Глубина буфера
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Наилучшие усилия
            durability=QoSDurabilityPolicy.VOLATILE,  # Нестабильная сохранность
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST  # Хранить последние сообщения
        )

        self.image_sub = message_filters.Subscriber(self, Image, '/camera/image_color')
        self.lidar_sub = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=lidar_qos)
        self.publisher_ = self.create_publisher(Image, '/yolo_detect_image', 10)
        
        
        # Yolov8

        

        # Чтобы один было время

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)  

        # Отсталые
        self.bridge = CvBridge()
        self.model = YOLO('/bmstu/ros2_ws/src/core/train/weights/best.pt')

        # Параметры камеры
        self.image_width = 640  # Ширина изображения
        self.camera_fov = 90.0  # Поле зрения камеры в градусах 

        # Масив
        self.data_pairs= []

        self.get_logger().info('YOLOv8 нода запущена')

    def sync_callback(self, image_msg, lidar_msg):
       
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        results = self.model(cv_image)

        # Параметры лидара
        angle_min = -3.14  # Минимальный угол
        angle_max = 3.14  # Максимальный угол
        angle_increment = 0.01  # Шаг угла
        ranges = lidar_msg.ranges  # Массив расстояний


        for result in results:
            boxes = result.boxes  
            for box in boxes:
                
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  # Преобразуем в целые числа
                
                width = x2 - x1
                height = y2 - y1
                area = width * height 

                center_x = x1 + (width // 2)
                center_y = y1 + (height // 2)
                
                
                # center_x (в пикселях) -> доля от ширины изображения -> угол относительно центра камеры
                fov_rad = math.radians(self.camera_fov)
                pixel_ratio = (center_x / self.image_width)  # Доля от 0 до 1
                angle_from_center = (pixel_ratio - 0.5) * fov_rad  # Угол относительно центра камеры
                
               
                lidar_angle = angle_from_center  # Предполагаем, что камера и лидар выровнены
                
               
                if angle_min <= lidar_angle <= angle_max:
                    lidar_index = int((lidar_angle - angle_min) / angle_increment)
                    if 0 <= lidar_index < len(ranges):
                        distance = ranges[lidar_index]  # Расстояние от лидара
                        if distance != float('inf') and distance != float('nan'):
                            # Сохраняем пару (площадь, расстояние)
                            self.data_pairs.append((area, distance))
                            self.get_logger().info(
                                f'Объект: площадь={area} пикселей, расстояние={distance:.2f} м'
                            )

                if self.data_pairs:  # Если список не пуст
                    with open('/bmstu/ros2_ws/src/core/data_pairs.txt', 'a') as f:  # Создаем/дописываем файл
                        for area, distance in self.data_pairs:
                            f.write(f'{area},{distance}\n')

                # class_id = int(box.cls)  # ID класса объекта
                # confidence = box.conf.item()  # Уверенность
                # self.get_logger().info(
                #     f'Объект: {class_id}, уверенность={confidence:.2f}, '
                #     f'Площадь={area} пикс*пикс, Центр=({center_x}, {center_y})'
                # )
        
        annotated_image = results[0].plot()  # Метод plot() делает bounding boxes
        
        
        processed_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        processed_image_msg.header = image_msg.header
        self.publisher_.publish(processed_image_msg)
        #self.get_logger().info('Обработал.. Проверяй')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()