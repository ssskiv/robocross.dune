# ros2 run core yolo_detect стартуем ноду

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image , LaserScan
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import numpy as np

import math
from geometry_msgs.msg import Point

 # config_ekf2= os.path.join(get_package_share_directory(package_name),'config','ekf_params2.yaml') РАБОТАЕТ ТВАРЬ ПЖ


class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')
        self.subscription = self.create_subscription(
            Image,
            '/rs1/rs1/color/image_raw',
            self.image_saver_rs1,
            10)
        
        self.subscription = self.create_subscription(
            Image,
            '/rs2/rs2/color/image_raw',
            self.image_saver_rs2,
            10)

        self.publisher_image_rs1 = self.create_publisher(Image, '/yolo_detect_image_rs1', 10)
        self.publisher_image_rs2 = self.create_publisher(Image, '/yolo_detect_image_rs2', 10)
        self.publisher_scan = self.create_publisher(LaserScan, '/yolo_object_scan', 10)  # Топик для LaserScan
        self.create_timer(0.20, self.image_callback)
    
        # # Чтобы один было время

        # self.ts = message_filters.ApproximateTimeSynchronizer(
        #     [self.image_sub, self.lidar_sub], queue_size=10, slop=0.1)
        # self.ts.registerCallback(self.sync_callback)  

        # Отсталые Yolov8
        self.bridge = CvBridge()
        self.model = YOLO('/home/developer/robocross.dune/ros2_ws/src/core/train/weights/best.pt')

        # Параметры камеры
        self.image_width = 640  # Ширина изображения
        self.fov_horizontal = 3.1453/2  # Поле зрения камеры в градусах 


        self.k = 200.0  # Как определить? Запустить либо скрипт, либо дебил с рулеткой и бочкой

        self.img_rs1 = []
        self.img_rs2 = []
        # Масив
        self.data_pairs= []

        self.get_logger().info('YOLOv8 нода запущена')
    
    def image_saver_rs1(self, msg):
        self.img_rs1 = msg
    def image_saver_rs2(self, msg):
        self.img_rs2 = msg


    def image_callback(self):
        if(self.img_rs1 and self.img_rs2):
            cv_image_rs1 = self.bridge.imgmsg_to_cv2(self.img_rs1, desired_encoding='bgr8')
        
            results_rs1 = self.model(cv_image_rs1)
        
            cv_image_rs2 = self.bridge.imgmsg_to_cv2(self.img_rs2, desired_encoding='bgr8')
            
            results_rs2 = self.model(cv_image_rs2)

            # # Параметры лидара
            # angle_min = -3.14  # Минимальный угол
            # angle_max = 3.14  # Максимальный угол
            # angle_increment = 0.01  # Шаг угла
            # ranges = lidar_msg.ranges  # Массив расстояний

            scan_msg = LaserScan()
            scan_msg.header.frame_id = "cam_link"  # ВАНЯЯ
            scan_msg.header.stamp = self.get_clock().now().to_msg()

            scan_msg.angle_min = ( -self.fov_horizontal / 2.0 )*2 # Начальный угол
            scan_msg.angle_max = ( self.fov_horizontal / 2.0 )*2    # Конечный угол
            scan_msg.angle_increment = 0.1  # Шаг угла (можно настроить)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.0
            scan_msg.range_min = 1.0
            scan_msg.range_max = 50.0


            num_points = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
            scan_msg.ranges = [float('nan')] * num_points  # Инициализация nan
            

            for result in results_rs1:
                boxes = result.boxes  
                for box in boxes:
                    
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  
                    
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height 

                    #center_x = x1 + (width // 2)
                    #center_y = y1 + (height // 2)


                    # Россия вперед кто такой этот 
                    z = self.k / math.sqrt(area)
                    
                    # Края приколов
                    theta_left = (x1 / self.image_width - 0.5) * self.fov_horizontal
                    theta_right = (x2 / self.image_width - 0.5) * self.fov_horizontal

                    
                    # Тип объекта (ID класса)
                    #class_id = int(box.cls)  # ID класса объекта
                    
                    
                    # Маппинг углов на индексы
                    for i in range(int(num_points/2)):

                        angle = 2*(scan_msg.angle_min) + i * scan_msg.angle_increment

                        if abs(angle - theta_left) < scan_msg.angle_increment / 2.0:
                            scan_msg.ranges[int(num_points/2) - i] = z
                            
                        elif abs(angle - theta_right) < scan_msg.angle_increment / 2.0:
                            scan_msg.ranges[int(num_points/2) - i] = z




            for result in results_rs2:
                boxes = result.boxes  
                for box in boxes:
                    
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  
                    
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height 

                    #center_x = x1 + (width // 2)
                    #center_y = y1 + (height // 2)


                    # Россия вперед кто такой этот 
                    z = self.k / math.sqrt(area)
                    
                    # Края приколов
                    theta_left = (x1 / self.image_width - 0.5) * self.fov_horizontal
                    theta_right = (x2 / self.image_width - 0.5) * self.fov_horizontal

                    
                    # Тип объекта (ID класса)
                    #class_id = int(box.cls)  # ID класса объекта
                    
                    
                    # Маппинг углов на индексы
                    for i in range(int(num_points/2),num_points):

                        angle = 2*(scan_msg.angle_min) + i * scan_msg.angle_increment

                        if abs(angle - theta_left) < scan_msg.angle_increment / 2.0:
                            scan_msg.ranges[num_points - i] = z
                            
                        elif abs(angle - theta_right) < scan_msg.angle_increment / 2.0:
                            scan_msg.ranges[num_points - i] = z
                        
                        
                    
                    
                    # self.get_logger().info(
                    #     f'Объект: тип={class_id}, площадь={area} пикселей, расстояние={z:.2f} м, '
                    #     f'углы (left, right)=({theta_left:.2f}, {theta_right:.2f} рад'
                    
                    # )
                    
                    # # Если нужно сохранить пары (площадь, расстояние) в файл

                    # # center_x (в пикселях) -> доля от ширины изображения -> угол относительно центра камеры
                    # fov_rad = math.radians(self.camera_fov)
                    # pixel_ratio = (center_x / self.image_width)  # Доля от 0 до 1
                    # angle_from_center = (pixel_ratio - 0.5) * fov_rad  # Угол относительно центра камеры
                    
                
                    # lidar_angle = angle_from_center  # Предполагаем, что камера и лидар выровнены
                    
                
                    # if angle_min <= lidar_angle <= angle_max:
                    #     lidar_index = int((lidar_angle - angle_min) / angle_increment)
                    #     if 0 <= lidar_index < len(ranges):
                    #         distance = ranges[lidar_index]  # Расстояние от лидара
                    #         if distance != float('inf') and distance != float('nan'):
                    #             # Сохраняем пару (площадь, расстояние)
                    #             self.data_pairs.append((area, distance))
                    #             self.get_logger().info(
                    #                 f'Объект: площадь={area} пикселей, расстояние={distance:.2f} м'
                    #             )

                    # if self.data_pairs:  # Если список не пуст
                    #     with open('/bmstu/ros2_ws/src/core/data_pairs.txt', 'a') as f:  # Создаем/дописываем файл
                    #         for area, distance in self.data_pairs:
                    #             f.write(f'{area},{distance}\n')

                    # class_id = int(box.cls)  # ID класса объекта
                    # confidence = box.conf.item()  # Уверенность
                    # self.get_logger().info(
                    #     f'Объект: {class_id}, уверенность={confidence:.2f}, '
                    #     f'Площадь={area} пикс*пикс, Центр=({center_x}, {center_y})'
                    # )
            
        
            self.publisher_scan.publish(scan_msg)

            annotated_image_rs1 = results_rs1[0].plot()  # Метод plot() делает bounding boxes 
            processed_image_msg_rs1 = self.bridge.cv2_to_imgmsg(annotated_image_rs1, encoding='bgr8')
            processed_image_msg_rs1.header = self.img_rs1.header
            self.publisher_image_rs1.publish(processed_image_msg_rs1)

            annotated_image_rs2 = results_rs2[0].plot()  # Метод plot() делает bounding boxes 
            processed_image_msg_rs2 = self.bridge.cv2_to_imgmsg(annotated_image_rs2, encoding='bgr8')
            processed_image_msg_rs2.header = self.img_rs2.header
            self.publisher_image_rs2.publish(processed_image_msg_rs2)

            #self.get_logger().info('Обработал.. Проверяй')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()