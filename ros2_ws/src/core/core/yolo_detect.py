# ros2 run core yolo_detect стартуем ноду

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image , LaserScan
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import numpy as np

import math
from geometry_msgs.msg import Point

 # config_ekf2= os.path.join(get_package_share_directory(package_name),'config','ekf_params2.yaml') РАБОТАЕТ ТВАРЬ ПЖ

class YoloDetect(Node):
    def __init__(self):
        super().__init__('yolo_detect')


        self.declare_parameter('input_topic','error0')
        param_camera = self.get_parameter('input_topic').get_parameter_value().string_value

        self.declare_parameter('base_frame','error1')
        

        self.declare_parameter('outtopic_img','error2')
        param_outtopic_img = self.get_parameter('outtopic_img').get_parameter_value().string_value

        self.declare_parameter('outtopic_scan','error3')
        param_outtopic_scan = self.get_parameter('outtopic_scan').get_parameter_value().string_value


        self.img_camera=[]



        self.subscription = self.create_subscription(
            Image,
            param_camera,
            self.image_saver,
            1)
        self.publisher_image = self.create_publisher(Image, param_outtopic_img, 1)
        self.publisher_scan = self.create_publisher(LaserScan, param_outtopic_scan, 1)  # Топик для LaserScan

        

        
        self.timer=self.create_timer(0.7,self.image_callback)
        # # Чтобы один было время

        # self.ts = message_filters.ApproximateTimeSynchronizer(
        #     [self.image_sub, self.lidar_sub], queue_size=10, slop=0.1)
        # self.ts.registerCallback(self.sync_callback)  

        # Отсталые Yolov8
        self.bridge = CvBridge()
        self.model = YOLO('/home/developer/robocross.dune/ros2_ws/src/core/train6/weights/best.pt')

        # Параметры камеры
        self.image_width = 424  # Ширина изображения
        self.fov_horizontal = 2.09 # Поле зрения камеры в градусах 


        self.k = 120.36 # Как определить? Запустить либо скрипт, либо дебил с рулеткой и бочкой


        

        self.get_logger().info('YOLOv8 нода запущена')
    def image_saver(self, msg):
        
        self.img_camera=msg
    def image_callback(self):
        if(self.img_camera):
            msg= self.img_camera
            param_base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            results = self.model(cv_image)

            annotated_image = cv_image.copy()
            

            # # Параметры лидара
            # angle_min = -3.14  # Минимальный угол
            # angle_max = 3.14  # Максимальный угол
            # angle_increment = 0.01  # Шаг угла
            # ranges = lidar_msg.ranges  # Массив расстояний

            scan_msg = LaserScan()
            
            scan_msg.header.frame_id = param_base_frame  # ВАНЯЯ
            scan_msg.header.stamp = self.get_clock().now().to_msg()

            scan_msg.angle_min = -self.fov_horizontal / 2.0  # Начальный угол
            scan_msg.angle_max = self.fov_horizontal / 2.0   # Конечный угол
            scan_msg.angle_increment = 0.015  # Шаг угла (можно настроить)
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 0.0
            scan_msg.range_min = 1.0
            scan_msg.range_max = 50.0


            num_points = int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment) + 1
            scan_msg.ranges = [float('nan')] * num_points  # Инициализация nan
            


            for result in results:
                boxes = result.boxes  
                for box in boxes:
                    confidence = box.conf.item()  # Уверенность 
                    if confidence < 0.50:
                        continue
                    
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()  
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)  
                    
                    
                    #Test
                    cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    class_id = int(box.cls)
                    label = f"{result.names[class_id]} {confidence:.2f}"
                    cv2.putText(annotated_image, label, (x1, y1 - 10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    
                    width = x2 - x1
                    height = y2 - y1
                    area = width * height 

                    #center_x = x1 + (width // 2)
                    #center_y = y1 + (height // 2)

                    if class_id == 0:
                        
                        # Россия вперед кто такой этот 
                        z = self.k / math.sqrt(area)
                        #self.get_logger().info('Обработал.. Проверяй площадь'+str(area))
                        # Края приколов
                        theta_left = (x1 / self.image_width - 0.5) * self.fov_horizontal
                        theta_right = (x2 / self.image_width - 0.5) * self.fov_horizontal

                        
                        # Тип объекта (ID класса)
                        #class_id = int(box.cls)  # ID класса объекта
                        
                        
                        # Маппинг углов на индексы
                        for i in range(1,num_points):

                            angle = scan_msg.angle_min + i * scan_msg.angle_increment

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

            #annotated_image = results[0].plot()  # Метод plot() делает bounding boxes
            
            processed_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            processed_image_msg.header = msg.header
            self.publisher_image.publish(processed_image_msg)
            #self.get_logger().info('Обработал.. Проверяй')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()