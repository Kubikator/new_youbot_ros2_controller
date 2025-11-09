#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, TransformStamped
from yb_interfaces.msg import BoundingBoxArray, BoundingBox, ObjectCenter3, ObjectCenter3Array
from cv_bridge import CvBridge
import numpy as np
import cv2
import math
from typing import Tuple, Optional
from tf2_ros import TransformListener, Buffer
from yb_interfaces.msg import BoundingBoxArray, BoundingBox, ObjectCenter3, ObjectCenter3Array
from yb_controller.triangulation.triangulator import Triangulation


class CoordinateFinder(Node):
    def __init__(self):
        super().__init__('coordinate_finder')

        width = 480
        height = 480
        fov = 1

        self.cx = width/2
        self.cy = height/2
        self.fx = (width/2) / (math.tan(fov/2))
        self.fy = self.fx
        
        # Bridge для конвертации изображений
        self.bridge = CvBridge()
        
        self.boxs_camera = None
        self.boxs_camera_1 = None

        # TF buffer и listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.detection_camera_subscriber = self.create_subscription(
            BoundingBoxArray,
            '/camera/detected_objects',
            self.detection_camera_callback,
            10
        )

        self.detection_camera_1_subscriber = self.create_subscription(
            BoundingBoxArray,
            '/camera_1/detected_objects',
            self.detection_camera_1_callback,
            10
        )
        
        # Публикация координат объектов
        self.points_publisher = self.create_publisher(
            ObjectCenter3Array,
            '/objects_centers',
            10
        )

        # Таймер для обработки данных
        self.process_timer = self.create_timer(0.1, self.process_objects)  # 10 Hz
        
        self.get_logger().info('Object Coordinate Finder initialized.')
    
    def detection_camera_callback(self, msg):
        self.boxs_camera = msg

    def detection_camera_1_callback(self, msg):
        self.boxs_camera_1 = msg
    
    def process_objects(self):
        # Создаем сообщение для результатов
        result_msg = ObjectCenter3Array()
        result_msg.header.stamp = self.get_clock().now().to_msg()
        result_msg.header.frame_id = 'base_link'
        
        # Проверяем наличие данных с обеих камер
        if self.boxs_camera is None or self.boxs_camera_1 is None:
            # Публикуем пустой топик
            self.points_publisher.publish(result_msg)
            return
        
        try:
            # Создаем словари для группировки объектов по классам
            camera_objects = {}  # {class_name: BoundingBox}
            camera_1_objects = {}  # {class_name: BoundingBox}
            
            # Собираем объекты с первой камеры
            for bbox in self.boxs_camera.boxes:
                if bbox.class_name not in camera_objects:
                    camera_objects[bbox.class_name] = []
                camera_objects[bbox.class_name].append(bbox)
            
            # Собираем объекты со второй камеры
            for bbox in self.boxs_camera_1.boxes:
                if bbox.class_name not in camera_1_objects:
                    camera_1_objects[bbox.class_name] = []
                camera_1_objects[bbox.class_name].append(bbox)
            
            # Находим общие классы
            common_classes = set(camera_objects.keys()) & set(camera_1_objects.keys())
            
            if not common_classes:
                # Публикуем пустой топик
                self.points_publisher.publish(result_msg)
                return
            
            # Получаем позы камер
            camera_pose = self.get_camera_pose('camera_link', 'base_link')
            camera_1_pose = self.get_camera_pose('camera_1_link', 'base_link')
            
            if camera_pose is None or camera_1_pose is None:
                self.get_logger().warn("Could not get camera poses, skipping triangulation")
                # Публикуем пустой топик
                self.points_publisher.publish(result_msg)
                return
            
            # Применяем коррекцию системы координат
            R_camera, t_camera = self.apply_coordinate_correction(*camera_pose)
            R_camera_1, t_camera_1 = self.apply_coordinate_correction(*camera_1_pose)
            
            # Инициализируем триангулятор
            camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])
            triangulator = Triangulation(camera_matrix)
            
            # Обрабатываем каждый общий класс
            for class_name in common_classes:
                # Берем первый объект каждого класса (можно усложнить логику)
                bbox_camera = camera_objects[class_name][0]
                bbox_camera_1 = camera_1_objects[class_name][0]
                
                # Получаем координаты центров на изображениях
                point_camera = np.array([bbox_camera.center_x, bbox_camera.center_y])
                point_camera_1 = np.array([bbox_camera_1.center_x, bbox_camera_1.center_y])
                
                # Выполняем триангуляцию
                object_3d = triangulator.triangulate_single_object(
                    (R_camera, t_camera),
                    (R_camera_1, t_camera_1),
                    point_camera,
                    point_camera_1
                )
                
                # Создаем сообщение для объекта
                obj_center = ObjectCenter3()
                obj_center.center.x = float(object_3d[0])
                obj_center.center.y = float(object_3d[1])
                obj_center.center.z = float(object_3d[2])
                obj_center.class_name = class_name
                
                result_msg.centers.append(obj_center)
            
            # Публикуем результаты (пустой или с данными)
            self.points_publisher.publish(result_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing objects: {e}')
            import traceback
            traceback.print_exc()
            # В случае ошибки публикуем пустой топик
            self.points_publisher.publish(result_msg)

    def get_camera_pose(self, camera_frame: str, target_frame: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:

        try:
            # Получаем трансформацию (используем самое актуальное время)
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Извлекаем трансляцию
            t = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            
            # Извлекаем ротацию (кватернион в матрицу поворота)
            q = transform.transform.rotation
            R = self.quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
            
            return R, t
            
        except Exception as e:
            self.get_logger().warn(f"Transform exception for {camera_frame}: {e}")
            return None
        
    def quaternion_to_rotation_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Конвертируем кватернион в матрицу поворота 3x3"""
        # Нормализуем кватернион
        norm = np.sqrt(x*x + y*y + z*z + w*w)
        if norm == 0:
            return np.eye(3)
        x, y, z, w = x/norm, y/norm, z/norm, w/norm
        
        # Вычисляем матрицу поворота
        R = np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [2*x*y + 2*z*w,     1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [2*x*z - 2*y*w,     2*y*z + 2*x*w,     1 - 2*x*x - 2*y*y]
        ])
        
        return R
    
    def apply_coordinate_correction(self, R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        применяем коррекцию системы координат для OpenCV
        Сначала поворот вокруг исходной оси Y на +90°, 
        затем поворот вокруг НОВОЙ оси Z на -90°
        """
        # Создаем матрицы поворотов
        # Поворот вокруг исходной оси Y на +90°
        Ry_90 = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        
        # Поворот вокруг оси Z на -90°
        # В новой системе после первого поворота
        Rz_minus_90_new = np.array([
            [0, 1, 0],
            [-1, 0, 0],
            [0, 0, 1]
        ])
        
        # Правильная композиция: сначала Ry, потом Rz в новой системе
        # Это эквивалентно R_total = Ry_90 @ Rz_minus_90_new
        correction_matrix = Ry_90 @ Rz_minus_90_new
        
        # Применяем коррекцию
        R_corrected = (R @ correction_matrix).T
        t_corrected = -R_corrected @ t
        
        return R_corrected, t_corrected


def main(args=None):
    rclpy.init(args=args)
    node = CoordinateFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()