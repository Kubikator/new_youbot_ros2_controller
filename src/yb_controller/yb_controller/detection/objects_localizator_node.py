#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from yb_interfaces.msg import BoundingBox, BoundingBoxArray, LocalizedObjectArray, LocalizedObject
from tf2_ros import Buffer, TransformListener
import math


class DetectedObject:
    """Класс для хранения информации об обнаруженном объекте"""
    def __init__(self, class_name, x, y, yaw, bbox_area, confidence):
        self.class_name = class_name
        self.x = x  # Глобальная координата X на карте
        self.y = y  # Глобальная координата Y на карте
        self.yaw = yaw  # Угол относительно робота
        self.bbox_area = bbox_area  # Площадь bounding box
        self.confidence = confidence
        self.last_seen_time = None
    
    def update(self, x, y, yaw, bbox_area, confidence):
        """Обновить информацию об объекте если новый bbox больше"""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.bbox_area = bbox_area
        self.confidence = confidence

class ObjectsFinder(Node):
    def __init__(self):
        super().__init__('objects_finder')
        
        # TF2 для получения трансформаций
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Хранилище обнаруженных объектов
        # Ключ: class_name, Значение: DetectedObject
        self.detected_objects = {}
        
        # Параметры для определения "того же" объекта
        self.declare_parameter('distance_threshold', 0.5)  # метры
        self.declare_parameter('update_area_threshold', 1.1)  # Коэффициент увеличения площади
        
        # Подписка на обнаруженные объекты
        self.bbox_subscriber = self.create_subscription(
            BoundingBoxArray,
            '/camera/detected_objects',
            self.bbox_callback,
            10
        )

        self.objects_publisher = self.create_publisher(
            LocalizedObjectArray,
            '/localized_objects_map',
            10
        )

        self.publish_timer = self.create_timer(0.1, self.publish_objects)
        
        self.get_logger().info('Objects Finder Node started!')
        self.get_logger().info('Listening to /camera/detected_objects')
    
    def bbox_callback(self, msg):
        """
        Callback при получении новых обнаруженных объектов
        
        Args:
            msg (BoundingBoxArray): Массив обнаруженных объектов
        """
        robot_pose_map = self.get_robot_pose_in_map()
        
        if not robot_pose_map:
            self.get_logger().warn('Could not get robot pose in map frame. Skipping...')
            return
        
        current_time = self.get_clock().now()
        global_x = robot_pose_map['x']
        global_y = robot_pose_map['y']
        global_yaw = robot_pose_map['yaw']

        for bbox in msg.boxes:
            class_name = bbox.class_name
            object_key = self.find_existing_object(class_name)
            bbox_area = bbox.size_x * bbox.size_y

            if object_key:
                existing_obj = self.detected_objects[object_key]
                area_threshold = self.get_parameter('update_area_threshold').value
                if bbox_area > existing_obj.bbox_area * area_threshold:
                    # Обновляем информацию
                    existing_obj.update(global_x, global_y, global_yaw, bbox_area, bbox.confidence)
                    existing_obj.last_seen_time = current_time

                    self.get_logger().info(
                        f'Updated {class_name}: pos=({global_x:.2f}, {global_y:.2f}), '
                        f'area={bbox_area:.2f} (was {existing_obj.bbox_area:.2f})'
                    )

                else:
                    # Просто обновляем время последнего обнаружения
                    existing_obj.last_seen_time = current_time
                    
            else:
                new_obj = DetectedObject(
                    class_name, robot_pose_map['x'], robot_pose_map['y'], robot_pose_map['yaw'], bbox_area, bbox.confidence
                )
                new_obj.last_seen_time = current_time

                self.detected_objects[class_name] = new_obj

                self.get_logger().info(
                    f'New object detected: {class_name} at ({global_x:.2f}, {global_y:.2f}), '
                    f'area={bbox_area:.2f}, confidence={bbox.confidence:.2f}'
                )
                

    def find_existing_object(self, class_name):
        
        for key, obj in self.detected_objects.items():
            # Проверяем только объекты того же класса
            if obj.class_name == class_name:
                return key
        
        return None
    
    def get_robot_pose_in_map(self):
        """
        Получить текущую позицию робота в системе координат map через TF
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float, 'yaw': float} или None
        """
        try:
            # Получаем трансформацию от map до base_link
            transform = self.tf_buffer.lookup_transform(
                'map',           # target frame
                'base_link',     # source frame
                rclpy.time.Time(), # latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Извлекаем позицию
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            
            # Извлекаем ориентацию (quaternion -> yaw)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Преобразуем quaternion в yaw (угол поворота вокруг оси Z)
            yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                           1.0 - 2.0 * (qy * qy + qz * qz))
            
            return {
                'x': x,
                'y': y,
                'z': z,
                'yaw': yaw,
                'quaternion': {'x': qx, 'y': qy, 'z': qz, 'w': qw}
            }
            
        except Exception as e:
            self.get_logger().warn(f'Could not get transform map->base_link: {e}')
            return None
        
    def publish_objects(self):
        """Публикация всех обнаруженных объектов каждые 100 мс"""
        if len(self.detected_objects) == 0:
            return
        
        array = LocalizedObjectArray()

        for i, (key, obj) in enumerate(self.detected_objects.items()):
            object = LocalizedObject()
            object.class_name = obj.class_name
            object.area = obj.bbox_area
            object.x = obj.x
            object.y = obj.y
            object.yaw = obj.yaw

            array.objects.append(object)

        self.objects_publisher.publish(array)
        
def main(args=None):
    rclpy.init(args=args)
    objects_finder = ObjectsFinder()
    
    try:
        rclpy.spin(objects_finder)
    except KeyboardInterrupt:
        objects_finder.get_logger().info('Objects Finder stopped by user')
    finally:
        objects_finder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()