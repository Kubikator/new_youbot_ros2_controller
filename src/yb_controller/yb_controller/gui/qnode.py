#!/usr/bin/env python3

import os
import subprocess
import sys
import threading
from PyQt5 import QtWidgets, uic
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from rclpy.action import ActionClient
from yb_interfaces.action import PickupObject
from yb_interfaces.msg import LocalizedObjectArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from PyQt5.QtGui import QImage

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

class QNode(Node):
    def __init__(self):
        super().__init__('qnode')
        self.object_subscriber = self.create_subscription(
            LocalizedObjectArray,
            '/localized_objects_map',
            self.object_callback,
            10
        )
         # Подписка на топик с изображениями
        self.image_subscription = self.create_subscription(
            Image,
            'camera/detection_image',
            self.image_callback,
            10
        )

        self.image_1_subscription = self.create_subscription(
            Image,
            'camera_1/detection_image',
            self.image_1_callback,
            10
        )

        self.explore_success_subscriber = self.create_subscription(
            Bool,
            '/map_exploration_status',
            self.explore_success_callback,
            10
        )

        self.bridge = CvBridge()
        self.pickup_action_client = ActionClient(self, PickupObject, '/pickup_object')

        self.detected_objects = []
        self.explore_success = False
        self.curr_image = QImage()
        self.curr_image_1 = QImage()

    def object_callback(self, msg):
        """Обработчик новых обнаруженных объектов"""
        for obj in msg.objects:
            if obj.class_name not in self.detected_objects:
                self.detected_objects.append(obj.class_name)

    def start_exploration(self):
        """Запуск explorer_launch.py в отдельном процессе"""
        def run_launch():
            subprocess.run([
                'ros2', 'launch', 'yb_controller', 'explorer_launch.py'
            ], env=dict(os.environ, ROS_DOMAIN_ID='0'))
        
        threading.Thread(target=run_launch, daemon=True).start()

    def capture_object(self, object_name):
        """Упрощенная отправка action goal для захвата объекта"""
        if not self.pickup_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server /pickup_object not available')
            return False

        goal_msg = PickupObject.Goal()
        goal_msg.object_name = object_name
        
        self.get_logger().info(f'Sending pickup goal for: {object_name}')
        
        # Простая отправка без обработки результата
        future = self.pickup_action_client.send_goal_async(goal_msg)
        return True

    def image_callback(self, msg):
        """Обработчик сообщений с изображениями"""
        try:
            # Конвертируем ROS Image в OpenCV изображение
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Конвертируем OpenCV изображение в QImage
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            
            # Конвертируем BGR в RGB для QImage
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Создаем QImage
            qimage = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Делаем копию, так как данные могут быть освобождены
            self.curr_image = qimage.copy()
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def image_1_callback(self, msg):
        """Обработчик сообщений с изображениями"""
        try:
            # Конвертируем ROS Image в OpenCV изображение
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Конвертируем OpenCV изображение в QImage
            height, width, channel = cv_image.shape
            bytes_per_line = 3 * width
            
            # Конвертируем BGR в RGB для QImage
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Создаем QImage
            qimage = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Делаем копию, так как данные могут быть освобождены
            self.curr_image_1 = qimage.copy()
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def explore_success_callback(self, msg):
        
        self.explore_success = msg.data
