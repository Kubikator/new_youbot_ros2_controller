import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
import numpy as np
from yb_interfaces.action import PickupObject
from yb_interfaces.msg import BoundingBoxArray, ObjectCenter3Array, LocalizedObjectArray
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import math
import smach
import smach_ros

class NavigationManager:
    """Единый менеджер для управления навигацией"""
    
    def __init__(self, node):
        self.node = node
        self.nav_client = None
        self.current_goal_handle = None
        self._init_navigation_client()
    
    def _init_navigation_client(self):
        """Инициализация клиента навигации"""
        if self.nav_client is None:
            self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
    
    def wait_for_nav_server(self, timeout_sec=10.0):
        """Ожидание сервера навигации"""
        self._init_navigation_client()
        if self.nav_client.wait_for_server(timeout_sec=timeout_sec):
            self.node.get_logger().info('Сервер навигации доступен')
            return True
        else:
            self.node.get_logger().error('Сервер навигации недоступен!')
            return False
    
    def send_goal(self, x, y, yaw=0.0):
        """Отправка цели навигации"""
        if not self.wait_for_nav_server(timeout_sec=5.0):
            return False
            
        # Отменяем предыдущую цель если есть
        if self.current_goal_handle:
            self.cancel_goal()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Преобразуем yaw в quaternion
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if not future.done():
            self.node.get_logger().error('Таймаут отправки цели навигации')
            return False
            
        self.current_goal_handle = future.result()
        if not self.current_goal_handle.accepted:
            self.node.get_logger().error('Цель навигации отклонена')
            return False
            
        return True
    
    def wait_for_result(self, timeout_sec=30.0):
        """Ожидание результата навигации"""
        if not self.current_goal_handle:
            return False
            
        result_future = self.current_goal_handle.get_result_async()
        
        start_time = self.node.get_clock().now()
        while rclpy.ok():
            # Проверяем таймаут
            current_time = self.node.get_clock().now()
            if (current_time - start_time).nanoseconds / 1e9 > timeout_sec:
                self.node.get_logger().error('Таймаут навигации')
                self.cancel_goal()
                return False
            
            # Проверяем завершение
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if result_future.done():
                try:
                    result = result_future.result()
                    self.current_goal_handle = None
                    return True
                except Exception as e:
                    self.node.get_logger().error(f'Ошибка навигации: {e}')
                    self.current_goal_handle = None
                    return False
    
    def cancel_goal(self):
        """Отмена текущей цели"""
        if self.current_goal_handle:
            future = self.current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, future)
            self.current_goal_handle = None