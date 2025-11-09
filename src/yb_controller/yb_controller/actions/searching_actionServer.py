#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String

from yb_interfaces.action import SearchObject
from yb_interfaces.msg import BoundingBoxArray, BoundingBox, ObjectCenter3, ObjectCenter3Array

import math
import time


class SearchObjectActionServer(Node):
    
    def __init__(self):
        super().__init__('search_object_action_server')
        
        # Параметры
        self.declare_parameter('angular_speed', 0.3)  # рад/с
        self.declare_parameter('detection_topic', '/camera/detected_objects')
        self.declare_parameter('full_rotation_angle', 2 * math.pi)
        
        self.angular_speed = self.get_parameter('angular_speed').value
        self.full_rotation = self.get_parameter('full_rotation_angle').value
        
        # Callback group для параллельной обработки
        self.cb_group = ReentrantCallbackGroup()
        
        # Action Server
        self._action_server = ActionServer(
            self,
            SearchObject,
            'search_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscribers
        self.detection_camera_subscriber = self.create_subscription(
            BoundingBoxArray,
            self.get_parameter('detection_topic').value,
            self.detection_callback,
            10,
            callback_group=self.cb_group
        )

        # Состояние
        self.detected_objects = []
        self.target_found = False
        self.target_object_type = None
        
        self.get_logger().info('Search Object Action Server started')
    
    def goal_callback(self, goal_request):
        """Принять или отклонить новую цель"""
        self.get_logger().info(f'Received goal request for object: {goal_request.object_type}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Обработка запроса на отмену"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def detection_callback(self, msg):
        """Обработка сообщений от системы детекции"""
        self.detected_objects = msg.boxes
    
    def stop_robot(self):
        """Остановка робота"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def rotate_robot(self, angular_velocity):
        """Вращение робота с заданной угловой скоростью"""
        twist = Twist()
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)
    
    def check_for_target(self, target_type):
        """
        Проверить, есть ли целевой объект в списке детекций
        Returns: (found, object_data)
        """
        for obj in self.detected_objects:
            if obj.class_name == target_type:
                self.get_logger().info(f'Target object "{target_type}" detected!')
                return True, obj
        return False, None
    
    async def execute_callback(self, goal_handle):

        self.get_logger().info('Executing search...')
        
        # Получаем параметры цели
        target_type = goal_handle.request.object_type
        
        # Сброс состояния
        self.target_found = False
        self.detected_objects = []
        
        # Feedback сообщение
        feedback_msg = SearchObject.Feedback()
        
        # Переменные для отслеживания
        rate = self.create_rate(10)  # 10 Hz
        
        try:
            while rclpy.ok():
                # Проверка на отмену
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.stop_robot()
                    self.get_logger().info('Goal canceled')
                    return SearchObject.Result(success=False)
                
                # Проверка: найден ли объект
                found, _ = self.check_for_target(target_type)
                if found:
                    self.stop_robot()
                    self.target_found = True
                    
                    # Возвращаем успешный результат
                    result = SearchObject.Result()
                    result.success = True
                    
                    goal_handle.succeed()
                    self.get_logger().info('Search succeeded!')
                    return result
                
                # Продолжаем вращение
                self.rotate_robot(self.angular_speed)
                
                # Публикуем feedback
                feedback_msg.objects_in_view = len(self.detected_objects)
                goal_handle.publish_feedback(feedback_msg)
                
                # Ждём следующий цикл
                rate.sleep()
            
            # Если вышли из цикла без нахождения объекта
            self.stop_robot()
            result = SearchObject.Result()
            result.success = False
            
            goal_handle.abort()
            self.get_logger().info('Search failed - object not found')
            return result
            
        except Exception as e:
            self.get_logger().error(f'Error during search: {str(e)}')
            self.stop_robot()
            goal_handle.abort()
            return SearchObject.Result(success=False)


def main(args=None):
    rclpy.init(args=args)
    
    search_server = SearchObjectActionServer()
    
    # Используем многопоточный executor для параллельной обработки callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(search_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        search_server.stop_robot()
        search_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()