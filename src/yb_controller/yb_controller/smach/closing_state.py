import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
import numpy as np
from yb_interfaces.action import PickupObject
from yb_interfaces.msg import BoundingBoxArray
from rclpy.action import ActionServer
import math
import smach
import smach_ros
from geometry_msgs.msg import Pose, Point, Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

class ClosingState(smach.State):
    """Состояние: сближение с целевым объектом до расстояния ~0.4м в системе mnp_link"""
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['closed', 'closing', 'lost', 'align', 'failure', 'preempted'],
            input_keys=['object_name'],
            output_keys=[]
        )
        self.node = node
        self._miss_counter = 0
        self._max_misses = 30  # Максимальное число кадров без объекта
        self._consecutive_success_frames = 0
        self._required_success_frames = 5  # Требуется N кадров на целевом расстоянии
        
        # Параметры управления
        self.target_distance = 0.35  # Целевое расстояние в системе mnp_link (метры)
        self.distance_tolerance = 0.05  # Допуск расстояния
        self.kp = 0.5  # Коэффициент P-регулятора
        self.min_speed = 0.01  # Минимальная скорость
        self.max_speed = 0.1  # Максимальная скорость
    
    def execute(self, userdata):
        object_name = userdata.object_name
        
        # Проверка на отмену
        if self.node.goal_handle.is_cancel_requested:
            self._stop_robot()
            return 'preempted'
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[3/9] APPROACHING_OBJECT'
        feedback_msg.status_message = f'Сближение с объектом {object_name}...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Ищем объект в objects_centers
        object_found = False
        obj_x_base = 0.0
        obj_y_base = 0.0
        obj_z_base = 0.0
        
        for center in self.node.objects_centers:
            if center.class_name.lower() == object_name.lower():
                obj_x_base = center.center.x
                obj_y_base = center.center.y
                obj_z_base = center.center.z
                object_found = True
                break
        
        # Объект не найден
        if not object_found:
            self._miss_counter += 1
            if self._miss_counter >= self._max_misses:
                self.node.get_logger().warn(
                    f'⚠ Объект {object_name} потерян (пропусков: {self._miss_counter})'
                )
                self._stop_robot()
                self._miss_counter = 0
                return 'lost'
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            return 'closing'
        
        # Объект найден - сбрасываем счётчик пропусков
        self._miss_counter = 0
        
        try:
            # Трансформируем координаты из base_link в mnp_link
            obj_x_mnp = obj_x_base - self.node.ARM_LINK_OFFSET['x']
            obj_y_mnp = obj_y_base - self.node.ARM_LINK_OFFSET['y']
            obj_z_mnp = obj_z_base - self.node.ARM_LINK_OFFSET['z']
            
            # Текущее расстояние до объекта в системе mnp_link (координата X)
            current_distance = obj_x_mnp
            
            # Вычисляем ошибку расстояния
            distance_error = current_distance - self.target_distance
            
            self.node.get_logger().info(
                f'Сближение: расстояние={current_distance:.3f}м, '
                f'цель={self.target_distance}м, ошибка={distance_error:.3f}м, '
                f'Y={obj_y_mnp:.3f}м'
            )
            
            # Проверяем достижение целевого расстояния
            if abs(distance_error) <= self.distance_tolerance:
                self._consecutive_success_frames += 1
                
                self.node.get_logger().info(
                    f'Целевое расстояние! ({self._consecutive_success_frames}/{self._required_success_frames})'
                )
                
                if self._consecutive_success_frames >= self._required_success_frames:
                    self.node.get_logger().info(
                        f'✓ Сближение завершено! Расстояние: {current_distance:.3f}м'
                    )
                    self._stop_robot()
                    self._consecutive_success_frames = 0
                    return 'closed'
                
                self._stop_robot()
                rclpy.spin_once(self.node, timeout_sec=0.1)
                return 'closing'
            
            # Сбрасываем счетчик успешных кадров при движении
            self._consecutive_success_frames = 0
            
            # Проверка на потерю выравнивания (координата Y в mnp_link)
            if abs(obj_y_mnp) > 0.2:
                self.node.get_logger().warn(
                    f'⚠ Потеряно выравнивание! Y={obj_y_mnp:.3f}м'
                )
                self._stop_robot()
                return 'align'
            
            # Проверка: объект слишком близко?
            if distance_error < -0.1:  # На 10см ближе чем нужно
                self.node.get_logger().warn(
                    f'⚠ Слишком близко! Расстояние: {current_distance:.3f}м, требуется отъезд'
                )
                self._stop_robot()
                return 'failure'
            
            # Управление движением с помощью P-регулятора
            linear_velocity = self.kp * distance_error
            
            # Обеспечиваем минимальную скорость для преодоления трения
            if linear_velocity > 0 and linear_velocity < self.min_speed:
                linear_velocity = self.min_speed
            
            # Ограничиваем максимальную скорость
            linear_velocity = max(-self.max_speed, min(self.max_speed, linear_velocity))
            
            # Если слишком медленно движемся к цели, увеличиваем скорость
            if 0 < linear_velocity < self.min_speed:
                linear_velocity = self.min_speed
            
            # Публикуем команду движения
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            self.node.cmd_vel_publisher.publish(twist_msg)
            
            rclpy.spin_once(self.node, timeout_sec=0.1)
            return 'closing'
            
        except Exception as e:
            self.node.get_logger().error(f'Ошибка при сближении: {e}')
            self._stop_robot()
            return 'failure'
    
    def _stop_robot(self):
        """Останавливает робота"""
        twist_msg = Twist()
        self.node.cmd_vel_publisher.publish(twist_msg)
