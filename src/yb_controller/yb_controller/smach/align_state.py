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

class AlignToObjectState(smach.State):
    """Состояние: выравнивание робота по объекту"""
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['aligned', 'aligning', 'lost', 'failure', 'preempted'],
            input_keys=['object_name'],
            output_keys=[]
        )
        self._miss_counter = 0
        self._max_misses = 10  # Если объект пропал 10 раз подряд - возврат к поиску
        self.node = node
    
    def execute(self, userdata):
        object_name = userdata.object_name
        
        # Проверка на отмену
        if self.node.goal_handle.is_cancel_requested:
            self._stop_robot(self.node)
            return 'preempted'
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[1/6] ALIGNING_TO_OBJECT'
        feedback_msg.status_message = f'Выравнивание по объекту {object_name}...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Ищем объект в detected_objects
        target_bbox = None
        for bbox in self.node.detected_objects:
            if bbox.class_name.lower() == object_name.lower():
                target_bbox = bbox
                break
        
        # Объект не найден
        if target_bbox is None:
            self._miss_counter += 1
            if self._miss_counter >= self._max_misses:
                self.node.get_logger().warn(
                    f'⚠ Объект {object_name} потерян (пропусков: {self._miss_counter}). '
                    f'Возврат к поиску'
                )
                self._stop_robot(self.node)
                self._miss_counter = 0
                return 'lost'
            
            # Продолжаем медленно вращаться
            twist_msg = Twist()
            twist_msg.angular.z = self.node.ALIGN_ANGULAR_VELOCITY
            self.node.cmd_vel_publisher.publish(twist_msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            return 'aligning'
        
        # Объект найден - сбрасываем счётчик пропусков
        self._miss_counter = 0
        
        # Вычисляем отклонение от центра
        center_x = target_bbox.center_x
        error = center_x - self.node.TARGET_CENTER_X
        
        # Проверяем выравнивание
        if abs(error) < self.node.ALIGN_TOLERANCE:
            self.node.get_logger().info(
                f'✓ Выравнивание завершено! center_x={center_x:.0f}, '
                f'отклонение={error:.0f}px'
            )
            self._stop_robot(self.node)
            return 'aligned'
        
        # Вращаем робота для выравнивания
        # Если объект слева (error < 0), крутим влево (положительная angular.z)
        # Если объект справа (error > 0), крутим вправо (отрицательная angular.z)
        angular_velocity = -self.node.ALIGN_ANGULAR_VELOCITY if error > 0 else self.node.ALIGN_ANGULAR_VELOCITY
        
        twist_msg = Twist()
        twist_msg.angular.z = angular_velocity
        self.node.cmd_vel_publisher.publish(twist_msg)
        
        self.node.get_logger().info(
            f'Выравнивание: center_x={center_x:.0f}, цель={self.node.TARGET_CENTER_X}, '
            f'ошибка={error:.0f}px'
        )
        
        rclpy.spin_once(self.node, timeout_sec=0.1)
        return 'aligning'
    
    def _stop_robot(self, node):
        """Останавливает робота"""
        twist_msg = Twist()
        node.cmd_vel_publisher.publish(twist_msg)
