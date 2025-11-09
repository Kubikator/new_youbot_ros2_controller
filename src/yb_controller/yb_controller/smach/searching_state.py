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

class SearchObjectState(smach.State):
    """Состояние: поиск объекта (вращение на месте)"""
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['found', 'searching', 'failure', 'preempted'],
            input_keys=['object_name'],
            output_keys=[]
        )

        self.node = node
    
    def execute(self, userdata):
        object_name = userdata.object_name
        
        # Проверка на отмену
        if self.node.goal_handle.is_cancel_requested:
            # Останавливаем робота
            self._stop_robot(self.node)
            return 'preempted'
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[0/5] SEARCHING_OBJECT'
        feedback_msg.status_message = f'Поиск объекта {object_name}...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Проверяем, виден ли объект
        object_visible = False
        for bbox in self.node.detected_objects:
            if bbox.class_name.lower() == object_name.lower():
                self.node.get_logger().info(
                    f'✓ Объект "{object_name}" найден! Уверенность: {bbox.confidence:.2f}'
                )
                object_visible = True
                break
        
        if object_visible:
            # Объект найден - останавливаем робота
            self._stop_robot(self.node)
            self.node.get_logger().info('✓ Поиск завершён успешно')
            return 'found'
        
        # Крутим робота на месте
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = self.node.SEARCH_ANGULAR_VELOCITY  # Вращение против часовой стрелки
        
        self.node.cmd_vel_publisher.publish(twist_msg)
        
        # Даём время на обработку
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        return 'searching'
    
    def _stop_robot(self, node):
        """Останавливает робота"""
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        node.cmd_vel_publisher.publish(twist_msg)
