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

class LiftObjectState(smach.State):
    """Состояние: подъём объекта"""
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['lifted', 'lifting', 'failure', 'preempted'],
            input_keys=['node', 'object_name', 'goal_handle'],
            output_keys=[]
        )
        self._lifting_logged = False
        self.node = node
    
    def execute(self, userdata):
        object_name = userdata.object_name
        
        # Проверка на отмену
        if self.node.goal_handle.is_cancel_requested:
            return 'preempted'
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[7/8] LIFTING_OBJECT'
        feedback_msg.status_message = f'Подъём объекта {object_name}...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Поднимаем объект на безопасную высоту
        lift_height = 0.3
        
        # Отправляем команду на подъём
        pose_msg = Point()
        pose_msg.x = 0.1
        pose_msg.y = 0.0
        pose_msg.z = lift_height
        
        self.node.arm_target_publisher.publish(pose_msg)
        
        # Даём время на подъём
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Проверяем достижение высоты
        if abs(self.node.current_position['z'] - lift_height) < self.node.POSITION_TOLERANCE:
            self.node.get_logger().info(f'✓ Объект поднят на высоту {lift_height:.3f}м')
            return 'lifted'
        else:
            if not self._lifting_logged:
                self.node.get_logger().info(
                    f'Подъём объекта... Текущая высота: {self.node.current_position["z"]:.3f}м, '
                    f'цель: {lift_height:.3f}м'
                )
                self._lifting_logged = True
            return 'lifting'