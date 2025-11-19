import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
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
import time

class CloseGripperState(smach.State):
    """Состояние: закрытие захвата"""
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['closed', 'closing', 'failure', 'preempted'],
            input_keys=['node', 'object_name', 'goal_handle'],
            output_keys=[]
        )
        self._closing_logged = False
        self.node = node
    
    def execute(self, userdata):
        object_name = userdata.object_name
        
        # Проверка на отмену
        if self.node.goal_handle.is_cancel_requested:
            return 'preempted'
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[7/9] CLOSING_GRIPPER'
        feedback_msg.status_message = f'Закрытие захвата для захвата объекта {object_name}...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Отправляем команду на закрытие захвата
        gap_msg = Float64()
        gap_msg.data = self.node.GRIPPER_CLOSED_GAP
        self.node.gripper_target_gap_publisher.publish(gap_msg)
        
        # Получаем текущее время
        start_time = self.node.get_clock().now()
        delay_duration = Duration(seconds=2)
        
        # Цикл ожидания
        while (self.node.get_clock().now() - start_time) < delay_duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        return 'closed'
        
        # Проверяем, закрылся ли захват
        # if abs(self.node.current_gripper_gap - self.node.GRIPPER_CLOSED_GAP) < self.node.GRIPPER_TOLERANCE:
        #     self.node.get_logger().info(f'✓ Захват закрыт (gap={self.node.current_gripper_gap:.4f}м), объект захвачен')
        #     return 'closed'
        # else:
        #     if not self._closing_logged:
        #         self.node.get_logger().info(
        #             f'Закрытие захвата... Текущий gap: {self.node.current_gripper_gap:.4f}м, '
        #             f'цель: {self.node.GRIPPER_CLOSED_GAP:.4f}м'
        #         )
        #         self._closing_logged = True
        #     return 'closing'
        