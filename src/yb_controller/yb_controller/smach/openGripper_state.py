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

class OpenGripperState(smach.State):
    """Состояние: открытие захвата"""
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['opened', 'waiting', 'failure', 'preempted'],
            input_keys=['node', 'object_name', 'goal_handle'],
            output_keys=[]
        )
        self._check_count = 0
        self.node = node
    
    def execute(self, userdata):
        object_name = userdata.object_name
        
        # Проверка на отмену
        if self.node.goal_handle.is_cancel_requested:
            return 'preempted'
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[1/4] OPENING_GRIPPER'
        feedback_msg.status_message = f'Проверка видимости объекта {object_name}...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Объект виден - открываем захват
        self.node.get_logger().info(f'Открываю захват')
        gap_msg = Float64()
        gap_msg.data = self.node.GRIPPER_OPEN_GAP
        self.node.gripper_target_gap_publisher.publish(gap_msg)
        
        # Даём время на открытие
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Проверяем, открылся ли захват
        if abs(self.node.current_gripper_gap - self.node.GRIPPER_OPEN_GAP) < self.node.GRIPPER_TOLERANCE:
            self.node.get_logger().info('✓ Захват открыт')
            return 'opened'
        else:
            return 'waiting'