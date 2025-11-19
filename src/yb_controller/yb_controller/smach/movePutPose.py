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

class MovingPutPoseState(smach.State):
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['reached', 'moving', 'failure', 'preempted'],
            input_keys=['object_name'],
            output_keys=[]
        )
        self._coords_warning_shown = False
        self.node = node
    
    def execute(self, userdata):
        object_name = userdata.object_name
        
        # Проверка на отмену
        if self.node.goal_handle.is_cancel_requested:
            return 'preempted'
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[5/9] MOVING_TO_PUT_POSE'
        feedback_msg.status_message = f'Движение манипулятора к точке поклажки {object_name}...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
                
        obj_x_arm = self.node.put_pose_x
        obj_y_arm = self.node.put_pose_y
        obj_z_arm = self.node.put_pose_z
        
        # Вычисляем целевую позицию
        target_x = obj_x_arm
        target_y = obj_y_arm
        target_z = obj_z_arm
        
        self.node.get_logger().info(
            f'Целевая позиция манипулятора: '
            f'x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}'
        )
        
        # Отправляем целевую позу манипулятору
        pose_msg = Point()
        pose_msg.x = target_x
        pose_msg.y = target_y
        pose_msg.z = target_z
        
        self.node.arm_target_publisher.publish(pose_msg)
        
        # Даём время на движение
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # Проверяем достижение целевой позиции
        distance = math.sqrt(
            (target_x - self.node.current_position['x'])**2 +
            (target_y - self.node.current_position['y'])**2 +
            (target_z - self.node.current_position['z'])**2
        )
        
        cur_x = self.node.current_position['x']
        cur_y = self.node.current_position['y']
        cur_z = self.node.current_position['z']

        if distance < self.node.POSITION_TOLERANCE:
            self.node.get_logger().info(f'✓ Манипулятор достиг целевой позиции (расстояние: {distance:.3f}м)')
            return 'reached'
        else:
            self.node.get_logger().info(f' Манипулятор в процессе движения (цель: {target_x:.3f}, {target_y:.3f}, {target_z:.3f}) \n'
                                        f'(Реальное положение: {cur_x:.3f}, {cur_y:.3f}, {cur_z:.3f}))')
            return 'moving'

