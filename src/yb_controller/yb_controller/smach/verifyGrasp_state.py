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

class VerifyGraspState(smach.State):
    
    def __init__(self, node):
        smach.State.__init__(
            self,
            outcomes=['grasped', 'not_grasped', 'preempted'],
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
        feedback_msg.current_state = '[9/9] VERIFY_GRASP'
        feedback_msg.status_message = f'Верификация захвата {object_name}...'
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
            return 'not_grasped'
        else:
            return 'grasped'
        