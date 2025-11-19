import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from yb_interfaces.action import PickupObject
import smach
import smach_ros
from geometry_msgs.msg import Twist
import math
import copy

class NavigateToGoalState(smach.State):
    """Состояние: навигация к целевой точке"""
    
    def __init__(self, node, nav_manager):
        smach.State.__init__(
            self,
            outcomes=['navigated', 'no_object', 'aborted', 'preempted'],
            input_keys=['object_name'],
            output_keys=[]
        )
        self.node = node
        self.nav_manager = nav_manager
        self.target_x = None
        self.target_y = None
        self.target_yaw = None
        
    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Отправляем цель навигации
        for obj in self.node.localized_objects:
            if obj.class_name == userdata.object_name:
                self.target_x = obj.x
                self.target_y = obj.y
                self.target_yaw = obj.yaw

        if self.target_x == None:
            self.node.get_logger().error(f'{userdata.object_name} нет в списке найденных объектов')
            return 'no_object'
        
        self.node.get_logger().info(f'Начало навигации к точке: ({self.target_x}, {self.target_y})')
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[9/10] NAVIGATING_TO_HOME'
        feedback_msg.status_message = 'Движение к домашней точке...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Отправляем цель
        if not self.nav_manager.send_goal(self.target_x, self.target_y, self.target_yaw):
            return 'aborted'
        
        # Ждём результат
        if self.nav_manager.wait_for_result(timeout_sec=60.0):
            self.node.get_logger().info('Навигация завершена успешно')
            return 'navigated'
        else:
            self.node.get_logger().error('Навигация завершилась ошибкой')
            return 'aborted'
    
    def request_preempt(self):
        """Обработчик прерывания"""
        self.nav_manager.cancel_goal()
        super().request_preempt()
