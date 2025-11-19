import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from yb_interfaces.action import PickupObject
import smach
import smach_ros
from geometry_msgs.msg import Twist
import math

class NavigateToHomeState(smach.State):
    """Состояние: навигация к домашней позиции"""
    
    def __init__(self, node, nav_manager):
        smach.State.__init__(
            self,
            outcomes=['navigated', 'aborted', 'preempted'],
            input_keys=['object_name'],
            output_keys=[]
        )
        self.node = node
        self.nav_manager = nav_manager
        # Задайте координаты домашней позиции
        self.home_x = self.node.home_x  # измените на ваши координаты
        self.home_y = self.node.home_y  # измените на ваши координаты
        self.home_yaw = self.node.home_yaw
        
    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        self.node.get_logger().info(f'Навигация к домашней позиции: ({self.home_x}, {self.home_y})')
        
        # Публикуем feedback
        feedback_msg = PickupObject.Feedback()
        feedback_msg.current_state = '[10/10] NAVIGATING_TO_HOME'
        feedback_msg.status_message = 'Возврат к домашней позиции...'
        feedback_msg.current_x = self.node.current_position['x']
        feedback_msg.current_y = self.node.current_position['y']
        feedback_msg.current_z = self.node.current_position['z']
        self.node.goal_handle.publish_feedback(feedback_msg)
        
        # Отправляем цель
        if not self.nav_manager.send_goal(self.home_x, self.home_y, self.home_yaw):
            return 'aborted'
        
        # Ждём результат
        if self.nav_manager.wait_for_result(timeout_sec=90.0):
            self.node.get_logger().info('Возврат домой завершён успешно')
            return 'navigated'
        else:
            self.node.get_logger().error('Возврат домой завершился ошибкой')
            return 'aborted'
    
    def request_preempt(self):
        """Обработчик прерывания"""
        self.nav_manager.cancel_goal()
        super().request_preempt()
