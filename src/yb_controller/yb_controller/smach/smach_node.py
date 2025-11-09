#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
import smach
import sys
from smach import State
from yb_interfaces.action import SearchObject
from yb_interfaces.action import AlignToObject
from yb_interfaces.action import ClosingToObject


class SearchObjectState(State):
    
    def __init__(self, node, object_type='cube'):
        State.__init__(
            self,
            outcomes=['found', 'not_found', 'error']
        )
        
        self._node = node
        self._object_type = object_type
        
        # Action Client
        self._action_client = ActionClient(
            self._node,
            SearchObject,
            'search_object'
        )
        
        self._node.get_logger().info('Waiting for search_object action server...')
        self._action_client.wait_for_server()
        self._node.get_logger().info('Connected to search_object action server')
    
    def execute(self, userdata):

        self._node.get_logger().info(f'Searching for object: {self._object_type}')
        
        # Создаём цель
        goal = SearchObject.Goal()
        goal.object_type = self._object_type
        
        # Отправляем цель
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        # Ждём принятия цели
        rclpy.spin_until_future_complete(self._node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self._node.get_logger().error('Goal rejected by action server')
            return 'error'
        
        self._node.get_logger().info('Goal accepted, waiting for result...')
        
        # Ждём результата
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        
        result = result_future.result().result
        
        # Обрабатываем результат
        if result.success:
            self._node.get_logger().info('Object found!')
            return 'found'
        else:
            self._node.get_logger().warn('Object not found')
            return 'not_found'
    
    def feedback_callback(self, feedback_msg):
        """Обработка feedback от action server"""
        feedback = feedback_msg.feedback
        self._node.get_logger().info(
            f'Search progress: objects={feedback.objects_in_view}',
            throttle_duration_sec=2.0  # Логируем не чаще раза в 2 сек
        )

class AlignObjectState(State):
    
    def __init__(self, node, object_type='cube'):
        State.__init__(
            self,
            outcomes=['aligned', 'object_lost', 'error']
        )
        
        self._node = node
        self._object_type = object_type
        
        # Action Client
        self._action_client = ActionClient(
            self._node,
            AlignToObject,
            'align_to_object'
        )
        
        self._node.get_logger().info('Waiting for align_to_object action server...')
        self._action_client.wait_for_server()
        self._node.get_logger().info('Connected to align_to_object action server')
    
    def execute(self, userdata):

        self._node.get_logger().info(f'Align to object: {self._object_type}')
        
        # Создаём цель
        goal = AlignToObject.Goal()
        goal.object_type = self._object_type
        
        # Отправляем цель
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        # Ждём принятия цели
        rclpy.spin_until_future_complete(self._node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self._node.get_logger().error('Goal rejected by action server')
            return 'error'
        
        self._node.get_logger().info('Goal accepted, waiting for result...')
        
        # Ждём результата
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        
        result = result_future.result().result
        
        # Обрабатываем результат
        if result.success:
            self._node.get_logger().info('Aligned!')
            return 'aligned'
        else:
            self._node.get_logger().warn('Object lost')
            return 'object_lost'
    
    def feedback_callback(self, feedback_msg):
        """Обработка feedback от action server"""
        feedback = feedback_msg.feedback
        self._node.get_logger().info(
            f'Align progress: aligned error={feedback.alignment_error} '
            f'object_visible={feedback.object_visible}',
            throttle_duration_sec=2.0  # Логируем не чаще раза в 2 сек
        )

class ClosingToObjectState(State):
    
    def __init__(self, node, object_type='cube'):
        State.__init__(
            self,
            outcomes=['closed', 'align_lost', 'object_lost', 'error']
        )
        
        self._node = node
        self._object_type = object_type
        
        # Action Client
        self._action_client = ActionClient(
            self._node,
            ClosingToObject,
            'closing_object'
        )
        
        self._node.get_logger().info('Waiting for closing_object action server...')
        self._action_client.wait_for_server()
        self._node.get_logger().info('Connected to closing_object action server')
    
    def execute(self, userdata):

        self._node.get_logger().info(f'Closing to object: {self._object_type}')
        
        # Создаём цель
        goal = ClosingToObject.Goal()
        goal.object_type = self._object_type
        
        # Отправляем цель
        send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        
        # Ждём принятия цели
        rclpy.spin_until_future_complete(self._node, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self._node.get_logger().error('Goal rejected by action server')
            return 'error'
        
        self._node.get_logger().info('Goal accepted, waiting for result...')
        
        # Ждём результата
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        
        result = result_future.result().result
        
        # Обрабатываем результат
        if result.success:
            self._node.get_logger().info('Closed!')
            return 'closed'
        elif result.problem == 'lost':
            self._node.get_logger().warn('Object lost')
            return 'object_lost'
        elif result.problem == 'align':
            self._node.get_logger().warn('Align lost')
            return 'align_lost'
        else:
            self._node.get_logger().warn('Error')
            return 'error'
    
    def feedback_callback(self, feedback_msg):
        """Обработка feedback от action server"""
        feedback = feedback_msg.feedback
        self._node.get_logger().info(
            f'Closing progress: current_distance={feedback.current_distance} '
            f'object_visible={feedback.object_visible}',
            throttle_duration_sec=2.0  # Логируем не чаще раза в 2 сек
        )


def create_search_state_machine(node, target_object):
    
    sm = smach.StateMachine(outcomes=['task_completed', 'task_failed'])
    
    with sm:
        # Добавляем состояние поиска
        smach.StateMachine.add(
            'SEARCH_OBJECT',
            SearchObjectState(node, object_type=target_object),
            transitions={
                'found': 'ALIGN_OBJECT',
                'not_found': 'task_failed',
                'error': 'task_failed'
            }
        )

        # Добавляем состояние выравнивания
        smach.StateMachine.add(
            'ALIGN_OBJECT',
            AlignObjectState(node, object_type=target_object),
            transitions={
                'aligned': 'CLOSING_OBJECT',
                'object_lost': 'SEARCH_OBJECT',
                'error': 'task_failed'
            }
        )

        # Добавляем состояние приближения
        smach.StateMachine.add(
            'CLOSING_OBJECT',
            ClosingToObjectState(node, object_type=target_object),
            transitions={
                'closed': 'task_completed',
                'align_lost': 'ALIGN_OBJECT',
                'object_lost': 'SEARCH_OBJECT',
                'error': 'task_failed'
            }
        )
    
    return sm


# Пример запуска
def main(args=None):

    # Обработка аргументов командной строки
    if len(sys.argv) < 2:
        return
    
    target_object = sys.argv[1]
    valid_objects = ['apple', 'ball', 'cube']
    
    if target_object not in valid_objects:
        print(f"Ошибка: неизвестный объект '{target_object}'")
        print("Доступные объекты:", ', '.join(valid_objects))
        return

    rclpy.init(args=args)
    node = rclpy.create_node('smach_node')
    
    try:
        sm = create_search_state_machine(node, target_object)
        outcome = sm.execute()
        node.get_logger().info(f'State machine finished with outcome: {outcome}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()