#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
import numpy as np
from yb_interfaces.action import PickupObject
from yb_interfaces.msg import BoundingBoxArray, ObjectCenter3Array
from rclpy.action import ActionServer
import math
import smach
import smach_ros
from geometry_msgs.msg import Pose, Point, Twist
from yb_controller.smach.searching_state import SearchObjectState
from yb_controller.smach.align_state import AlignToObjectState
from yb_controller.smach.openGripper_state import OpenGripperState
from yb_controller.smach.moveGripper_state import MovingGripperState
from yb_controller.smach.closeGripper_state import CloseGripperState
from yb_controller.smach.LiftObject_state import LiftObjectState

class PickupActionNode(Node):
    """Главная нода для управления действием захвата объекта"""

    def __init__(self):
        super().__init__('pickup_action_server')

        self.goal_handle = None

        self._action_server = ActionServer(
            self,
            PickupObject,
            'pickup_object',
            self.execute_callback,
        )
        
        # Subscribers
        self.gripper_current_point_subscriber = self.create_subscription(
            Point,
            'arm_current_position_point',
            self.gripper_current_point_callback,
            10
        )
        self.gripper_gap_subscriber = self.create_subscription(
            Float64,
            'gripper_current_gap',
            self.gripper_gap_callback,
            10
        )
        
        self.detected_objects_subscriber = self.create_subscription(
            BoundingBoxArray,
            '/camera/detected_objects',
            self.detected_objects_callback,
            10
        )

        self.determined_objects_subscriber = self.create_subscription(
            ObjectCenter3Array,
            '/objects_centers',
            self.determined_objects_callback,
            10
        )

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.arm_target_publisher = self.create_publisher(
            Point,
            'arm_target_pose',
            10
        )

        self.gripper_target_gap_publisher = self.create_publisher(
            Float64,
            'gripper_target_gap',
            10
        )

        self.GRIPPER_OPEN_GAP = 0.071
        self.GRIPPER_CLOSED_GAP = 0.04
        self.POSITION_TOLERANCE = 0.02
        self.GRIPPER_TOLERANCE = 0.005
        self.SEARCH_ANGULAR_VELOCITY = 0.3  # рад/с для поиска объекта
        self.ALIGN_ANGULAR_VELOCITY = 0.1  # рад/с для выравнивания
        self.TARGET_CENTER_X = 240  # Целевая X координата центра изображения
        self.ALIGN_TOLERANCE = 20  # Допуск по X в пикселях
        
        # Смещение манипулятора относительно base_link
        self.ARM_LINK_OFFSET = {
            'x': 0.16,
            'y': 0.0,
            'z': 0.144 - 0.253  # -0.109
        }
        
        # State variables
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_gripper_gap = 0.0
        self.detected_objects = []
        self.objects_centers = []
        
        self.get_logger().info('Pickup Object Action Server запущен')
    
    def execute_callback(self, goal_handle):
        """Callback для выполнения action"""
        self.get_logger().info(f'Получена цель захвата объекта: {goal_handle.request.object_name}')
        self.goal_handle = goal_handle
        
        # Создаём State Machine
        sm = smach.StateMachine(outcomes=['success', 'failure', 'preempted'])
        
        # Добавляем userdata для передачи данных между состояниями
        sm.userdata.object_name = goal_handle.request.object_name
        
        # Определяем состояния
        with sm:
            smach.StateMachine.add(
                'SEARCHING_OBJECT',
                SearchObjectState(self),
                transitions={
                    'found': 'ALIGNING_TO_OBJECT',
                    'searching': 'SEARCHING_OBJECT',
                    'failure': 'failure',
                    'preempted': 'preempted'
                }
            )
            
            smach.StateMachine.add(
                'ALIGNING_TO_OBJECT',
                AlignToObjectState(self),
                transitions={
                    'aligned': 'OPENING_GRIPPER',
                    'aligning': 'ALIGNING_TO_OBJECT',
                    'lost': 'SEARCHING_OBJECT',
                    'failure': 'failure',
                    'preempted': 'preempted'
                }
            )
            
            smach.StateMachine.add(
                'OPENING_GRIPPER',
                OpenGripperState(self),
                transitions={
                    'opened': 'MOVING_TO_TARGET',
                    'waiting': 'OPENING_GRIPPER',
                    'failure': 'failure',
                    'preempted': 'preempted'
                }
            )
            
            smach.StateMachine.add(
                'MOVING_TO_TARGET',
                MovingGripperState(self),
                transitions={
                    'reached': 'CLOSING_GRIPPER',
                    'moving': 'MOVING_TO_TARGET',
                    'failure': 'failure',
                    'preempted': 'preempted'
                }
            )
            
            smach.StateMachine.add(
                'CLOSING_GRIPPER',
                CloseGripperState(self),
                transitions={
                    'closed': 'LIFTING_OBJECT',
                    'closing': 'CLOSING_GRIPPER',
                    'failure': 'failure',
                    'preempted': 'preempted'
                }
            )
            
            smach.StateMachine.add(
                'LIFTING_OBJECT',
                LiftObjectState(self),
                transitions={
                    'lifted': 'success',
                    'lifting': 'LIFTING_OBJECT',
                    'failure': 'failure',
                    'preempted': 'preempted'
                }
            )
        
        # Выполняем State Machine
        outcome = sm.execute()
        
        # Формируем результат
        result = PickupObject.Result()
        
        if outcome == 'success':
            goal_handle.succeed()
            result.success = True
            result.message = f'Объект {goal_handle.request.object_name} успешно захвачен'
        elif outcome == 'preempted':
            goal_handle.canceled()
            result.success = False
            result.message = 'Захват объекта отменён'
        else:  # failure
            goal_handle.abort()
            result.success = False
            result.message = f'Ошибка при захвате объекта {goal_handle.request.object_name}'
        
        return result

    def gripper_gap_callback(self, msg: Float64):
        """Callback для текущего раскрытия захвата"""
        self.current_gripper_gap = msg.data

    def gripper_current_point_callback(self, msg: Point):
        """Callback для текущей позиции манипулятора"""
        self.current_position['x'] = msg.x
        self.current_position['y'] = msg.y
        self.current_position['z'] = msg.z
    
    def detected_objects_callback(self, msg: BoundingBoxArray):
        """Callback для обнаруженных объектов"""
        self.detected_objects = msg.boxes

    def determined_objects_callback(self, msg: ObjectCenter3Array):
        """Callback для центров объектов"""
        self.objects_centers = msg.centers

def main(args=None):
    rclpy.init(args=args)
    action_server = PickupActionNode()
    
    try:
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        pass
    
    action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()