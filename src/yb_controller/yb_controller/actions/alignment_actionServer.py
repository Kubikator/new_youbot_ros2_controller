#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from yb_interfaces.action import AlignToObject
from yb_interfaces.msg import BoundingBoxArray


class AlignToObjectActionServer(Node):
    
    def __init__(self):
        super().__init__('align_to_object_action_server')
        
        # Параметры
        self.declare_parameter('angular_speed_max', 0.4)  # рад/с
        self.declare_parameter('detection_topic', '/camera/detected_objects')
        self.declare_parameter('target_center_x', 240)  # Целевая позиция в пикселях
        self.declare_parameter('alignment_tolerance', 15.0)  # Допустимое отклонение в пикселях
        self.declare_parameter('max_lost_frames', 30)  # Максимальное число кадров без объекта (3 сек при 10Hz)
        self.declare_parameter('kp', 0.002)  # Коэффициент P-регулятора
        
        self.angular_speed_max = self.get_parameter('angular_speed_max').value
        self.target_center_x = self.get_parameter('target_center_x').value
        self.alignment_tolerance = self.get_parameter('alignment_tolerance').value
        self.max_lost_frames = self.get_parameter('max_lost_frames').value
        self.kp = self.get_parameter('kp').value
        
        # Callback group для параллельной обработки
        self.cb_group = ReentrantCallbackGroup()
        
        # Action Server
        self._action_server = ActionServer(
            self,
            AlignToObject,
            'align_to_object',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # Subscribers
        self.detection_subscriber = self.create_subscription(
            BoundingBoxArray,
            self.get_parameter('detection_topic').value,
            self.detection_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Состояние
        self.detected_objects = []
        
        self.get_logger().info('Align To Object Action Server started')
    
    def goal_callback(self, goal_request):
        """Принять или отклонить новую цель"""
        self.get_logger().info(f'Received alignment goal for object: {goal_request.object_type}')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        """Обработка запроса на отмену"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def detection_callback(self, msg):
        """Обработка сообщений от системы детекции"""
        self.detected_objects = msg.boxes
    
    def stop_robot(self):
        """Остановка робота"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def rotate_robot(self, angular_velocity):
        """Вращение робота с заданной угловой скоростью"""
        # Ограничиваем скорость
        angular_velocity = max(-self.angular_speed_max, 
                               min(self.angular_speed_max, angular_velocity))
        
        twist = Twist()
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)
    
    def find_target_object(self, target_type):
        """
        Найти целевой объект в списке детекций
        Returns: (found, object_data)
        """
        for obj in self.detected_objects:
            if obj.class_name == target_type:
                return True, obj
        return False, None
    
    def calculate_alignment_error(self, center_x):
        """
        Вычислить ошибку выравнивания
        Returns: error в пикселях (положительное = объект справа, отрицательное = слева)
        """
        return center_x - self.target_center_x
    
    def is_aligned(self, error):
        """Проверить, достигнуто ли выравнивание"""
        return abs(error) <= self.alignment_tolerance
    
    async def execute_callback(self, goal_handle):
        """
        Главная логика выполнения action
        """
        self.get_logger().info('Executing alignment...')
        
        # Получаем параметры цели
        target_type = goal_handle.request.object_type
        
        # Сброс состояния
        self.detected_objects = []
        
        # Feedback сообщение
        feedback_msg = AlignToObject.Feedback()
        
        # Счётчики
        lost_frames_counter = 0
        consecutive_aligned_frames = 0
        required_aligned_frames = 5  # Требуется N последовательных выровненных кадров
        
        # Управление
        rate = self.create_rate(10)  # 10 Hz
        
        try:
            while rclpy.ok():
                # Проверка на отмену
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.stop_robot()
                    self.get_logger().info('Goal canceled')
                    return AlignToObject.Result(success=False)
                
                # Поиск объекта в текущем кадре
                found, obj_data = self.find_target_object(target_type)
                
                if not found:
                    # Объект не найден в текущем кадре
                    lost_frames_counter += 1
                    consecutive_aligned_frames = 0
                    
                    # Проверка: превышен ли лимит потерянных кадров
                    if lost_frames_counter >= self.max_lost_frames:
                        self.stop_robot()
                        self.get_logger().error('Object lost for too long, aborting alignment')
                        goal_handle.abort()
                        return AlignToObject.Result(success=False)
                    
                    # Останавливаем робота когда объект не виден
                    self.stop_robot()
                    
                    # Обновляем feedback
                    feedback_msg.alignment_error = 0.0
                    feedback_msg.object_visible = False
                    goal_handle.publish_feedback(feedback_msg)
                    
                else:
                    # Объект найден - сбрасываем счётчик потерь
                    lost_frames_counter = 0
                    
                    # Вычисляем ошибку выравнивания
                    error = self.calculate_alignment_error(obj_data.center_x)
                    
                    # Проверяем выравнивание
                    if self.is_aligned(error):
                        consecutive_aligned_frames += 1
                        
                        self.get_logger().info(
                            f'Aligned! ({consecutive_aligned_frames}/{required_aligned_frames})',
                            throttle_duration_sec=0.5
                        )
                        
                        # Если выровнены достаточно долго - успех
                        if consecutive_aligned_frames >= required_aligned_frames:
                            self.stop_robot()
                            self.get_logger().info('Alignment succeeded!')
                            goal_handle.succeed()
                            return AlignToObject.Result(success=True)
                        
                        # Останавливаем при выравнивании
                        self.stop_robot()
                    else:
                        # Не выровнены - сбрасываем счётчик
                        consecutive_aligned_frames = 0
                        
                        # P-регулятор для управления угловой скоростью
                        # Положительная ошибка (объект справа) -> поворот вправо (положительная angular.z)
                        angular_velocity = -self.kp * error
                        
                        self.rotate_robot(angular_velocity)
                        
                        self.get_logger().info(
                            f'Aligning... error: {error:.1f}px, cmd: {angular_velocity:.3f} rad/s',
                            throttle_duration_sec=0.5
                        )
                    
                    # Обновляем feedback
                    feedback_msg.alignment_error = error
                    feedback_msg.object_visible = True
                    goal_handle.publish_feedback(feedback_msg)
                
                # Ждём следующий цикл
                rate.sleep()
            
            # Если вышли из цикла (не должно происходить при нормальной работе)
            self.stop_robot()
            goal_handle.abort()
            self.get_logger().error('Alignment loop exited unexpectedly')
            return AlignToObject.Result(success=False)
            
        except Exception as e:
            self.get_logger().error(f'Error during alignment: {str(e)}')
            self.stop_robot()
            goal_handle.abort()
            return AlignToObject.Result(success=False)


def main(args=None):
    rclpy.init(args=args)
    
    align_server = AlignToObjectActionServer()
    
    # Используем многопоточный executor для параллельной обработки callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(align_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        align_server.stop_robot()
        align_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()