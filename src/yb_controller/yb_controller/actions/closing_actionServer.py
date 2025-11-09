#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from yb_interfaces.action import ClosingToObject
from yb_interfaces.msg import ObjectCenter3Array, ObjectCenter3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import tf2_geometry_msgs
from geometry_msgs.msg import Vector3

import math


class ClosingToObjectActionServer(Node):
    
    def __init__(self):
        super().__init__('closing_object_action_server')

        # Инициализация tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Переменная для хранения преобразования
        self.transform = None
        self.transform_ready = False
        
        # Таймер для получения преобразования с повторными попытками
        self.transform_timer = self.create_timer(1.0, self.lookup_transform)
        
        # Параметры
        self.declare_parameter('linear_speed_max', 0.2)  # м/с
        self.declare_parameter('objects_topic', '/objects_centers')
        self.declare_parameter('target_distance', 0.3)  # Целевое расстояние в метрах
        self.declare_parameter('distance_tolerance', 0.05)  # Допустимое отклонение в метрах
        self.declare_parameter('max_lost_frames', 30)  # Максимальное число кадров без объекта
        self.declare_parameter('kp', 0.5)  # Коэффициент P-регулятора для скорости
        self.declare_parameter('min_speed', 0.05)  # Минимальная скорость движения
        
        self.linear_speed_max = self.get_parameter('linear_speed_max').value
        self.target_distance = self.get_parameter('target_distance').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value
        self.max_lost_frames = self.get_parameter('max_lost_frames').value
        self.kp = self.get_parameter('kp').value
        self.min_speed = self.get_parameter('min_speed').value
        
        # Callback group для параллельной обработки
        self.cb_group = ReentrantCallbackGroup()
        
        # Action Server
        self._action_server = ActionServer(
            self,
            ClosingToObject,
            'closing_object',
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
        self.objects_subscriber = self.create_subscription(
            ObjectCenter3Array,
            self.get_parameter('objects_topic').value,
            self.objects_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Состояние
        self.detected_objects = []
        
        self.get_logger().info('Closing Object Action Server started')
    
    def goal_callback(self, goal_request):
        """Принять или отклонить новую цель"""
        self.get_logger().info(f'Received approach goal for object: {goal_request.object_type}')
        return GoalResponse.ACCEPT
    
    def lookup_transform(self):
        """Получение преобразования из tf2 с обработкой ошибок"""
        try:
            # Получаем преобразование из base_link в mnp_link
            self.transform = self.tf_buffer.lookup_transform(
                'mnp_link',
                'base_link', 
                rclpy.time.Time()
            )
            self.transform_ready = True
            self.get_logger().info('✅ Transform base_link -> mnp_link успешно получен')
            
            # Останавливаем таймер после успешного получения
            self.transform_timer.cancel()
            
        except TransformException as ex:
            self.get_logger().warn(f'⏳ Ожидание преобразования base_link -> mnp_link: {ex}')
            self.transform_ready = False

    
    def cancel_callback(self, goal_handle):
        """Обработка запроса на отмену"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def objects_callback(self, msg):
        """Обработка сообщений с координатами объектов"""
        self.detected_objects = msg.centers
    
    def stop_robot(self):
        """Остановка робота"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def move_robot(self, linear_velocity):
        """Движение робота с заданной линейной скоростью"""
        # Ограничиваем скорость
        linear_velocity = max(0.0, min(self.linear_speed_max, linear_velocity))
        
        twist = Twist()
        twist.linear.x = linear_velocity
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
    
    def calculate_distance(self, obj):
        
        distance = obj.center.x
        
        return distance
    
    def calculate_distance_error(self, current_distance):
        """
        Вычислить ошибку расстояния
        Returns: error в метрах (положительное = нужно подъехать, отрицательное = слишком близко)
        """
        return current_distance - self.target_distance
    
    def is_at_target_distance(self, error):
        """Проверить, достигнуто ли целевое расстояние"""
        return abs(error) <= self.distance_tolerance
    
    def transform_point(self, point_base_link):
            
        try:
            point_stamped = tf2_geometry_msgs.PointStamped()
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.header.frame_id = 'base_link'
            point_stamped.point.x = point_base_link.x
            point_stamped.point.y = point_base_link.y
            point_stamped.point.z = point_base_link.z

            transformed = tf2_geometry_msgs.do_transform_point(point_stamped, self.transform)
            
            result = Vector3()
            result.x = transformed.point.x
            result.y = transformed.point.y
            result.z = transformed.point.z
            
            return result
            
        except Exception as e:
            self.node.get_logger().error(f'Ошибка преобразования: {e}')
            return None
    
    async def execute_callback(self, goal_handle):
        """
        Главная логика выполнения action
        """
        self.get_logger().info('Executing approach...')
        
        # Получаем параметры цели
        target_type = goal_handle.request.object_type
        
        # Сброс состояния
        self.detected_objects = []
        
        # Feedback сообщение
        feedback_msg = ClosingToObject.Feedback()
        
        # Счётчики
        lost_frames_counter = 0
        consecutive_arrived_frames = 0
        required_arrived_frames = 5  # Требуется N последовательных кадров на целевом расстоянии
        
        # Управление
        rate = self.create_rate(10)  # 10 Hz
        
        try:
            while rclpy.ok():
                # Проверка на отмену
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.stop_robot()
                    self.get_logger().info('Goal canceled')
                    return ClosingToObject.Result(success=False, problem='lost')
                
                # Поиск объекта в текущем кадре
                found, obj_data = self.find_target_object(target_type)
                
                if not found:
                    # Объект не найден в текущем кадре
                    lost_frames_counter += 1
                    consecutive_arrived_frames = 0
                    
                    # Проверка: превышен ли лимит потерянных кадров
                    if lost_frames_counter >= self.max_lost_frames:
                        self.stop_robot()
                        self.get_logger().error('Object lost for too long, aborting approach')
                        goal_handle.abort()
                        return ClosingToObject.Result(success=False, problem='lost')
                    
                    # Останавливаем робота когда объект не виден
                    self.stop_robot()
                    
                    # Обновляем feedback
                    feedback_msg.current_distance = 0.0
                    feedback_msg.object_visible = False
                    goal_handle.publish_feedback(feedback_msg)
                    
                else:
                    # Объект найден - сбрасываем счётчик потерь
                    lost_frames_counter = 0
                    obj_data.center = self.transform_point(obj_data.center)
                    
                    # Вычисляем текущее расстояние
                    current_distance = self.calculate_distance(obj_data)
                    
                    # Вычисляем ошибку расстояния
                    error = self.calculate_distance_error(current_distance)

                    # Проверка на потерю выравнивания
                    if abs(obj_data.center.y) > 0.1:
                        self.stop_robot()
                        self.get_logger().info('Lost alignment!')
                        goal_handle.abort()
                        return ClosingToObject.Result(success=False, problem='align')
                    
                    # Проверяем достижение целевого расстояния
                    if self.is_at_target_distance(error):
                        consecutive_arrived_frames += 1
                        
                        self.get_logger().info(
                            f'At target distance! ({consecutive_arrived_frames}/{required_arrived_frames})',
                            throttle_duration_sec=0.5
                        )
                        
                        # Если на целевом расстоянии достаточно долго - успех
                        if consecutive_arrived_frames >= required_arrived_frames:
                            self.stop_robot()
                            self.get_logger().info('Approach succeeded!')
                            goal_handle.succeed()
                            return ClosingToObject.Result(success=True, problem='')
                        
                        # Останавливаем при достижении цели
                        self.stop_robot()
                    else:
                        # Не достигли цели - сбрасываем счётчик
                        consecutive_arrived_frames = 0
                        
                        # Проверка: не слишком ли близко?
                        if error < 0:
                            # Слишком близко - отъезжаем назад
                            self.get_logger().warn(
                                f'Too close! Distance: {current_distance:.3f}m, backing up...',
                                throttle_duration_sec=0.5
                            )
                            # Если хотите отъезжать назад:
                            linear_velocity = self.kp * error
                            self.move_robot(linear_velocity)
                        else:
                            # Нужно подъехать ближе
                            # P-регулятор для управления линейной скоростью
                            linear_velocity = self.kp * error
                            
                            # Обеспечиваем минимальную скорость для преодоления трения
                            if linear_velocity > 0 and linear_velocity < self.min_speed:
                                linear_velocity = self.min_speed
                            
                            self.move_robot(linear_velocity)
                            
                            self.get_logger().info(
                                f'Approaching... distance: {current_distance:.3f}m, '
                                f'error: {error:.3f}m, cmd: {linear_velocity:.3f} m/s',
                                throttle_duration_sec=0.5
                            )
                    
                    # Обновляем feedback
                    feedback_msg.current_distance = current_distance
                    feedback_msg.object_visible = True
                    goal_handle.publish_feedback(feedback_msg)
                
                # Ждём следующий цикл
                rate.sleep()
            
            # Если вышли из цикла (не должно происходить при нормальной работе)
            self.stop_robot()
            goal_handle.abort()
            self.get_logger().error('Approach loop exited unexpectedly')
            return ClosingToObject.Result(success=False, problem='error')
            
        except Exception as e:
            self.get_logger().error(f'Error during approach: {str(e)}')
            self.stop_robot()
            goal_handle.abort()
            return ClosingToObject.Result(success=False, problem='error')


def main(args=None):
    rclpy.init(args=args)
    
    approach_server = ClosingToObjectActionServer()
    
    # Используем многопоточный executor для параллельной обработки callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(approach_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        approach_server.stop_robot()
        approach_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()