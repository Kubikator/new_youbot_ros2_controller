#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import threading


class ExplorationStatusNode(Node):
    def __init__(self):
        super().__init__('exploration_status_publisher')
        
        # Параметры
        self.declare_parameter('frontiers_topic', 'explore/frontiers')
        self.declare_parameter('status_topic', 'map_exploration_status')
        self.declare_parameter('timeout_seconds', 5.0)
        self.declare_parameter('min_frontiers_threshold', 0)
        self.declare_parameter('publish_rate', 2.0)  # Hz
        
        frontiers_topic = self.get_parameter('frontiers_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value
        self.min_frontiers_threshold = self.get_parameter('min_frontiers_threshold').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Переменные состояния
        self.last_frontiers_count = 0
        self.last_frontiers_time = None
        self.is_exploration_complete = False
        self.lock = threading.Lock()
        
        # Подписка на frontiers
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.frontiers_subscription = self.create_subscription(
            MarkerArray,
            frontiers_topic,
            self.frontiers_callback,
            qos_profile
        )
        
        # Публикация статуса исследования (с транзиентной локальной durability)
        status_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Сохраняет последнее сообщение для новых подписчиков
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.status_publisher = self.create_publisher(
            Bool,
            status_topic,
            status_qos
        )
        
        # Таймер для публикации статуса
        self.publish_timer = self.create_timer(1.0 / publish_rate, self.publish_status)
        
        # Таймер для проверки таймаута
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)
        
        self.get_logger().info('Exploration Status Publisher запущен')
        self.get_logger().info(f'Подписан на: {frontiers_topic}')
        self.get_logger().info(f'Публикует в: {status_topic}')
        self.get_logger().info(f'Таймаут: {self.timeout_seconds} сек')
        self.get_logger().info(f'Порог frontiers: {self.min_frontiers_threshold}')
    
    def frontiers_callback(self, msg):
        """Обработчик сообщений с frontiers"""
        with self.lock:
            frontiers_count = len(msg.markers) // 2  # Каждый frontier = 2 маркера
            self.last_frontiers_count = frontiers_count
            self.last_frontiers_time = self.get_clock().now()
            
            # Определяем статус исследования
            old_status = self.is_exploration_complete
            self.is_exploration_complete = (frontiers_count <= self.min_frontiers_threshold)
            
            # Логируем изменение статуса
            if old_status != self.is_exploration_complete:
                if self.is_exploration_complete:
                    self.get_logger().info('🚩 ИССЛЕДОВАНИЕ ЗАВЕРШЕНО! Frontiers: {}'.format(frontiers_count))
                else:
                    self.get_logger().info('🔄 Исследование продолжается. Frontiers: {}'.format(frontiers_count))
            
            self.get_logger().debug(f'Frontiers: {frontiers_count}, Статус: {self.is_exploration_complete}')
    
    def check_timeout(self):
        """Проверка таймаута получения frontiers"""
        with self.lock:
            if self.last_frontiers_time is None:
                return
            
            current_time = self.get_clock().now()
            time_diff = current_time - self.last_frontiers_time
            time_diff_sec = time_diff.nanoseconds / 1e9
            
            if time_diff_sec > self.timeout_seconds:
                # Если долго нет сообщений, считаем что explore_lite неактивен
                old_status = self.is_exploration_complete
                self.is_exploration_complete = True  # или False, в зависимости от логики
                
                if not old_status:
                    self.get_logger().warn(
                        f'Таймаут frontiers ({time_diff_sec:.1f} сек). '
                        'Предполагаем, что исследование завершено.'
                    )
    
    def publish_status(self):
        """Публикация текущего статуса исследования"""
        with self.lock:
            status_msg = Bool()
            
            # Если нет данных от explore_lite, считаем что исследование не завершено
            if self.last_frontiers_time is None:
                status_msg.data = False
            else:
                status_msg.data = self.is_exploration_complete
            
            self.status_publisher.publish(status_msg)
    
    def get_detailed_status(self):
        """Получение детального статуса (может использоваться другими компонентами)"""
        with self.lock:
            if self.last_frontiers_time is None:
                return {
                    'exploring': False,
                    'completed': False,
                    'frontiers_count': 0,
                    'message': 'Нет данных от explore_lite',
                    'active': False
                }
            
            current_time = self.get_clock().now()
            time_diff = current_time - self.last_frontiers_time
            time_diff_sec = time_diff.nanoseconds / 1e9
            
            return {
                'exploring': not self.is_exploration_complete,
                'completed': self.is_exploration_complete,
                'frontiers_count': self.last_frontiers_count,
                'time_since_last_msg': time_diff_sec,
                'active': time_diff_sec <= self.timeout_seconds
            }


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ExplorationStatusNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Завершение работы Exploration Status Publisher')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()