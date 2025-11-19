#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import sys
import math


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self, x, y, yaw=0.0):
        """
        Отправить цель навигации
        x, y - координаты в метрах
        yaw - угол поворота в радианах 
        """
        self.get_logger().info(f'Waiting for action server...')
        self.nav_to_pose_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0
        
        # Преобразуем yaw в quaternion
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw} rad')
        
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Robot is moving...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('=== Goal reached! ===')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Можно раскомментировать для отслеживания прогресса
        # self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f}m')
        pass


def main():
    if len(sys.argv) < 3:
        print("Usage: ros2 run yb_controller send_goal <x> <y> [yaw]")
        print("Example: ros2 run yb_controller send_goal 5.0 3.0 1.57")
        print("  x, y - coordinates in meters")
        print("  yaw - rotation angle in radians (optional, default=0)")
        return
    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    
    rclpy.init()
    goal_sender = GoalSender()
    goal_sender.send_goal(x, y, yaw)
    
    try:
        rclpy.spin(goal_sender)
    except KeyboardInterrupt:
        pass
    finally:
        goal_sender.destroy_node()


if __name__ == '__main__':
    main()