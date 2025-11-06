import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import Imu
import numpy as np

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher_node')

        self.odometry_publisher = self.create_publisher(Odometry, 'odom', 10)

        self.gps_subscriber = self.create_subscription(PointStamped, 'my_robot/gps', self.gps_callback, 1)
        self.gps_speed_subscriber = self.create_subscription(Vector3, 'my_robot/gps/speed_vector', self.gps_speed_callback, 1)
        self.imu_subscriber = self.create_subscription(Imu, 'imu', self.imu_callback, 1)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu = Imu()
        self.gps = PointStamped()
        
        self.velocity_vec = None
        self.filtered_vx = 0.0
        self.filtered_vy = 0.0
        self.filtered_vz = 0.0

        self.timer = self.create_timer(0.1, self.publish_odometry)

    def gps_callback(self, msg):
        self.gps = msg

    def gps_speed_callback(self, msg):
        self.velocity_vec = msg

    def imu_callback(self, msg):
        self.imu = msg
    
    def publish_odometry(self):
        current_time = self.get_clock().now()
        
        gps_values = self.gps.point
        x = gps_values.x
        y = gps_values.y
        z = gps_values.z

        angular_velocities = self.imu.angular_velocity
        angular_x = angular_velocities.x
        angular_y = angular_velocities.y
        angular_z = angular_velocities.z

        orientation = self.imu.orientation
        x_orient = orientation.x
        y_orient = orientation.y
        z_orient = orientation.z
        w_orient = orientation.w
                
        # Ограничение максимальной скорости (защита от выбросов)
        max_velocity = 5.0  # м/с
        self.filtered_vx = np.clip(self.velocity_vec.x, -max_velocity, max_velocity)
        self.filtered_vy = np.clip(self.velocity_vec.y, -max_velocity, max_velocity)
        self.filtered_vz = np.clip(self.velocity_vec.z, -max_velocity, max_velocity)

        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z

        odom_msg.pose.pose.orientation.x = x_orient
        odom_msg.pose.pose.orientation.y = y_orient
        odom_msg.pose.pose.orientation.z = z_orient
        odom_msg.pose.pose.orientation.w = w_orient

        # Используем отфильтрованные скорости
        odom_msg.twist.twist.linear.x = self.filtered_vx
        odom_msg.twist.twist.linear.y = self.filtered_vy
        odom_msg.twist.twist.linear.z = self.filtered_vz

        odom_msg.twist.twist.angular.x = angular_x
        odom_msg.twist.twist.angular.y = angular_y
        odom_msg.twist.twist.angular.z = angular_z

        self.odometry_publisher.publish(odom_msg)

        # Broadcast TF transform
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = z

        tf_msg.transform.rotation = orientation

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_publisher = OdometryPublisher()
    rclpy.spin(odometry_publisher)
    odometry_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()