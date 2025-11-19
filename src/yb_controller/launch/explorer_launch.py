import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('yb_controller')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Explore Lite
        Node(
            package='explore_lite',
            executable='explore',
            name='explore',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_base_frame': 'base_link'},
                {'costmap_topic': '/map'},
                {'costmap_updates_topic': '/map_updates'},
                {'visualize': True},
                {'planner_frequency': 0.33},
                {'progress_timeout': 30.0},
                {'potential_scale': 3.0},
                {'orientation_scale': 0.0},
                {'gain_scale': 1.0},
                {'transform_tolerance': 0.3},
                {'min_frontier_size': 0.5}
            ]
        )
    ])