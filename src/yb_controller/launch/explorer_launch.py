import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('yb_controller')
    
    nav2_params_path = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup'
        ),
        
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/cmd_vel')]
        ),
        
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
        ),
        
        # Behavior Server
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
        ),
        
        # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
        ),
        
        # Velocity Smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params_path, {'use_sim_time': use_sim_time}]
        ),
        
        # Lifecycle Manager для навигации
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': [
                    'controller_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'velocity_smoother'
                ]}
            ]
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
                {'costmap_topic': 'map'},
                {'costmap_updates_topic': 'map_updates'},
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