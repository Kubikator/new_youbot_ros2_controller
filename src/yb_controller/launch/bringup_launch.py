import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    youbot_control_dir = get_package_share_directory('yb_controller')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(youbot_control_dir, 'launch', 'robot_launch.py')
        )
    )
    
    arm_controller = Node(
        package='yb_controller',
        executable='arm_controller',
        name='arm_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    gripper_controller = Node(
        package='yb_controller',
        executable='gripper_controller_node',
        name='gripper_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )

    detection_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='yb_controller',
                executable='detection_node',
                name='detection_camera_node',
                output='screen',
                parameters=[
                    {'camera_name': 'camera'},
                    {'confidence_threshold': 0.5},
                    {'iou_threshold': 0.4},
                    {'device': 0},  # 0 для GPU, 'cpu' для CPU
                ]
            )
        ]
    )

    detection_1_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='yb_controller',
                executable='detection_node',
                name='detection_camera_1_node',
                output='screen',
                parameters=[
                    {'camera_name': 'camera_1'},
                    {'confidence_threshold': 0.5},
                    {'iou_threshold': 0.4},
                    {'device': 0},  # 0 для GPU, 'cpu' для CPU
                ]
            )
        ]
    )

    coordinate_finder_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='yb_controller',
                executable='coordinate_finder_node',
                name='coordinate_finder_node',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    searching_action = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='yb_controller',
                executable='searching_actionServer',
                name='searching_actionServer',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    align_action = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='yb_controller',
                executable='alignment_actionServer',
                name='alignment_actionServer',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    closing_action = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='yb_controller',
                executable='closing_actionServer',
                name='closing_actionServer',
                output='screen',
                parameters=[
                    {'use_sim_time': True}
                ]
            )
        ]
    )

    return LaunchDescription([
        # Сразу запускаем
        robot_launch,
        arm_controller,
        gripper_controller,
        detection_node,
        detection_1_node,
        coordinate_finder_node,
        searching_action,
        align_action,
        closing_action,
    ])