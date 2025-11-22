from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'yb_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/resource', ['resource/youbot.urdf']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/models', glob('models/*')),
        ('share/' + package_name + '/ui', glob('ui/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='egor',
    maintainer_email='monsterkuba@yandex.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yb_odometry = yb_controller.nodes.yb_odometry:main',
            'arm_controller = yb_controller.nodes.arm_controller:main',
            'gripper_controller_node = yb_controller.nodes.gripper_controller_node:main',
            'detection_node = yb_controller.detection.detection_node:main',
            'coordinate_finder_node = yb_controller.triangulation.coordinate_finder_node:main',
            'pickingObject_actionServer = yb_controller.actions.pickingObject_actionServer:main',
            'smach_node = yb_controller.smach.smach_node:main',
            'send_goal = yb_controller.nodes.send_goal:main',
            'objects_finder = yb_controller.detection.objects_localizator_node:main',
            'gui_controller = yb_controller.gui.gui_controller:main',
            'explore_status_monitor = yb_controller.nodes.explore_status_monitor:main'
        ],
    },
)
