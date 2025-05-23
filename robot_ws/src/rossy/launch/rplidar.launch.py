import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    nodo_rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/lidar',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    return LaunchDescription([
        nodo_rplidar
    ])