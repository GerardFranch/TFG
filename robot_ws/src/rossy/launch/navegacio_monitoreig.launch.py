import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declarar argumentos
    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_time_argumento = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar tiempo simulado si es verdadero'
    )

    # Nodo de SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ]),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('rossy'),
                'configuracions',
                'mapper_params_online_async.yaml'
            ),
            'use_sim_time': use_sim_time
        }.items()
    )

    # Nodo de Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Nodo de Twist Stamper
    twist_stamper_node = Node(
        package="twist_stamper",
        executable="twist_stamper",
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_in', '/cmd_vel'),
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ]
    )

    # Nodo de RViz2
    rviz_config_path = os.path.join(
        get_package_share_directory('rossy'),
        'configuracions',
        'nav.rviz'
    )
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Retrasar la ejecución de Nav2, Twist Stamper y RViz2
    delayed_nav2_launch = TimerAction(
        period=5.0,  # Retrasar 5 segundos
        actions=[nav2_launch]
    )

    delayed_twist_stamper_node = TimerAction(
        period=10.0,  # Retrasar 10 segundos
        actions=[twist_stamper_node]
    )

    delayed_rviz2_node = TimerAction(
        period=15.0,  # Retrasar 15 segundos
        actions=[rviz2_node]
    )

    # Retornar la descripción del lanzamiento
    return LaunchDescription([
        sim_time_argumento,
        slam_toolbox_launch, 
        delayed_nav2_launch,  
        delayed_twist_stamper_node,  
        delayed_rviz2_node 
    ])