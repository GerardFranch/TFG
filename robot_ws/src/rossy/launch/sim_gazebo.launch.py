import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node



def generate_launch_description():

    archivo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('rossy'),'launch','archivo.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    mundo_vacio = os.path.join(
        get_package_share_directory('rossy'),
        'worlds',
        'empty.sdf'
        )
    
    mundo = LaunchConfiguration('mundo')

    mundo_argumento = DeclareLaunchArgument(
        'mundo',
        default_value=mundo_vacio,
        description='Mundo a cargar'
        )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', mundo], 'on_exit_shutdown': 'true'}.items()  
             )

    generar_robot = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'rossy',
                                   '-z', '0.1'],
                        output='screen')
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller"
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster"
        ],
    )
    
    parametros_bridge = os.path.join(get_package_share_directory('rossy'),'configuracions','gz_bridge.yaml')
    
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={parametros_bridge}',
        ]
    )

    return LaunchDescription([
        mundo_argumento,
        archivo_launch,
        gazebo,
        generar_robot,
        diff_drive_spawner,
        joint_broad_spawner,
        ros_gz_bridge,
    ])