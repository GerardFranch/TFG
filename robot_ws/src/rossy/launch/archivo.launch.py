import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
import xacro

def generate_launch_description():
    
    # Ver si tenemos que usar sim_time o ros2_contro
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    sim_time_argumento = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar tiempo simulado si es verdadero'
            )
    
    ros2_control_argumento = DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Usar ros2_control si es verdadero'
            )
    
    # Ruta al archivo .xacro
    ruta_paquete = get_package_share_directory('rossy')
    archivo_xacro = os.path.join(ruta_paquete, 'description', 'robot.urdf.xacro')
    
    # Procesar el archivo .xacro para generar el URDF
    # robot_description = xacro.process_file(archivo_xacro).toxml()
    
    # Pasamos argumento al xacro para la logica de ros2_control o gazebo_control
    robot_description_conf = ParameterValue(
        Command(['xacro ',archivo_xacro,' use_ros2_control:=',use_ros2_control]),
        value_type=str
    )
    # Nodo de robot_state_publisher
    nodo_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_conf,
            'use_sim_time': use_sim_time
        }]
    )

    # Devolver la descripción del lanzamiento
    return LaunchDescription([
        sim_time_argumento,
        ros2_control_argumento,
        nodo_robot_state_publisher
    ])
