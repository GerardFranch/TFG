import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    # Ruta al archivo .xacro
    ruta_paquete = get_package_share_directory('rossy')
    archivo_xacro = os.path.join(ruta_paquete, 'description', 'robot.urdf.xacro')
    
    # Procesar el archivo .xacro para generar el URDF
    descripcion_robot = xacro.process_file(archivo_xacro).toxml()

    # Declarar el argumento 'usar_tiempo_simulado'
    usar_tiempo_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Usar tiempo simulado si es verdadero'
    )

    # Nodo de publicador_estado_robot
    nodo_publicador_estado_robot = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': descripcion_robot,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Devolver la descripción del lanzamiento
    return LaunchDescription([
        usar_tiempo_sim,
        nodo_publicador_estado_robot
    ])
