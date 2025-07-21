from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Ruta al archivo URDF del robot
    urdf_path = os.path.join(
        get_package_share_directory('triciclo_py'),
        'urdf',
        'triciclo.urdf'
    )

    # Leer contenido del archivo URDF
    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    # Definir los nodos a lanzar: Robot_state_publisher, JointStatePublisher y RViz
    return LaunchDescription([
    
    # Publica las transformaciones entre los enlaces (tf) a partir del URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
    # GUI que permite mover las articulaciones del robot en tiempo real
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        
     # Lanza RViz para visualizar el modelo
        Node(
            package='rviz2',
            executable='rviz2'
        )
    ])
