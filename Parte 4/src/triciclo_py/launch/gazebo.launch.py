from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('triciclo_py')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'triciclo.urdf')

    with open(urdf_path, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        # Publicar robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        # Spawn del robot en Gazebo Harmonic
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', 'triciclo',
                '-file', urdf_path
            ],
            output='screen'
        )
    ])

