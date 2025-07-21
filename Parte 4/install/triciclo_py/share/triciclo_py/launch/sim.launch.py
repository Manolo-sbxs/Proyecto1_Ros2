from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = get_package_share_directory('triciclo_py')
    urdf_file = os.path.join(pkg_path, 'urdf', 'triciclo.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_triciclo',
            arguments=[
                '-name', 'triciclo',
                '-x', '0', '-y', '0', '-z', '0.1',
                '-file', urdf_file
            ],
            output='screen'
        ),
    ])

