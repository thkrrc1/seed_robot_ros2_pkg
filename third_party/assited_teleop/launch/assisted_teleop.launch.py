from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('assisted_teleop')
    params = os.path.join(pkg_share, 'config', 'assisted_teleop.yaml')

    return LaunchDescription([
        Node(
            package='assisted_teleop',
            executable='assisted_teleop_node',
            name='assisted_teleop',
            output='screen',
            parameters=[params],
        )
    ])
