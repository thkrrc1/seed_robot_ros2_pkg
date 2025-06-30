import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
  

def real_lidar(description):
    launch_dir = os.path.join(get_package_share_directory('urg_node2'), 'launch')
    description.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'urg_node2.launch.py')
            ),
            launch_arguments={
                'scan_topic_name': "scan_raw",
            }.items(),
        ))
    
    description.add_action(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name='front_laser_filter',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("lifter_mover"),
                    "config","laser", "laser_filter.yaml",
                ])],
            remappings=[('/scan', '/scan_raw'),('/scan_filtered', '/scan')],
        )
    )


def generate_launch_description():
    launch_description = launch.LaunchDescription()
    real_lidar(launch_description)
    return launch_description
