import os
import shutil
import yaml
import rclpy
from distutils.util import strtobool
import threading

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, LogInfo, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition, UnlessCondition 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, TextSubstitution,
    Command, FindExecutable
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from tf2_msgs.msg import TFMessage
from pal_statistics_msgs.msg import StatisticsNames
from nav_msgs.msg import Odometry, OccupancyGrid

from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool

import subprocess


# rviz2の起動を行う。
def bringup_rviz(description, robot_pkg):
    # SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
    rviz_config_file = PathJoinSubstitution(
        [robot_pkg, "rviz", "rviz.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "--ros-args", "--log-level", "error"],
    )
    description.add_action(rviz_node)


def call_launch(name, description, robot_pkg, extra_args=None):
    launch_arguments = {'robot_pkg_path': PathJoinSubstitution([robot_pkg])}

    if extra_args:
        launch_arguments.update(extra_args)

    launch_file_path = PathJoinSubstitution([
        robot_pkg,
        'launch',
        'parts',
        name
    ])

    action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file_path),
        launch_arguments=[(key, value) for key, value in launch_arguments.items()]
    )

    description.add_action(action)
        

def generate_launch_description():
    pkg_name = 'lifter_mover'
    
    robot_pkg = FindPackageShare(pkg_name)
    robot_pkg_path = get_package_share_directory(pkg_name)
    
    slam = LaunchConfiguration('slam')
    use_localization = LaunchConfiguration('use_localization')

    slam_arg = DeclareLaunchArgument('slam', default_value='false')
    use_localization_arg = DeclareLaunchArgument('use_localization', default_value='true')

    read_map_yaml_file = PathJoinSubstitution([robot_pkg, 'config', 'navigation', 'map', 'scan_map.yaml'])

    ld = LaunchDescription()
    
    ld.add_action(slam_arg)
    ld.add_action(use_localization_arg)

    bringup_rviz(ld, robot_pkg)

    ros2_control_ready_monitor_node = Node(
        package=pkg_name,
        executable='ros2_control_ready_monitor_node',
        name='ros2_control_ready_monitor_node',
        output='screen',
        parameters=[
            {'robot_pkg_path': robot_pkg},
        ]
    )
    ld.add_action(ros2_control_ready_monitor_node)

    controllers_ready_monitor_node = Node(
        package=pkg_name,
        executable='controllers_ready_monitor_node',
        name='controllers_ready_monitor_node',
        output='screen',
        parameters=[
            {'robot_pkg_path': robot_pkg},
            {'simulation': False},
            {'slam': slam},
            {'use_localization': use_localization},
            {'map': read_map_yaml_file},
        ]
    )
    ld.add_action(controllers_ready_monitor_node)

    bringup_tools_node = Node(
        package=pkg_name,
        executable='amcl_state_monitor_node',
        name='amcl_state_monitor_node',
        output='screen',
        parameters=[
            {'robot_pkg_path': robot_pkg},
            {'simulation': False},
        ],
        condition=UnlessCondition(slam)
    )
    ld.add_action(bringup_tools_node)

    call_launch("bringup_robot_model.launch.py", ld, robot_pkg, extra_args={'simulation': "false",})

    return ld
