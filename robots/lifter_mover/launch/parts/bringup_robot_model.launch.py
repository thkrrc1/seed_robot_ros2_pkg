import os
import shutil
import yaml
import threading
import time
import rclpy
from distutils.util import strtobool

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from tf2_msgs.msg import TFMessage
from pal_statistics_msgs.msg import StatisticsNames
from rclpy.node import Node as RclpyNode
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool


def load_driver_settings(context, *args, **kwargs):
    driver_settings_file_raw = kwargs["driver_settings_raw"].perform(context)
    driver_settings_file = kwargs["driver_settings"].perform(context)
    shutil.copy(driver_settings_file_raw, driver_settings_file)


def interpret_robot_model(driver_settings_file, robot_pkg):
    robot_description_content = Command([
        PathJoinSubstitution(FindExecutable(name="xacro")),
        " ",
        PathJoinSubstitution([robot_pkg, "model", "lifter_mover.urdf.xacro"]),
        " ",
        "driver_settings:=",
        driver_settings_file,
    ])
    robot_description = {"robot_description": robot_description_content}
    return robot_description


def call_launch(name, description, robot_pkg, delay_time=0, extra_args=None):
    launch_arguments = {
        'robot_pkg_path': PathJoinSubstitution([robot_pkg])
    }
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
    if(delay_time != 0):
        delayed_controller_node = TimerAction(
            period=delay_time,
            actions=[action]
        )
        description.add_action(delayed_controller_node)
    else:
        description.add_action(action)


def generate_launch_description():
    pkg_name = 'lifter_mover'
    robot_pkg = FindPackageShare(pkg_name)
    robot_pkg_path = get_package_share_directory(pkg_name)

    driver_settings_raw = PathJoinSubstitution([robot_pkg, 'config', 'driver_settings.yaml'])
    driver_settings = PathJoinSubstitution([robot_pkg, 'config', 'driver_settings_tmp.yaml'])
    controller_settings = PathJoinSubstitution([robot_pkg, 'config', 'controller_settings.yaml'])

    ld = LaunchDescription()

    tf_static_monitor_node = Node(
        package="lifter_mover",
        executable="tf_static_monitor_node",
        name="tf_static_monitor_node",
        output="screen",
    )
    ld.add_action(tf_static_monitor_node)

    tf_static_ready_monitor_node = Node(
        package="lifter_mover",
        executable="tf_static_ready_monitor_node",
        name="tf_static_ready_monitor_node",
        output="screen",
        parameters=[{'param_path': controller_settings}],
    )
    ld.add_action(tf_static_ready_monitor_node)

    load_driver_func = OpaqueFunction(function=load_driver_settings, kwargs={
        "driver_settings_raw": driver_settings_raw,
        "driver_settings": driver_settings
    })
    ld.add_action(load_driver_func)

    ld.add_action(DeclareLaunchArgument(
        'robot_pkg_path',
        default_value=TextSubstitution(text=robot_pkg_path),
        description='Path to the ' + pkg_name + ' package'
    ))

    robot_description = interpret_robot_model(driver_settings, robot_pkg)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    ld.add_action(robot_state_pub_node)

    return ld
