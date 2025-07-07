import os
import shutil
import yaml
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from rclpy.node import Node as RclpyNode


def load_driver_settings(context, *args, **kwargs):
    driver_settings_file_raw = kwargs["driver_settings_raw"].perform(context)
    driver_settings_file = kwargs["driver_settings"].perform(context)
    shutil.copy(driver_settings_file_raw, driver_settings_file)


def interpret_robot_model(driver_settings_file, robot_pkg):
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([robot_pkg, "model", "noid.urdf.xacro"]),
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
    if delay_time != 0:
        delayed_controller_node = TimerAction(
            period=delay_time,
            actions=[action]
        )
        description.add_action(delayed_controller_node)
    else:
        description.add_action(action)


def launch_setup(context, *args, **kwargs):
    pkg_name = kwargs["pkg_name"]
    robot_pkg = FindPackageShare(pkg_name).perform(context)

    controller_defs = [
        {
            "name": "lifter_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "rarm_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "larm_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "rhand_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "lhand_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "head_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "waist_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"),
            "remappings": []
        },
        {
            "name": "joint_state_broadcaster",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_joint_state.yaml"),
            "remappings": []
        },
        {
            "name": "mechanum_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": [("~/cmd_vel", "/cmd_vel_nav")]
        },
        {
            "name": "diagnostic_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "aero_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "status_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "robotstatus_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
        {
            "name": "config_controller",
            "param_file": os.path.join(robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"),
            "remappings": []
        },
    ]

    high_priority = {"mechanum_controller", "joint_state_broadcaster"}
    mid_priority = {"head_controller", "rarm_controller", "larm_controller", "rhand_controller", "lhand_controller", "waist_controller", "lifter_controller"}
    high_defs = [ctrl for ctrl in controller_defs if ctrl["name"] in high_priority]
    mid_defs = [ctrl for ctrl in controller_defs if ctrl["name"] in mid_priority]
    low_defs = [ctrl for ctrl in controller_defs if ctrl["name"] not in high_priority.union(mid_priority)]

    actions = []
    delay_sec = 0.0

    for ctrl in high_defs:
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{ctrl['name']}_spawner",
            output="screen",
            arguments=[
                ctrl["name"],
                "--controller-manager", "/controller_manager",
                "-p", ctrl["param_file"]
            ],
            remappings=ctrl["remappings"]
        )
        actions.append(TimerAction(period=delay_sec, actions=[spawner]))
        delay_sec += 2.0

    for ctrl in mid_defs:
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{ctrl['name']}_spawner",
            output="screen",
            arguments=[
                ctrl["name"],
                "--controller-manager", "/controller_manager",
                "-p", ctrl["param_file"]
            ],
            remappings=ctrl["remappings"]
        )
        actions.append(TimerAction(period=delay_sec, actions=[spawner]))
        delay_sec += 2.0

    low_spawners = []
    for ctrl in low_defs:
        spawner = Node(
            package="controller_manager",
            executable="spawner",
            name=f"{ctrl['name']}_spawner",
            output="screen",
            arguments=[
                ctrl["name"],
                "--controller-manager", "/controller_manager",
                "-p", ctrl["param_file"]
            ],
            remappings=ctrl["remappings"]
        )
        low_spawners.append(spawner)

    actions.append(TimerAction(period=delay_sec, actions=low_spawners))

    return actions



def generate_launch_description():
    pkg_name = 'typeg'
    robot_pkg = FindPackageShare(pkg_name)
    robot_pkg_path = get_package_share_directory(pkg_name)

    driver_settings_raw = PathJoinSubstitution([robot_pkg, 'config', 'driver_settings.yaml'])
    driver_settings = PathJoinSubstitution([robot_pkg, 'config', 'driver_settings_tmp.yaml'])
    controller_settings = PathJoinSubstitution([robot_pkg, 'config', 'controller_settings.yaml'])
    teleop_settings = PathJoinSubstitution([robot_pkg, 'config', 'teleop', 'teleop_settings.yaml'])

    ld = LaunchDescription()

    ld.add_action(Node(
        package=pkg_name,
        executable="tf_static_monitor_node",
        name="tf_static_monitor_node",
        output="screen",
    ))
    ld.add_action(Node(
        package=pkg_name,
        executable="tf_static_ready_monitor_node",
        name="tf_static_ready_monitor_node",
        output="screen",
        parameters=[{'cm_param_path': controller_settings}, {'teleop_param_path': teleop_settings}],
    ))

    ld.add_action(OpaqueFunction(function=load_driver_settings, kwargs={
        "driver_settings_raw": driver_settings_raw,
        "driver_settings": driver_settings
    }))

    robot_description = interpret_robot_model(driver_settings, robot_pkg)

    ld.add_action(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    ))

    ld.add_action(OpaqueFunction(function=launch_setup, kwargs={"pkg_name": pkg_name}))

    return ld
