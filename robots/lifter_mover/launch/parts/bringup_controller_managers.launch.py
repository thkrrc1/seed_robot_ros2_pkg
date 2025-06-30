import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction, TimerAction

import rclpy
from rclpy.node import Node as RclpyNode
from controller_manager_msgs.srv import ListControllers


def wait_and_spawn(context, controller_name, param_file, remappings=None):
    rclpy.init()
    node = RclpyNode('controller_manager_waiter')
    client = node.create_client(ListControllers, 'controller_manager/list_controllers')

    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().warn(f"Service not available for {controller_name}")
        rclpy.shutdown()
        return []
    
    node.get_logger().info(f"Service available - spawning {controller_name}")
    rclpy.shutdown()

    return [Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            controller_name,
            "--controller-manager", "/controller_manager",
            "-p", param_file
        ],
        remappings=remappings or []
    )]
    

def generate_launch_description():
    robot_pkg = LaunchConfiguration("robot_pkg_path") 
    robot_pkg = FindPackageShare("lifter_mover") 

    controller_sets = {
        "trajectory": {
            "param_file": PathJoinSubstitution([robot_pkg, "config", "controllers", "controller_settings_joint_trajectory.yaml"]),
            "controllers": ["lifter_controller"]
        },
        "joint_state": {
            "param_file": PathJoinSubstitution([robot_pkg, "config", "controllers", "controller_settings_joint_state.yaml"]),
            "controllers": ["joint_state_broadcaster"]
        },
        "general": {
            "param_file": PathJoinSubstitution([robot_pkg, "config", "controllers", "controller_settings_mechanum.yaml"]),
            "controllers": ["mechanum_controller", "diagnostic_controller", "aero_controller", "status_controller", "robotstatus_controller", "config_controller"],
            "remappings": [("~/cmd_vel", "/cmd_vel_nav")]
        },
    }

    ld = LaunchDescription()

    lifecycle_monitor_node = Node(
        package="lifter_mover",
        executable="lifecycle_monitor_node",
        name="lifecycle_monitor_node",
        output="screen",
    )
    ld.add_action(lifecycle_monitor_node)

    all_controllers = []
    for category, config in controller_sets.items():
        param_file = config["param_file"]
        remaps = config.get("remappings", None)
        for ctrl in config["controllers"]:
            all_controllers.append((ctrl, param_file, remaps))

    delay = 0.0
    for ctrl, param_file, remaps in all_controllers:
        spawn_action = OpaqueFunction(function=wait_and_spawn, kwargs={
                'controller_name': ctrl,
                'param_file': param_file,
                'remappings': remaps
        })
        
        delayed_action = TimerAction(
            period=delay,
            actions=[spawn_action]
        )
        ld.add_action(delayed_action)
        delay += 2.0

    return ld
