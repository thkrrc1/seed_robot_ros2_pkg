import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_share_path

import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument

import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import xacro

import yaml

def replace_usb_settings(fpath_in, fpath_out):
    
    with open(fpath_in) as in_file:
        config = yaml.safe_load(in_file)
    for usb_setting in config["usb_settings"]:
        usb_setting["port"] = "./tmp" + usb_setting["port"]
    with open(fpath_out, "w") as out_file:
        yaml.dump(config, out_file, default_flow_style=False)
    

def generate_launch_description():
    driver_settings_file = os.path.join(get_package_share_directory('typeg'), 'config', 'driver_settings.yaml')
    driver_settings_file_tmp = os.path.join(get_package_share_directory('typeg'), 'config', 'driver_settings_tmp.yaml')
    replace_usb_settings(driver_settings_file, driver_settings_file_tmp)
    
    ms_stub_node = Node(
        package="ms_stub",
        name = 'ms_stub',
        executable="ms_stub",
        arguments=[driver_settings_file_tmp],
        output="both",
    )
        
        
    nodes = [
        ms_stub_node,
        ]
    
    return launch.LaunchDescription(nodes)
