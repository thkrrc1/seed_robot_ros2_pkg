from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def bringup_joy(description):
    joy = Node(
        package="joy",
        executable="joy_node",
        output="log"
    )
    description.add_action(joy)


def bringup_teleop(description, robot_pkg):
    config = PathJoinSubstitution(
        [robot_pkg,
         "config",
         "teleop",
         "teleop_settings.yaml"]
    )
    
    teleop_twist_joy = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        output="log",
        parameters=[config]
    )
    description.add_action(teleop_twist_joy)
    

def generate_launch_description():
    launch_description = LaunchDescription()
    robot_pkg = LaunchConfiguration("robot_pkg_path") 
    
    bringup_joy(launch_description)
    bringup_teleop(launch_description, robot_pkg)
    
    return launch_description
