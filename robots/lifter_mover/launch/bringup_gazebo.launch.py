import os
import xacro
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution, 
    Command, 
    FindExecutable
)
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


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
        

def generate_launch_description() -> LaunchDescription:
    pkg_name = 'lifter_mover'

    robot_pkg = FindPackageShare(pkg_name)
    robot_pkg_path = get_package_share_directory(pkg_name)

    model_file_default = PathJoinSubstitution([robot_pkg, "gazebo", "urdf", "gazebo.urdf.xacro"])
    rviz_config_default = PathJoinSubstitution([robot_pkg, "gazebo", "config", "rviz.rviz"])

    model = LaunchConfiguration("model")
    model_name = LaunchConfiguration("model_name")
    world = LaunchConfiguration("world")
    rviz_config = LaunchConfiguration("rviz_config")
    slam = LaunchConfiguration('slam')
    use_localization = LaunchConfiguration('use_localization')

    model_arg = DeclareLaunchArgument("model", default_value=model_file_default)
    model_name_arg = DeclareLaunchArgument("model_name", default_value="SEED-Lifter-Mover-typeG")
    world_arg = DeclareLaunchArgument("world", default_value="empty.world")
    rviz_config_arg = DeclareLaunchArgument("rviz_config", default_value=rviz_config_default)
    slam_arg = DeclareLaunchArgument('slam', default_value='True')
    use_localization_arg = DeclareLaunchArgument('use_localization', default_value='True')
    
    robot_description_content = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([robot_pkg, "gazebo", "urdf", "gazebo.urdf.xacro"])
        ])
    robot_description = {"robot_description": robot_description_content}

    ld = LaunchDescription()
    
    ld.add_action(model_arg)
    ld.add_action(model_name_arg)
    ld.add_action(world_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(slam_arg)
    ld.add_action(use_localization_arg)

    show_robot_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[robot_description]
    )
    ld.add_action(show_robot_node)

    gz_sim_resource_path_candidates = [
        PathJoinSubstitution([robot_pkg, "gazebo", "models"]),
        ":",
        PathJoinSubstitution([robot_pkg, os.pardir]),
    ]
    set_gz_sim_resource_path_env = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        gz_sim_resource_path_candidates,
    )
    ld.add_action(set_gz_sim_resource_path_env)

    gazebo_robot_pkg_path = FindPackageShare("ros_gz_sim")
    gazebo_launch = IncludeLaunchDescription(
        launch_description_source=PathJoinSubstitution(
            [gazebo_robot_pkg_path, "launch", "gz_sim.launch.py"]
        ),
        launch_arguments={
            "gz_args": [
                "-r ",
                PathJoinSubstitution([robot_pkg, "gazebo", "worlds", "simple.world"]),
                " --physics-engine gz-physics-bullet-featherstone-plugin",
            ]
        }.items(),
    )
    ld.add_action(gazebo_launch)

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [
                        robot_pkg,
                        "gazebo",
                        "config",
                        "ros_gz_bridge.yaml",
                    ]
                ),
            }
        ],
        output="screen",
    )
    ld.add_action(ros_gz_bridge_node)

    gazebo_model_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="gazebo_robot_model_spawner",
        output="screen",
        arguments=[
            "-name",
            model_name,
            "-topic",
            "/robot_description",
        ],
    )
    ld.add_action(gazebo_model_spawner_node)

    robot_description_path = os.path.join(robot_pkg_path)
    xacro_file = os.path.join(robot_description_path, 'gazebo', 'urdf', 'gazebo.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    robot_desc = doc.toprettyxml(indent='  ')
    params = {'robot_description': robot_desc}
    controller_param_file = PathJoinSubstitution([robot_pkg, "gazebo", "config", "robot_hardware.yaml"])
    controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        name="controller_spawner",
        output="screen",
        arguments=[
            "lifter_controller",
            "--param-file",
            controller_param_file,
        ],
        parameters=[params, controller_param_file],
    )
    ld.add_action(controller_spawner_node)

    load_joint_state_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    ld.add_action(load_joint_state_controller)

    rviz2_node = Node(
        package="rviz2",
        arguments=["-d", rviz_config],
        executable="rviz2",
    )
    ld.add_action(rviz2_node)

    read_map_yaml_file = PathJoinSubstitution([robot_pkg_path, 'config', 'navigation', 'map', 'gz_test_map.yaml'])

    bringup_navigation_monitor_node = Node(
        package=pkg_name,
        executable='bringup_navigation_monitor_node',
        name='bringup_navigation_monitor_node',
        output='screen',
        parameters=[
            {'robot_pkg_path': robot_pkg_path},
            {'simulation': True},
            {'slam': slam},
            {'map': read_map_yaml_file},
        ]
    )
    ld.add_action(bringup_navigation_monitor_node)

    bringup_tools_node = Node(
        package=pkg_name,
        executable='amcl_state_monitor_node',
        name='amcl_state_monitor_node',
        output='screen',
        parameters=[
            {'robot_pkg_path': robot_pkg},
            {'simulation': True},
        ],
        condition=UnlessCondition(slam)
    )
    ld.add_action(bringup_tools_node)

    call_launch("bringup_gz_teleop.launch.py", ld, robot_pkg)

    return ld
