cmake_minimum_required(VERSION 3.5)
project(motion_player_msgs)

find_package(rosidl_default_generators REQUIRED)

set(action_files
  "action/Play.action"
)

set(srv_files
    "srv/Load.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${action_files}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()