cmake_minimum_required(VERSION 3.10)
project(config_controller_msgs)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/MsConfigRead.srv"
    "srv/MsConfigWrite.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()