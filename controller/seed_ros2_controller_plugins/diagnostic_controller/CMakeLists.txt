cmake_minimum_required(VERSION 3.5)
project(diagnostic_controller)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

set(rt_logger_DIR ${PROJECT_SOURCE_DIR}/../../../libs/rt_logger/cmake)

set(ROS_PKGS ${ROS_PKGS}
    rclcpp
    rclcpp_lifecycle
    pluginlib
    seed_ros2_controller
    diagnostic_updater
    aero_controller_msgs
    controller_interface
 )

foreach(Pkg IN ITEMS ${ROS_PKGS})
    find_package(${Pkg} REQUIRED)
endforeach()
find_package(rt_logger REQUIRED)

set(LIBS ${LIBS} 
    rt_logger
)

ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBS})
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ROS_PKGS})
target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g -Wall)

pluginlib_export_plugin_description_file(controller_interface ${PROJECT_NAME}_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
