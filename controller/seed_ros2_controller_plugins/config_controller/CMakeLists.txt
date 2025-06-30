cmake_minimum_required(VERSION 3.10)
project(config_controller)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

set(rt_logger_DIR ${PROJECT_SOURCE_DIR}/../../../libs/rt_logger/cmake)

set(ROS_PKGS ${ROS_PKGS}
    rclcpp
    rclcpp_lifecycle
    pluginlib
    controller_interface
    seed_ros2_controller
    controller_manager_msgs
    config_controller_msgs
    std_srvs
 )

foreach(Pkg IN ITEMS ${ROS_PKGS})
    find_package(${Pkg} REQUIRED)
endforeach()
find_package(rt_logger REQUIRED)
    
ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${HEADER_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBS} rt_logger pthread -lstdc++fs )
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ROS_PKGS})
target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g -Wall)

pluginlib_export_plugin_description_file(controller_interface config_controller_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
