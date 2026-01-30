cmake_minimum_required(VERSION 3.5)
project(motion_player)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

set(rt_logger_DIR ${PROJECT_SOURCE_DIR}/../../../libs/rt_logger/cmake)

set(ROS_PKGS ${ROS_PKGS}
    rclcpp
    rclcpp_lifecycle
    pluginlib
    seed_ros2_controller
    rclcpp_action
    urdf
    controller_manager_msgs
    controller_interface
    motion_player_msgs
    sensor_msgs
    control_msgs
 )

foreach(Pkg IN ITEMS ${ROS_PKGS})
    find_package(${Pkg} REQUIRED)
endforeach()
find_package(SDL2 REQUIRED)
find_package(rt_logger REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(Eigen3 REQUIRED)

set(LIBS ${LIBS} 
    Qt5::Widgets
    Eigen3::Eigen
    rt_logger
    avcodec
    avformat
    avutil
    avdevice
    swscale
    ${SDL2_LIBRARIES}
)

set(HEADER_DIRS ${HEADER_DIRS} ${PROJECT_SOURCE_DIR}/third_party)
set(HEADER_DIRS ${HEADER_DIRS} ${PROJECT_SOURCE_DIR}/third_party/picojson)

ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${HEADER_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${LIBS} pthread -lstdc++fs -rdynamic)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ROS_PKGS})
target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g -Wall)

pluginlib_export_plugin_description_file(controller_interface motion_player_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
