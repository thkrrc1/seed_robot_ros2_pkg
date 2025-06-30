cmake_minimum_required(VERSION 3.5)
project(mechanum_controller)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

set(rt_logger_DIR ${PROJECT_SOURCE_DIR}/../../../libs/rt_logger/cmake)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rt_logger REQUIRED)

generate_parameter_library(mechanum_controller_params
  src/mechanum_controller_params.yaml
)

set(LIBS ${LIBS} 
    mechanum_controller_params
    rt_logger)

ADD_FILES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_FILES)
ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)
add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${HEADER_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS} pthread -lstdc++fs -rdynamic)
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g -Wall)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  nav_msgs
  generate_parameter_library
  tf2
  tf2_msgs
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(controller_interface mechanum_controller.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_package()
