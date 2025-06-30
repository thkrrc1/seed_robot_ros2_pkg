cmake_minimum_required(VERSION 3.5)
project(seed_ros2_controller)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)
INIT_PROJECT()

USE_QT(Widgets)

set(rt_logger_DIR ${PROJECT_SOURCE_DIR}/../../libs/rt_logger/cmake)
set(hardware_interface_DIR ${CMAKE_PREFIX_PATH}/share/hardware_interface/cmake)
set(gz_ros2_control_DIR ${CMAKE_PREFIX_PATH}/share/gz_ros2_control/cmake)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rt_logger REQUIRED)
find_package(joint_limits REQUIRED)


ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
target_include_directories(${PROJECT_NAME} PUBLIC include)
#target_include_directories(${PROJECT_NAME} PRIVATE ${HEADER_DIRS})
target_include_directories(${PROJECT_NAME} PRIVATE
  ${HEADER_DIRS}
  ${CMAKE_INSTALL_PREFIX}/hardware_interface/include
  ${CMAKE_INSTALL_PREFIX}/gz_ros2_control/include
)
target_link_libraries(${PROJECT_NAME} PRIVATE rt_logger pthread -lstdc++fs -rdynamic yaml-cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -O0 -g -Wall)

ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  controller_interface
  hardware_interface
  gz_ros2_control
  pluginlib
  rclcpp
  rclcpp_lifecycle
  joint_limits
)

pluginlib_export_plugin_description_file(hardware_interface seed_ros2_controller.xml)
pluginlib_export_plugin_description_file(gz_ros2_control seed_ros2_controller.xml)

install(TARGETS
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/ DESTINATION include
)

ament_export_include_directories(
  include
)

ament_package()
