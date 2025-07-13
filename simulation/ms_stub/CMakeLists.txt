cmake_minimum_required(VERSION 3.10)
project(ms_stub)

set(CMAKE_CXX_STANDARD 20)

include(cmake/dirlist.cmake)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(seed_ros2_controller REQUIRED)
find_package(yaml-cpp REQUIRED)

ADD_FILES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_FILES)
ADD_FILES("${PROJECT_SOURCE_DIR}" "cpp" SOURCE_FILES)
ADD_FILE_DIRECTORIES("${PROJECT_SOURCE_DIR}" "hpp" HEADER_DIRS)

include_directories(include ${HEADER_DIRS})
add_executable(${PROJECT_NAME} ${HEADER_FILES} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBS} pthread -lstdc++fs -lutil yaml-cpp)
target_compile_options(${PROJECT_NAME} PUBLIC -O0 -g -Wall)

ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp seed_ros2_controller)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME} 
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()