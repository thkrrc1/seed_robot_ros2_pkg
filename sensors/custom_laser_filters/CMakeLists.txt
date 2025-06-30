cmake_minimum_required(VERSION 3.5)
project(custom_laser_filters)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
ament_auto_add_library(custom_laser_filters SHARED src/custom_laser_filters.cpp)
target_compile_options(custom_laser_filters PRIVATE -O0 -g -Wall)

pluginlib_export_plugin_description_file(filters custom_laser_filters_plugins.xml)

install(TARGETS custom_laser_filters DESTINATION lib)

ament_export_dependencies(rosidl_default_runtime)
ament_package()