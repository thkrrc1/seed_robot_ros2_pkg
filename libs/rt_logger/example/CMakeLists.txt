cmake_minimum_required (VERSION 2.6)

project (rt_logger_example)


SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")

add_executable(rt_logger_example EXCLUDE_FROM_ALL sample.cpp )
target_link_libraries(rt_logger_example PRIVATE spdlog::spdlog)
target_include_directories(rt_logger_example PUBLIC ${PROJECT_SOURCE_DIR}/../src)
add_custom_target(example)
add_dependencies(example rt_logger_example)
