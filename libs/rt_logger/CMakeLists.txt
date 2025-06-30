cmake_minimum_required(VERSION 3.28)
project (rt_logger)

find_package(spdlog REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} ${SPDLOG_INCLUDES_LEVEL} INTERFACE "${CMAKE_CURRENT_LIST_DIR}/include")
target_link_libraries(${PROJECT_NAME} INTERFACE spdlog::spdlog)
