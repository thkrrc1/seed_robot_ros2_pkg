set(RT_LOGGER_INCLUDE_DIR "${rt_logger_DIR}/../include")

find_package(spdlog REQUIRED)
add_library(rt_logger INTERFACE)
set_target_properties(rt_logger PROPERTIES
IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
INTERFACE_INCLUDE_DIRECTORIES "${RT_LOGGER_INCLUDE_DIR}"
)
