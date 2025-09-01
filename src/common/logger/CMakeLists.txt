add_library(liblogger
    src/logger.cpp
)

# 添加带命名空间的别名（关键）
add_library(common::liblogger ALIAS liblogger)

# 包含头文件路径
# target_include_directories(liblogger PUBLIC
#     ${CMAKE_CURRENT_SOURCE_DIR}/include
# )
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

target_include_directories(liblogger PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(liblogger
    rclcpp::rclcpp
    fmt::fmt
)

install(TARGETS liblogger
    EXPORT export_logger
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

# ament_export_include_directories(include)  # 导出头文件路径
ament_export_dependencies(rclcpp)          # 导出依赖库
ament_export_targets(export_logger HAS_LIBRARY_TARGET)
