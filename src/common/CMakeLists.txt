cmake_minimum_required(VERSION 3.8)
project(common)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖（如 Eigen、OpenCV）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fmt REQUIRED)
# find_package(Eigen3 REQUIRED)

# 添加子模块
# add_subdirectory(proto)
add_subdirectory(logger)
add_subdirectory(time)
# add_subdirectory(math)

# target_compile_features(logger PUBLIC cxx_std_20)

ament_export_targets(export_logger)
ament_export_targets(export_time)

# 导出依赖
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)          # 传递依赖
ament_package()