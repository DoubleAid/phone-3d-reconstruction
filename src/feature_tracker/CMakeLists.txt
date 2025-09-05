cmake_minimum_required(VERSION 3.8)
project(feature_tracker)

# 查找 ROS 2 和自定义依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(common REQUIRED)

# 添加可执行文件
add_executable(feature_tracker_node
  src/feature_tracker_node.cpp
  src/feature_tracker.cpp
)

target_include_directories(feature_tracker_node PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
)

ament_target_dependencies(feature_tracker_node
  rclcpp
  image_transport
  cv_bridge
  OpenCV
)

target_link_libraries(feature_tracker_node
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  image_transport::image_transport
  ${OpenCV_LIBS}
  ${std_srvs_LIBRARIES}
  common::liblogger
  common::libtime
)

# 安装可执行文件到 lib/<package_name>
install(TARGETS feature_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装启动文件（如果有）
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖，使其他包能找到当前包
ament_export_dependencies(
    rclcpp
    image_transport
    cv_bridge
    common
)

ament_package()