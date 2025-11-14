cmake_minimum_required(VERSION 3.8)
project(vins_estimator)

# 查找 ROS 2 和自定义依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

# 添加可执行文件
add_executable(vins_estimator_node
  src/estimator_node.cpp
  src/estimator.cpp
  src/feature_manager.cpp
)

target_include_directories(vins_estimator_node PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(vins_estimator_node
  rclcpp
  image_transport
  cv_bridge
  OpenCV
)

target_link_libraries(vins_estimator_node
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  image_transport::image_transport

  ${OpenCV_LIBS}
  ${std_srvs_LIBRARIES}
  common::liblogger
  common::libtime
)

# 安装可执行文件到 lib/<package_name>
install(TARGETS vins_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(unittest
  unittest/main.cpp
)

target_include_directories(unittest PRIVATE
  ${Boost_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(unittest
  rclcpp
  image_transport
  cv_bridge
  OpenCV
)

target_link_libraries(unittest
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  image_transport::image_transport

  ${CERES_LIBRARIES}
  ${OpenCV_LIBS}
  ${std_srvs_LIBRARIES}
  common::liblogger
  common::libtime
)

# 安装可执行文件到 lib/<package_name>
install(TARGETS unittest
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
# )

# 安装启动文件（如果有）
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# 导出依赖，使其他包能找到当前包
ament_export_dependencies(
    rclcpp
    image_transport
    cv_bridge
    common
)

ament_package()