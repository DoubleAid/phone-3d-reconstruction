#include "logger.hpp"

namespace common {

rclcpp::Logger Logger::ros_logger_ = rclcpp::get_logger("default_logger");

void Logger::init(rclcpp::Node::SharedPtr node) {
    ros_logger_ = node->get_logger();
}

}