#pragma once
#include <string>
#include <fmt/format.h>
#include <rclcpp/rclcpp.hpp>

namespace common {

class Logger {
public:

static void init(rclcpp::Node::SharedPtr node);

template<typename... Args>
static void debug(fmt::format_string<Args...> fmt, Args&&... args) {
    std::string message = fmt::format(fmt, std::forward<Args>(args)...);
    RCLCPP_DEBUG(ros_logger_, "%s", message.c_str());
}

template<typename... Args>
static void info(fmt::format_string<Args...> fmt, Args&&... args) {
    std::string message = fmt::format(fmt, std::forward<Args>(args)...);
    RCLCPP_INFO(ros_logger_, "%s", message.c_str());
}

template<typename... Args>
static void error(fmt::format_string<Args...> fmt, Args&&... args) {
    std::string message = fmt::format(fmt, std::forward<Args>(args)...);
    RCLCPP_ERROR(ros_logger_, "%s", message.c_str());
}

private:
static rclcpp::Logger ros_logger_;

};

}