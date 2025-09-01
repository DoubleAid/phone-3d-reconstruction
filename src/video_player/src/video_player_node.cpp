#include <rclcpp/rclcpp.hpp>
#include "video_player.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VideoPlayer>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}