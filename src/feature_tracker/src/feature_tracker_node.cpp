#include <rclcpp/rclcpp.hpp>
#include "feature_tracker.hpp"

using namespace std;

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTracker>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}