#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <filesystem>

#include <std_srvs/srv/trigger.hpp>

#include "logger.hpp"

using namespace std;
using namespace common;
using namespace std::chrono_literals;

class VideoPlayer : public rclcpp::Node {
public:
    VideoPlayer();

    ~VideoPlayer();

    void initialize();

private:
    void publishFrame();

    void prevFrameCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                           std_srvs::srv::Trigger::Response::SharedPtr response);

    void nextFrameCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                           std_srvs::srv::Trigger::Response::SharedPtr response);

    cv::VideoCapture cap_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    image_transport::Publisher image_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr next_frame_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prev_frame_service_;
    rclcpp::TimerBase::SharedPtr frame_timer_;

    bool        loop_;
    std::string video_path_;
    int8_t      frame_rate_;
    int32_t     frame_count_;
    int32_t     frame_width_;
    int32_t     frame_height_;
    int32_t     current_frame_;
};