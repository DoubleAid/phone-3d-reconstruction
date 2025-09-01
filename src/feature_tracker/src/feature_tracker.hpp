#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "track_worker.hpp"
#include "logger.hpp"

using namespace std;
using namespace common;

class FeatureTracker : public rclcpp::Node {
public:
    FeatureTracker();
    ~FeatureTracker();
    void initialize();
private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    bool first_image_flag_;
    double first_image_time_;
    double last_image_time_;

    TrackWorker tracker_;

    std::string sub_image_topic_;
    image_transport::Subscriber image_subscriber_;
    // 带特征跟踪结果的可视化图像
    image_transport::Publisher match_publisher_;
    // 特征点发布
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr feature_publisher_;
    // 重启信号（当跟踪失败时触发）
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr restart_publisher_;
};