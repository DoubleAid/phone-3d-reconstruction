#include "feature_tracker.hpp"

FeatureTracker::FeatureTracker() : Node("feature_tracker") {
    bool equalize = false;

    first_image_flag_ = true;
    
    // 获取参数
    declare_parameter("topic_name", "video_frames");
    declare_parameter("equalize", true);
    sub_image_topic_    = get_parameter("topic_name").as_string();
    equalize            = get_parameter("equalize").as_bool();

    tracker_.equalize_ = equalize;
}

FeatureTracker::~FeatureTracker() {

}

void FeatureTracker::initialize() {
    Logger::init(this->shared_from_this());
    Logger::info("feature tracker initialize");
    
    // 订阅消息并绑定响应函数
    image_transport::ImageTransport it(shared_from_this());
    image_subscriber_ = it.subscribe(
        sub_image_topic_,    // topic 名称
        100,            // 队列大小
        std::bind(&FeatureTracker::imageCallback, this, std::placeholders::_1)
    );

    // 初始化发布重启信号
    restart_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "/restart_signal",
        10
    );
}

void FeatureTracker::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
    // 首帧初始化
    rclcpp::Time stamp(img_msg->header.stamp);

    if (first_image_flag_) {
        Logger::info("first frame initialize");
        first_image_flag_ = false;
        first_image_time_ = stamp.seconds();
        last_image_time_ = stamp.seconds();
        return;
    }

    // 检测图像流不连续或者时间回退
    if (stamp.seconds() - last_image_time_ > 1.0 ||
        stamp.seconds() < last_image_time_) {
        Logger::info("image discontinue! reset feature tracker");
        first_image_flag_ = true;
        last_image_time_ = 0;
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        restart_publisher_->publish(msg);
        return;
    }

    last_image_time_ = stamp.seconds();
    Logger::info("received image time {}", last_image_time_);

    cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat show_img = ptr->image;

    tracker_.readImage(ptr->image, stamp.seconds());
}
