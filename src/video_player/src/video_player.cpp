#include "video_player.hpp"

VideoPlayer::VideoPlayer() : Node("video_player") {
    // 参数：视频文件路径
    declare_parameter("video_path", "");
    declare_parameter("loop", false);
    declare_parameter<int>("frame_rate", 30);
    // 获取参数
    video_path_ = get_parameter("video_path").as_string();
    loop_       = get_parameter("loop").as_bool();
    frame_rate_ = get_parameter("frame_rate").as_int();
}

VideoPlayer::~VideoPlayer() {
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void VideoPlayer::initialize() {
    // 日志打印
    Logger::init(this->shared_from_this());

    // 检查文件是否存在
    if (!std::filesystem::exists(video_path_)) {
        Logger::error("Video file not found {}", video_path_.c_str());
        throw std::runtime_error("Video file not Found");
    }

    // 打开视频文件
    cap_.open(video_path_);
    if (!cap_.isOpened()) {
        Logger::error("Failed to open video {}", video_path_.c_str());
        throw std::runtime_error("Failed to open video");
    }

    frame_width_ = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    frame_height_ = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    frame_count_ = cap_.get(cv::CAP_PROP_FRAME_COUNT);
    current_frame_ = 0;

    // 创建图像发布者
    image_transport::ImageTransport it(shared_from_this());
    image_publisher_ = it.advertise("video_frames", 1);

    // 创建服务
    next_frame_service_ = create_service<std_srvs::srv::Trigger>(
        "next_frame",
        std::bind(&VideoPlayer::nextFrameCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    prev_frame_service_ = create_service<std_srvs::srv::Trigger>(
        "prev_frame",
        std::bind(&VideoPlayer::prevFrameCallback, this,
                std::placeholders::_1, std::placeholders::_2));
    
    // 创建定时器
    frame_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / frame_rate_)),
        std::bind(&VideoPlayer::publishFrame, this));

    Logger::info("Video Node initialized");
}

void VideoPlayer::publishFrame() {
    if (!cap_.isOpened()) {
        Logger::error("publish frame failed, cant open cap");
        return;
    }
    cv::Mat frame;
    if (!cap_.read(frame)) {
        if (loop_) {
            Logger::info("replay...");
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            cap_.read(frame);
        } else {
            rclcpp::shutdown();
            return;
        }
    }

    // 发布当前帧
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = now();
    msg->header.frame_id = "camera_frame";
    // msg->header.fields.push_bacck
    image_publisher_.publish(msg);

    current_frame_ = cap_.get(cv::CAP_PROP_POS_FRAMES);
    Logger::info("published frame {}/{}", current_frame_, frame_count_);
}

void VideoPlayer::prevFrameCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                                    std_srvs::srv::Trigger::Response::SharedPtr response) {

}

void VideoPlayer::nextFrameCallback(const std_srvs::srv::Trigger::Request::SharedPtr,
                                    std_srvs::srv::Trigger::Response::SharedPtr response) {

}