#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "feature_tracker.hpp"
// #include "lo"

using namespace std;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace common;

class FeatureTrackerManager : public rclcpp::Node {
public:
    FeatureTrackerManager();
    void initialize();
    void imageCallback(const Image::ConstSharedPtr& img_msg);
private:
    int     camera_num_;
    int     publish_count_;
    int     publish_frequence_;         // frequence (Hz) of publish tracking result.
    bool    init_publish_;
    bool    show_track_;
    bool    first_image_flag_;
    double  first_image_time_;
    double  last_image_time_;
    string  sub_image_topic_;

    // 光流追踪器
    vector<FeatureTracker>                      trackers_;

    image_transport::Subscriber                 image_subscriber_;
    image_transport::Publisher                  match_publisher_;       // 带特征跟踪结果的可视化图像
    rclcpp::Publisher<PointCloud2>::SharedPtr   feature_publisher_;     // 特征点发布
    rclcpp::Publisher<Bool>::SharedPtr          restart_publisher_;     // 重启信号（当跟踪失败时触发）
};

FeatureTrackerManager::FeatureTrackerManager()
    : Node("feature_tracker") {
    declare_parameter("camera_num", 1);                 // 相机数量
    declare_parameter("topic_name", "video_frames");    // 订阅的图片话题
    declare_parameter("equalize", true);                // 是否对图片进行直方图均衡化
    declare_parameter("max_feature_count", 150);        // 每一帧最大的特征点数
    declare_parameter("publish_frequence", 20);         // 发送频率，每秒钟发送的帧的个数
    declare_parameter("min_feature_dist", 20);          // 两个特征点之间的最小距离
    declare_parameter("show_track", true);              // 显示包含光流追踪点的图片

    camera_num_             = get_parameter("camera_num").as_int();
    publish_frequence_      = get_parameter("publish_frequence").as_int();
    sub_image_topic_        = get_parameter("topic_name").as_string();
    show_track_             = get_parameter("show_track").as_bool();

    int min_dist            = get_parameter("min_feature_dist").as_int();
    int max_feature_cnt     = get_parameter("max_feature_count").as_int();
    bool equalize           = get_parameter("equalize").as_bool();

    first_image_flag_       = true;
    first_image_time_       = 0;
    last_image_time_        = 0;
    publish_count_          = 0;
    init_publish

    for (size_t i = 0; i < camera_num_; i++) {
        trackers_.emplace_back(equalize, min_dist, max_feature_cnt);
    }
}

void FeatureTrackerManager::initialize() {
    Logger::init(this->shared_from_this());
    Logger::info("feature tracker manager initialize");

    // 订阅图片消息并绑定相应函数
    image_transport::ImageTransport it(shared_from_this());
    image_subscriber_ = it.subscribe(
        sub_image_topic_,   // 话题名称
        100,                // 队列大小
        std::bind(&FeatureTrackerManager::imageCallback, this, std::placeholders::_1)
    );

    // 初始化发布重启信号
    restart_publisher_ = this->create_publisher<Bool>(
        "/restart_signal",
        10
    );
    // 初始化特征点发布信号
    feature_publisher_ = this->create_publisher<PointCloud2>(
        "/feature",
        1000
    );
}

void FeatureTrackerManager::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
    // 首帧初始化
    bool publish_this_frame = true;
    rclcpp::Time stamp(img_msg->header.stamp);

    // 读取图片部分
    {
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

        if (round(1.0 * publish_count_ / (last_image_time_ - first_image_time_)) <= publish_frequence_) {
            publish_this_frame = true;
            if (abs(1.0 * publish_count_ / (stamp.seconds() - first_image_time_) - publish_frequence_) < 0.1 * publish_frequence_)
            {
                first_image_time_ = stamp.seconds();
                publish_count_ = 0;
            }
        } else {
            publish_this_frame = false;
        }

        cv_bridge::CvImageConstPtr ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        
        // 这点是处理多个相机的问题，暂时不需要
        for (size_t i = 0; i < camera_num_; i++) {
            trackers_[i].readImage(ptr->image, stamp.seconds(), publish_this_frame);
        }

        // 更新光流点 id
        for (size_t i = 0; i < camera_num_; i++) {
            trackers_[i].updateID();
        }
    }

    // 发送订阅数据部分
    {
        if (publish_this_frame) {
            publish_count_++;

            // 创建 PointCloud2 消息
            shared_ptr<PointCloud2> feature_points = std::make_shared<PointCloud2>();
            feature_points->header = img_msg->header;
            feature_points->header.frame_id = "world";
            feature_points->height = 1;
            feature_points->is_dense = false;

            // 清除 field 并添加字段
            feature_points->fields.clear();
            
            int offset = 0;
            auto addField = [&](const std::string& name, uint8_t data_type, uint32_t count) {
                sensor_msgs::msg::PointField field;
                field.name = name;
                field.offset = offset;
                field.datatype = datatype;
                field.count = count;
                feature_points->fields.push_back(field);
                offset += count * sizeof(float);        // 假设都是 float
            }

            for (size_t i = 0; i < camera_num_; i++) {
                // auto &un_pts = trackers_[i].cur_un_pts
                // for (size_t j = 0; j < )
            }

            // 构建特征点列表并发送

            // 发送含有光流点的图片
            // if ()
        }
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FeatureTrackerManager>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}