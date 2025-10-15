#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "feature_tracker.hpp"

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
    declare_parameter("camera_global.camera_num", 1);                 // 相机数量
    declare_parameter("topic_name", "video_frames");    // 订阅的图片话题
    declare_parameter("equalize", true);                // 是否对图片进行直方图均衡化
    declare_parameter("max_feature_count", 150);        // 每一帧最大的特征点数
    declare_parameter("publish_frequence", 20);         // 发送频率，每秒钟发送的帧的个数
    declare_parameter("min_feature_dist", 20);          // 两个特征点之间的最小距离
    declare_parameter("show_track", true);              // 显示包含光流追踪点的图片

    camera_num_             = get_parameter("camera_global.camera_num").as_int();
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
    init_publish_           = false;

    for (size_t i = 0; i < camera_num_; i++) {
        trackers_.emplace_back(equalize, min_dist, max_feature_cnt);
    }
}

void FeatureTrackerManager::initialize() {
    Logger::init(this->shared_from_this());
    Logger::info("feature tracker manager initialize");

    declare_parameter("camera_calibration.cx", 10);
    int cx = get_parameter("camera_calibration.cx").as_int();
    Logger::info("cx is {}", cx);
    Logger::info("camera num {}", camera_num_);
    camera_num_ = 1;

    // 订阅图片消息并绑定相应函数
    image_transport::ImageTransport it(shared_from_this());
    image_subscriber_ = it.subscribe(
        sub_image_topic_,   // 话题名称
        100,                // 队列大小
        std::bind(&FeatureTrackerManager::imageCallback, this, std::placeholders::_1)
    );

    // 发布包含光流追踪点
    match_publisher_ = it.advertise("track_image", 1);

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
    cv::Mat show_img = ptr->image;

    // 这点是处理多个相机的问题，暂时不需要
    for (size_t i = 0; i < camera_num_; i++) {
        trackers_[i].readImage(ptr->image, stamp.seconds(), publish_this_frame);
    }

    // 更新光流点 id
    for (size_t i = 0; i < camera_num_; i++) {
        trackers_[i].updateID();
    }

    // 发送订阅数据部分
    {
        if (publish_this_frame) {
            publish_count_++;

            // 创建 PointCloud2 消息
            shared_ptr<PointCloud2> feature_points = std::make_shared<PointCloud2>();
            feature_points->header = img_msg->header;
            feature_points->header.frame_id = "odom";
            feature_points->height = 1;
            feature_points->is_dense = false;

            // 清除 field 并添加字段
            feature_points->fields.clear();
            
            int offset = 0;
            auto addField = [&](const std::string& name, uint8_t data_type, uint32_t count) {
                sensor_msgs::msg::PointField field;
                field.name = name;
                field.offset = offset;
                field.datatype = data_type;
                field.count = count;
                feature_points->fields.push_back(field);
                offset += count * sizeof(float);        // 假设都是 float
            };

            addField("x", PointField::FLOAT32, 1);
            addField("y", PointField::FLOAT32, 1);
            addField("z", PointField::FLOAT32, 1);
            addField("id", PointField::FLOAT32, 1);
            addField("u", PointField::FLOAT32, 1);
            addField("v", PointField::FLOAT32, 1);
            addField("velocity_x", PointField::FLOAT32, 1);
            addField("velocity_y", PointField::FLOAT32, 1);

            // 在这里看作 所有点作为一排，也就是 height = 1
            // 设置每一个点的步长
            feature_points->point_step = offset;
            feature_points->row_step = feature_points->point_step;

            size_t total_points = 0;
            for (size_t i = 0; i < camera_num_; i++) {
                for (size_t j = 0; j < trackers_[i].ids_.size(); j++) {
                    if (trackers_[i].track_cnt_[j] > 1) {
                        total_points++;
                    }
                }
            }

            feature_points->width = total_points;
            feature_points->data.resize(total_points * feature_points->point_step);

            // 获取迭代器指向响应的数据块
            sensor_msgs::PointCloud2Iterator<float> iter_x(*feature_points, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*feature_points, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*feature_points, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_id(*feature_points, "id");
            sensor_msgs::PointCloud2Iterator<float> iter_u(*feature_points, "u");
            sensor_msgs::PointCloud2Iterator<float> iter_v(*feature_points, "v");
            sensor_msgs::PointCloud2Iterator<float> iter_vx(*feature_points, "velocity_x");
            sensor_msgs::PointCloud2Iterator<float> iter_vy(*feature_points, "velocity_y");

            for (size_t i = 0; i < camera_num_; i++) {
                auto &un_pts        = trackers_[i].cur_un_pts_;
                auto &cur_pts       = trackers_[i].cur_pts_;
                auto &ids           = trackers_[i].ids_;
                auto &pts_velocity  = trackers_[i].pts_velocity_;

                for (size_t j = 0; j < ids.size(); j++) {
                    if (trackers_[i].track_cnt_[j] > 1) {
                        int p_id = ids[j];
                        *iter_x = un_pts[j].x;
                        *iter_y = un_pts[j].y;
                        *iter_z = 1.0f;
                        *iter_id = p_id * camera_num_ + i;
                        *iter_u = cur_pts[j].x;
                        *iter_v = cur_pts[j].y;
                        *iter_vx = pts_velocity[j].x;
                        *iter_vy = pts_velocity[j].y;

                        ++iter_x; ++iter_y; ++iter_z; ++iter_id;
                        ++iter_u; ++iter_v; ++iter_vx; ++iter_vy;
                    }
                }
            }

            // 构建特征点列表并发送
            if (!init_publish_) {
                init_publish_ = true;
            } else {
                feature_publisher_->publish(*feature_points);
            }

            // 发送含有光流点的图片
            if (show_track_) {
                ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
                cv::Mat stereo_img = ptr->image;
                for (size_t i = 0; i < camera_num_; i++) {
                    cv::Mat tmp_img = stereo_img;
                    cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                    for (size_t j = 0; j < trackers_[i].cur_pts_.size(); j++) {
                        double len = std::min(1.0, 1.0 * trackers_[i].track_cnt_[j]);
                        cv::circle(tmp_img, trackers_[i].cur_pts_[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                    }
                }
                match_publisher_.publish(ptr->toImageMsg());
            }
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