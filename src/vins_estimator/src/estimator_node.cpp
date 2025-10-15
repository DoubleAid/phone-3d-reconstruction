#include <rclcpp/rclcpp.hpp>
#include <map>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "estimator.hpp"

using namespace std;
using namespace std_msgs::msg;
using namespace sensor_msgs::msg;
using namespace common;

using PointCloudPtr = sensor_msgs::msg::PointCloud2::SharedPtr;

#define X(a, b) a + b

class VinsEstimatorManager : public rclcpp::Node {
public:
    VinsEstimatorManager();
    ~VinsEstimatorManager();
    void initialize();
    void featureCallback(const PointCloudPtr msg);
    void restartCallback(const std_msgs::msg::Bool::SharedPtr restart_msg);
    void process();
    std::vector<PointCloudPtr> getMeasurements();

private:
    bool feature_init_;
    std::mutex feature_mutex_;
    std::condition_variable feature_available_;
    std::thread measurement_thread_;

    std::queue<PointCloudPtr> feature_queue_;

    rclcpp::Subscription<Bool>::SharedPtr restart_subscriptor_;
    // 特征点数据
    rclcpp::Subscription<PointCloud2>::SharedPtr feature_subscriptor_;
    // 回环检测订阅点
    rclcpp::Subscription<PointCloud2>::SharedPtr relocal_subscriptor_;
};

VinsEstimatorManager::VinsEstimatorManager() 
    : Node("vins_estimator"), feature_init_(false) {
    declare_parameter("camera_num", 1);
    
    feature_subscriptor_ = this->create_subscription<PointCloud2>(
        "/feature", 
        100,
        std::bind(&VinsEstimatorManager::featureCallback, this, std::placeholders::_1)
    );

    restart_subscriptor_ = this->create_subscription<std_msgs::msg::Bool>(
        "/restart_signal",
        100,
        std::bind(&VinsEstimatorManager::restartCallback, this, std::placeholders::_1)
    );
}

VinsEstimatorManager::~VinsEstimatorManager() {
    if (measurement_thread_.joinable()) {
        measurement_thread_.join();
    }
}

void VinsEstimatorManager::initialize() {
    Logger::init(this->shared_from_this());
    Logger::info("vins estimator node initialize");
    measurement_thread_ = std::thread([this]() {
        this->process();
    });
}

void VinsEstimatorManager::featureCallback(const PointCloudPtr msg) {
    Logger::info("收到 feature 消息：");
    Logger::info("      帧id : {}", msg->header.frame_id);
    Logger::info("      点数 : 宽 {} X 高 {}", msg->width, msg->height);
    if (!feature_init_) {
        // 跳过第一帧，因为第一帧不包含光流点数据
        feature_init_ = true;
        return;
    }
    {
        std::lock_guard<std::mutex> lock(feature_mutex_);
        feature_queue_.push(msg);
    }
    feature_available_.notify_one();
}

void VinsEstimatorManager::restartCallback(const std_msgs::msg::Bool::SharedPtr restart_msg) {
    if (restart_msg->data) {
        Logger::info("开始执行重启操作 ...");
    } else {
        Logger::info("收到重启信号为 false, 不执行操作");
    }
}

void VinsEstimatorManager::process() {
    Logger::info("Vins Estimator Manager processing ..."); 
    while (true) {
        std::unique_lock<std::mutex> lock(feature_mutex_);
        std::vector<PointCloudPtr> measurements;
        feature_available_.wait(lock, [&] {
            measurements = getMeasurements();
            return measurements.size() != 0;
        });
        lock.unlock();

        for (auto &measurement : measurements) {
            auto img_msg = measurement;
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            sensor_msgs::PointCloud2Iterator<float> iter_x(*img_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(*img_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(*img_msg, "z");
            sensor_msgs::PointCloud2Iterator<float> iter_id(*img_msg, "id");
            sensor_msgs::PointCloud2Iterator<float> iter_u(*img_msg, "u");
            sensor_msgs::PointCloud2Iterator<float> iter_v(*img_msg, "v");
            sensor_msgs::PointCloud2Iterator<float> iter_vx(*img_msg, "velocity_x");
            sensor_msgs::PointCloud2Iterator<float> iter_vy(*img_msg, "velocity_y");

            for (size_t i = 0; i < img_msg->width; i++) {
                float x = *iter_x; float y = *iter_y; float z = *iter_z; float id = *iter_id;
                float u = *iter_u; float v = *iter_v; float vx = *iter_vx; float vy = *iter_vy;
                // 解析 id

                // Eigen::Matrix<double, 7, 1>
                ++iter_x; ++iter_y; ++iter_z; ++iter_id;
                ++iter_u; 
            }
        }
    }
}

std::vector<PointCloudPtr> VinsEstimatorManager::getMeasurements() {
    std::vector<PointCloud2::SharedPtr> measurements;
    if (!feature_queue_.empty()) {
        measurements.emplace_back(feature_queue_.front());
        feature_queue_.pop();
    }
    return measurements;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VinsEstimatorManager>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
