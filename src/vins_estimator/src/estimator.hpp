#pragma once
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "feature_manager.hpp"
#include "logger.hpp"

using namespace std;

class Estimator {
public:
    Estimator();

    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header);

    void clearState();

private:
    FeatureManager f_manager;

    // 当前滑动窗口中的帧数
    int frame_count_;
};
