#pragma once
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "feature_manager.hpp"
#include "logger.hpp"

using namespace std;
using namespace common;

enum SolverFlag {
    INITIAL,            // 初始化状态
    NON_LINERA          // 非线性优化
};

// 边缘化策略
enum MarginalizationFlag {
    MARGIN_OLD          = 0,    // 优化最老帧
    MARGIN_SECOND_NEW   = 1,    // 优化上一帧
};

class Estimator {
public:
    Estimator(int window_size);

    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header);

    void clearState();

private:
    FeatureManager f_manager;

    int window_size_;
    // 当前滑动窗口中的帧数
    int frame_count_;

    vector<std_msgs::msg::Header> headers_;

    SolverFlag solver_flag_;

    MarginalizationFlag marginalization_flag_;
};
