#pragma once
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "feature_manager.hpp"
#include "logger.hpp"

#include <unordered_map>

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
    Estimator(int window_size = 10);

    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header);

    void clearState();

    bool initialStructure();
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void solveOdometry();
    void optimization();

private:
    FeatureManager f_manager;
    // 需要在线标定外参
    bool need_online_calibration_;

    int window_size_;
    // 当前滑动窗口中的帧数
    int frame_count_;

    map<double, ImageFrame> all_image_frame_;

    vector<Vector3d> key_poses_;

    vector<std_msgs::msg::Header> headers_;

    SolverFlag solver_flag_;

    MarginalizationFlag marginalization_flag_;
};
