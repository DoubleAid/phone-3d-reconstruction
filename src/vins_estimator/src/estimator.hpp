#pragma once
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
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

    bool estimateRelativePose(int ref_frame, int target_frame);
    bool estimateCurrentPoseByPnP();
    bool globalBundleAdjustment(int ref_frame);
    bool initialStructure();
    bool isPoseValid(const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
    bool triangulatePoint(const FeaturePerId& feat_per_frame, Eigen::Vector3d& point_3d);
    bool visualInitialAlign();

    void clearState();
    void optimization();
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header);
    void slideWindow();
    void slideWindowNew();
    void slideWindowOld();
    void solveOdometry();

    int countTrackInFrame(int frame_id);
    int countCovisibilityFrames(int frame_id);
    int selectReferenceFrame();
    int triangulateAllPoints(int ref_frame);

    double calculateFrameScore(int frame_id);
    double calculateAverageParallax(int frame_id);

private:
    bool need_online_calibration_;              // 需要在线标定外参
    int window_size_;                           // 窗口长度
    int frame_count_;                           // 当前滑动窗口中的帧数

    map<double, ImageFrame> all_image_frame_;   // 时间和帧的映射

    deque<Vector3d> key_poses_;                // 关键帧的位姿
    deque<Vector3d> Ps;                        // 窗口每个帧的相机位置
    deque<Vector3d> Vs;                        // 窗口每个帧的旋转速度
    deque<Matrix3d> Rs;                        // 窗口每个帧的旋转矩阵

    deque<std_msgs::msg::Header> headers_;

    FeatureManager f_manager;
    MarginalizationFlag marginalization_flag_;
    SolverFlag solver_flag_;
};
