#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "logger.hpp"
#include "tic_toc.hpp"

using namespace std;
using namespace common;

class FeatureTracker {
public:
    FeatureTracker(bool equalize = true, int min_dist = 10);

    ~FeatureTracker();

    void readImage(const cv::Mat &image, double cur_time, bool publish);

    bool updateId(unsigned int i);

    void setMask();

    void addPoints();

    void rejectWithF();

private:
    bool equalize_;             // 是否进行直方图均衡化

    cv::Mat mask_;              // 特征检测掩码，控制新特征点的生成位置。
    cv::Mat prev_img_;          // 前一帧
    cv::Mat cur_img_;           // 当前处理帧
    cv::Mat forw_img_;          // 后一帧

    vector<cv::Point2f> n_pts_;
    vector<cv::Point2f> prev_pts_, cur_pts_, forw_pts_;     // 特征点坐标
    vector<cv::Point2f> prev_un_pts_, cur_un_pts_;          // 归一化特征点
    vector<cv::Point2f> pts_velocity_;                      // 特征点速度
    
    vector<int> ids_;
    vector<int> track_cnt_;

    map<int, cv::Point2f> cur_un_pts_map_;
    map<int, cv::Point2f> prev_un_pts_map_;
    
    double cur_time_;
    double prev_time_;

    int max_feature_cnt_;
    int min_feature_dist_;
    int cols_;                  // 当前图像的宽
    int rows_;                  // 当前图像的高

    static int n_id_;
};
