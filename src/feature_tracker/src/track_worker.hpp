#pragma once
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include "logger.hpp"

using namespace std;
using namespace common;

class TrackWorker {
public:
    TrackWorker();

    void readImage(const cv::Mat &image, double cur_time);
    bool updateID(unsigned int i);

    bool equalize_;
    // 前一帧，当前处理帧， 新入的后一帧
    cv::Mat prev_img_, cur_img_, forw_img_;

    vector<cv::Point2f> n_pts_;
    vector<cv::Point2f> prev_pts_, cur_pts_, forw_pts_;
};

bool isBorder(const cv::Point2f &pt);
