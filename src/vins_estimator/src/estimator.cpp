#include "estimator.hpp"

Estimator::Estimator() {

}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header) {
    Logger::info("new image comming ----------------------------");

    // 判断特征点并检查时差 （判断是否为关键帧) frame_count_ 是当前帧的计数，也可以理解成当前帧的id
    if (f_manager.addFeatureCheckParallax(frame_count_, image)) {

    } else {
        
    }

}
