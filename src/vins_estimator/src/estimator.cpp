#include "estimator.hpp"

Estimator::Estimator(int window_size) :
    window_size_(window_size) {
    headers_.resize(window_size_);
    frame_count_ = 0;
    need_online_calibration_ = false;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header) {
    Logger::info("new image comming ----------------------------");

    // 判断特征点并检查时差 （判断是否为关键帧) frame_count_ 是当前帧的计数，也可以理解成当前帧的id
    if (f_manager.addFeatureCheckParallax(frame_count_, image)) {
        marginalization_flag_ = MARGIN_OLD;         // 如果视差足够大，标记为关键帧，边缘化旧帧
    } else {
        marginalization_flag_ = MARGIN_SECOND_NEW;  // 视差小，标记为非关键帧，边缘化上一帧
    }

    Logger::debug("this frame is --------------- %s", marginalization_flag_ ? "reject" : "accept");

    headers_[frame_count_] = header;
    double seconds = rclcpp::Time(header.stamp).seconds();
    ImageFrame imageframe(image, seconds);

    all_image_frame_.insert(make_pair(seconds, imageframe));

    if (need_online_calibration_) {
        // 主要是标定 imu 和相机之间的外参
    }

    // 系统处于初始化状态
    if (solver_flag_ == INITIAL) {
        // 滑动窗口已满
        if (frame_count_ == window_size_) {
            bool result = false;
            if (!need_online_calibration_) {
                result = initialStructure();
            }
            if (result) {
                solver_flag_ = NON_LINERA;
                // 非线性优化
                solveOdometry();
                slideWindow();
            } else {
                slideWindow();
            }
        } else {
            frame_count_++;
        }
    } else {
        // 系统处于非线性优化阶段
        solveOdometry();
        slideWindow();
    }
}

bool Estimator::initialStructure() {

}

void Estimator::slideWindow() {
    if (marginalization_flag_ == MARGIN_OLD) {

    } else {
        if (frame_count_ == window_size_) {
            slideWindowNew();
        }
    }
}

void Estimator::slideWindowNew() {

}

void Estimator::slideWindowOld() {

}

void Estimator::solveOdometry() {
    if (frame_count_ < window_size_)
        return;
    if (solver_flag_ == NON_LINERA) {
        f_manager.
    }
}

void Estimator::optimization() {

}
