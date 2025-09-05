#include "feature_tracker.hpp"

FeatureTracker::FeatureTracker(bool equalize, int min_dist) :
        equalize_(equalize),
        max_feature_cnt_(0) {
    
}

FeatureTracker::~FeatureTracker() {
    Logger::info("feature tracker deinit");
}

void FeatureTracker::readImage(const cv::Mat &_img, double cur_time, bool publish) {
    cv::Mat img;
    if (equalize_) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(_img, img);
        Logger::info("clahe equalize finished");
    } else {
        img = _img;
    }

    cols_ = img.cols;
    rows_ = img.rows;

    if (forw_img_.empty()) {
        prev_img_ = cur_img_ = forw_img_ = img;
    }
    else
    {
        forw_img_ = img;
    }

    forw_pts_.clear();

    if (cur_pts_.size() > 0) {
        vector<uchar> status;       // 跟踪成功1， 失败0
        vector<float> err;          // 跟踪误差

        cv::calcOpticalFlowPyrLK(cur_img_, forw_img_, cur_pts_, forw_pts_, status, err, cv::Size(21, 21), 3);

        for (int i = 0; i < int(forw_pts_.size()); i++) {
            // if (status[i] && !inBorder())
        }
    }

    // 发送当前帧
    if (publish) {
        setMask();
        int n_max_cnt = max_feature_cnt_ - forw_pts_.size();
        if (n_max_cnt > 0) {
            cv::goodFeaturesToTrack(forw_img_, n_pts_, n_max_cnt, 0.01, min_feature_dist_, mask_);
        } else {
            n_pts_.clear();
        }
        addPoints()
    }
}

bool FeatureTracker::updateId(unsigned int i) {
    return true;
}

void FeatureTracker::setMask() {
    mask_ = cv::Mat(rows_, cols_, CV_8UC1, cv::Scalar(255));
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts_.size(); i++) {
        // cnt_pts_id.push_back(make_pair())
    }
    ids_.clear();
    forw_pts_.clear();
    track_cnt_.clear();

    for (auto &it : cnt_pts_id) {
        // 检查当前点是否在允许的区域内
        if (mask_.at<uchar>(it.second.first) == 255) {
            // 保留该点
            forw_pts_.push_back(it.second.first);
            ids_.push_back(it.second.second);
            track_cnt_.push_back(it.first);
            // 在掩码上屏蔽周围区域， 防止重复检测
            cv::circle(mask_, it.second.first, min_feature_dist_, 0, -1);
        }       
    }
}
