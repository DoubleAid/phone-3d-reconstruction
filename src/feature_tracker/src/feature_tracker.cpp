#include "feature_tracker.hpp"

int FeatureTracker::n_id_ = 0;

FeatureTracker::FeatureTracker(bool equalize, int min_dist, int max_feature_cnt) :
        equalize_(equalize),
        min_feature_dist_(min_dist),
        max_feature_cnt_(max_feature_cnt) {
    
}

FeatureTracker::~FeatureTracker() {
    Logger::info("feature tracker deinit");
}

void FeatureTracker::readImage(const cv::Mat &_img, double cur_time, bool publish) {
    cv::Mat img;

    // 如果需要均衡化，增强图片的对比度
    if (equalize_) {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(_img, img);
        Logger::info("clahe equalize finished");
    } else {
        img = _img;
    }

    // 更新图片的尺寸
    cols_ = img.cols;
    rows_ = img.rows;
    Logger::info("更新图片尺寸 [{}, {}]", cols_, rows_);

    // 如果前一帧是空的，也就是没有初始化，就先进性初始化
    if (forw_img_.empty()) {
        prev_img_ = cur_img_ = forw_img_ = img;
    }
    else
    {
        forw_img_ = img;
    }

    // 清除前一帧的特征点
    forw_pts_.clear();

    // 如果比较帧的特征点不为0， 就将当前帧和比较帧进行比较，更新特征点
    if (cur_pts_.size() > 0) {
        vector<uchar> status;       // 跟踪成功1， 失败0
        vector<float> err;          // 跟踪误差

        // 使用 LK 光流跟踪: 从 cur_img 到 forw_img 来根据 forw_pts 点追踪获取光流的特征点 cur_pts
        // status 用来表明一个特征点是否追踪到点
        cv::calcOpticalFlowPyrLK(cur_img_, forw_img_, cur_pts_, forw_pts_, status, err, cv::Size(21, 21), 3);
        for (int i = 0; i < int(forw_pts_.size()); i++) {
            // 判断被追踪点是否追踪到了，如果追踪到了，但是是边缘点，这个点也不要，因为光流点靠近图像边缘，很可能不准确
            if (status[i] && !inBorder(forw_pts_[i])) {
                status[i] = 0;
            }
        }
        reduceVector(prev_pts_, status);
        reduceVector(cur_pts_, status);
        reduceVector(forw_pts_, status);
        reduceVector(ids_, status);
        reduceVector(cur_un_pts_, status);
        reduceVector(track_cnt_, status);
    }

    // 发送当前帧
    if (publish) {
        rejectWithF();
        setMask();
        int n_max_cnt = max_feature_cnt_ - forw_pts_.size();
        if (n_max_cnt > 0) {
            if (mask_.empty())
                Logger::info("mask is empty");
            Logger::info("额外增加 {} 个光流点", n_max_cnt);
            cv::goodFeaturesToTrack(forw_img_, n_pts_, n_max_cnt, 0.01, min_feature_dist_, mask_);
        } else {
            n_pts_.clear();
        }
        addPoints();
    }
    prev_img_ = cur_img_;
    prev_pts_ = cur_pts_;
    prev_un_pts_ = cur_un_pts_;
    cur_img_ = forw_img_;
    cur_pts_ = forw_pts_;
    prev_time_ = cur_time_;
}

void FeatureTracker::updateID() {
    for (size_t i = 0; i < ids_.size(); i++) {
        if (ids_[i] == -1) {
            ids_[i] = n_id_++;
        }
    }
}

void FeatureTracker::setMask() {
    mask_ = cv::Mat(rows_, cols_, CV_8UC1, cv::Scalar(255));
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts_.size(); i++) {
        cnt_pts_id.push_back(make_pair(track_cnt_[i], make_pair(forw_pts_[i], ids_[i])));
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

void FeatureTracker::addPoints() {
    for (auto &p : n_pts_) {
        forw_pts_.push_back(p);
        ids_.push_back(-1);
        track_cnt_.push_back(1);
    }
}

void FeatureTracker::rejectWithF() {

}

bool FeatureTracker::inBorder(cv::Point2f point) {
    Logger::info("point value [{}, {}] 图片尺寸 [{}, {}]", point.x, point.y, cols_, rows_);
    return true;
}

// 根据 status 删除部分节点
void FeatureTracker::reduceVector(vector<cv::Point2f> &v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++) {
        if (status[i])
            v[j++] = v[i];
    }
    v.resize(j);
}

void FeatureTracker::reduceVector(vector<int> &v, vector<uchar> status) {
    int j = 0;
    for (int i = 0; i < int(v.size()); i++) {
        if (status[i])
            v[j++] = v[i];
    }
    v.resize(j);
}
