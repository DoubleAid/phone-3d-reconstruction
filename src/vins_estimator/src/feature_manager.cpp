#include "feature_manager.hpp"

FeatureManager::FeatureManager() {

}

bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7,1>>>> &image) {
    double parallax_sum = 0;        // 视差和
    int parallax_num = 0;           // 视差点数
    last_track_num_ = 0;            // 当前帧成功追踪的特征点数目
    // 遍历里面的每一个特征点
    for (auto&id_pts: image) {
        FeaturePerFrame f_per_frame(id_pts.second[0].second);
        int feature_id = id_pts.first;
        // 查找具有相同 feature_id, 也就是连续的追踪特征点
        auto it = std::find_if(feature_.begin(), feature_.end(), [feature_id](const FeaturePerId &it) {
            return it.feature_id == feature_id;
        });
        // 没找到就把这个点放到 特征点列表里
        if (it == feature_.end()) {
            feature_.push_back(FeaturePerId(feature_id, frame_count));
            feature_.back().feature_per_frame.push_back(f_per_frame);
        } else {
            // 将该特征点加入到观测
            it->feature_per_frame.push_back(f_per_frame);
            last_track_num_++;
        }
    }
    // 如果滑动窗口的帧数太少，或者当前帧的追踪数目小于20
    if (frame_count < 2 || last_track_num_ < 20) {
        // 强制视为关键帧，确保系统能够快速初始化或回复跟踪
        return true;
    }
    for (auto& it_per_id: feature_) {
        // 要满足两个条件
        //  1. 起始帧和当前帧的间隔大于等于2， 也就是至少要有三帧，可以用来计算视差
        //  2. 这个特征从start_frame一直持续到当前帧
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) >= frame_count) {
            
        }
    }

    return true;
}

void FeatureManager::clearState() {

}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count) {
    // frame_i = frame_per_frame[-3]
    // frame_j = frame_per_frame[-2]
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;
    Vector3d p_i_comp;

    
}