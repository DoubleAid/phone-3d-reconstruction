#pragma once
#include <list>
#include <vector>
#include <numeric>
#include <algorithm>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <std_msgs/msg/header.hpp>
#include <map>

using namespace std;
using namespace Eigen;

class FeaturePerFrame {
public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point) {
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.x() = _point(5);
        velocity.y() = _point(6);
    }
    Vector3d point;
    Vector2d uv;
    Vector2d velocity;
    bool is_used;
    double parallax;
};

class FeaturePerId {
public:
    const int feature_id;
    int start_frame;
    vector<FeaturePerFrame> feature_per_frame;

    int used_num;

    FeaturePerId(int _feature_id, int _start_frame) : 
        feature_id(_feature_id), start_frame(_start_frame),
        used_num(0) {

    }

    int endFrame();
};

class FeatureManager {
public:
    FeatureManager();
    int getFeatureCount();
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7,1>>>> &image);
    void clearState();

private:
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
    list<FeaturePerId> feature_;
    int last_track_num_;
};
