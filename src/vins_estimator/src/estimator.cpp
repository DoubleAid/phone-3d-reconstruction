#include "estimator.hpp"

// using namespace Logger;

Estimator::Estimator(int window_size) :
    window_size_(window_size + 1) {
    headers_.resize(window_size_ + 1);
    frame_count_ = 0;
    need_online_calibration_ = false;
    Rs.resize(window_size_ + 1);
    Vs.resize(window_size_ + 1);
    Ps.resize(window_size_ + 1);
    solver_flag_ = INITIAL;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::msg::Header &header) {
    Logger::info("new image comming ----------- current frame count: {}", frame_count_);
    // 判断特征点并检查时差 （判断是否为关键帧) frame_count_ 是当前帧的计数，也可以理解成当前帧的id
    if (f_manager.addFeatureCheckParallax(frame_count_, image)) {
        marginalization_flag_ = MARGIN_OLD;         // 如果视差足够大，标记为关键帧，边缘化旧帧
    } else {
        marginalization_flag_ = MARGIN_SECOND_NEW;  // 视差小，标记为非关键帧，边缘化上一帧
    }
    // 在非线性优化阶段，可以判断是接受该帧或者拒绝
    if (solver_flag_ == NON_LINERA) {
        Logger::info("this frame is --------------- {}", marginalization_flag_ != 0? "reject" : "accept");
    }
    headers_[frame_count_] = header;
    double seconds = rclcpp::Time(header.stamp).seconds();
    ImageFrame imageframe(image, seconds);

    all_image_frame_.insert(make_pair(seconds, imageframe));

    if (need_online_calibration_) {
        Logger::info("calibration");
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
            } else {
            }
            slideWindow();
        } else {
            frame_count_++;
        }
    } else {
        // 系统处于非线性优化阶段
        solveOdometry();
        slideWindow();
    }
}

bool Estimator::isPoseValid(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
    // 1. 检查旋转矩阵的正交性
    Eigen::Matrix3d should_be_identity = R * R.transpose();
    double ortho_error = (should_be_identity - Eigen::Matrix3d::Identity()).norm();
    if (ortho_error > 1e-3) {
        Logger::error("旋转矩阵非正交");
        return false;
    }

    // 2. 检查旋转矩阵行列式 (应该是 加减1)
    double det = R.determinant();
    if (std::abs(det - 1.0) > 1e-3) {
        Logger::error("旋转矩阵行列式非1: det = %f", det);
        return false;
    }

    // 3. 检查平移向量的合理性 (不能太大)
    double t_norm = t.norm();
    if (t_norm > 100.0 || t_norm < 1e-6) {
        Logger::error("位移向量长度错误: norm = %f", t_norm);
        return false;
    }
    return true;
}

bool Estimator::initialStructure() {
    Logger::info("视觉里程计初始化 ...");
    if (!visualInitialAlign()) {
        Logger::error("misalign visual structure");
        return false;
    }
    return true;
}

bool Estimator::visualInitialAlign() {
    // 便利当前窗口里的所有帧，挑选出一个追踪的特征点最多的帧作为参考帧
    // 其他帧的位姿都是相对与该帧的，
    // 对其他所有帧求解相对位姿
    // 步骤一：选择参考帧
    int ref_frame = selectReferenceFrame();
    if (ref_frame < 0) {
        Logger::error("No suitable reference frame found");
        return false;
    }
    Logger::info("Selected reference frame: %d", ref_frame);

    // 步骤二：初始化参考帧位姿
    Rs[ref_frame] = Eigen::Matrix3d::Identity();
    Ps[ref_frame] = Eigen::Vector3d::Zero();

    // 步骤三：计算其他帧相对于参考帧的位姿
    vector<bool> pose_success(frame_count_ + 1, false);
    pose_success[ref_frame] = true;

    for (int i = 0; i <= frame_count_; i++) {
        if (i == ref_frame) continue;
        if (estimateRelativePose(ref_frame, i)) {
            pose_success[i] = true;
            Logger::info("Frame {} pose estimated successfully", i);
        }
    }

    // 步骤四：三角化特征点
    int triangulated_points = triangulateAllPoints(ref_frame);
    if (triangulated_points < 20) {
        Logger::info("visual initialization failed: too few feature point os ");
        return false;
    }

    // 步骤五：全局BA优化
    if (!globalBundleAdjustment(ref_frame)) {
        Logger::info("全局BA优化失败");
        return false;
    }

    return true;
}

bool Estimator::estimateRelativePose(int ref_frame, int target_frame) {
    vector<cv::Point2f> pts_ref, pts_target;
    for (const auto& feat : f_manager.feature_) {
        if (feat.start_frame <= ref_frame && feat.start_frame <= target_frame &&
            feat.endFrame() >= ref_frame && feat.endFrame() >= target_frame) {
            int idx1 = ref_frame - feat.start_frame;
            int idx2 = target_frame - feat.start_frame;

            const auto& pt1 = feat.feature_per_frame[idx1];
            const auto& pt2 = feat.feature_per_frame[idx2];

            pts_ref.push_back(cv::Point2f(pt1.uv.x(), pt1.uv.y()));
            pts_target.push_back(cv::Point2f(pt2.uv.x(), pt2.uv.y()));
        }
    }

    if (pts_ref.size() < 8) {
        Logger::error("没有足够的点进行八点法计算基础矩阵和本质矩阵");
    }

    // 估计本质矩阵
    cv::Mat E, mask;
    double focal_length = 1.0;
    cv::Point2d principal_point(0, 0);

    try {
        E = cv::findEssentialMat(pts_ref, pts_target, focal_length, principal_point,
                                 cv::RANSAC, 0.999, 1.0, mask);
    } catch (const cv::Exception& e) {
        Logger::error("opencv exception in findEssential mat: {}", e.what());
        return false;
    }

    if (E.empty()) {
        Logger::error("基础矩阵估计失败");
        return false;
    }

    // 从本质矩阵恢复位姿
    cv::Mat R, t;

    int inliers = cv::recoverPose(E, pts_ref, pts_target, R, t,
                                    focal_length, principal_point, mask);

    if (inliers < 8) {
        Logger::error("用于位姿恢复的内点数量太少：{} < 8", inliers);
        return false;
    }

    Logger::info("位姿恢复内点数目：{} / {}", inliers, pts_ref.size());

    // 转换到 Eigen 格式
    cv::cv2eigen(R, Rs[target_frame]);
    cv::cv2eigen(t, Ps[target_frame]);

    if (!isPoseValid(Rs[target_frame], Ps[target_frame])) {
        Logger::error("恢复的位姿无效");
        return false;
    }
    
    Logger::info("位姿估计成功，内点 {} / {}", inliers, pts_ref.size());
    return true;
}

bool Estimator::estimateCurrentPoseByPnP() {
    int current_frame = frame_count_;
    Logger::info("Estimating pose for frame %d using PnP", current_frame);

    // 手机3D - 2D 的对应点
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;

    int correspondences = 0;
    for (const auto& feat: f_manager.feature_) {
        // 如果当前这个特征都没有进行三角化，就先跳过
        if (feat.point.norm() < 1e-6) continue;

        // bool observed_in_current = false;
        int max_frame = feat.start_frame + feat.feature_per_frame.size() - 1;
        // 该特征点是否在当前帧被观测到
        if (max_frame >= current_frame) {
            int frame_idx = current_frame - feat.start_frame;
            object_points.push_back(cv::Point3f(
                feat.point.x(), feat.point.y(), feat.point.z()
            ));
            image_points.push_back(cv::Point2f(feat.feature_per_frame[frame_idx].uv.x(), feat.feature_per_frame[frame_idx].uv.y()));
            correspondences++;
        }
    }

    if (correspondences < 6) {
        Logger::error("Not enough 3D-2D correspondences for PnP: %d", correspondences);
        return false;
    }

    Logger::info("PnP: found %d 3D-2D correspondences", correspondences);

    // 求解 PnP
    cv::Mat r_vec, t_vec, inliers;
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

    try {
        bool success = cv::solvePnPRansac(object_points, image_points, camera_matrix,
                                          cv::Mat(), r_vec, t_vec, false, 100, 2.0, 0.99, inliers);
        if (!success) {
            Logger::error("PnP Ransac failed");
            return false;
        }

        // 转换到旋转矩阵
        cv::Mat R;
        cv::Rodrigues(r_vec, R);

        // 转换到Eigen格式
        cv::cv2eigen(R, Rs[current_frame]);
        cv::cv2eigen(t_vec, Ps[current_frame]);

        int num_inliers = inliers.rows;
        Logger::info("PnP success: %d inliers / %d points", num_inliers, correspondences);

        return num_inliers >= 6;

    } catch (const cv::Exception& e) {
        Logger::error("OpenCV exception in PnP: %s", e.what());
        return false;
    }
}

int Estimator::selectReferenceFrame() {
    int best_frame = -1;
    double best_score = -1;

    for (int i = 0; i <= frame_count_; i++) {
        // 综合评分 = 跟踪点数 x 视差质量 x 共视帧数
        double score = calculateFrameScore(i);
        if (score > best_score) {
            best_score = score;
            best_frame = i;
        }
    }
    return best_frame;
}

double Estimator::calculateFrameScore(int frame_id) {
    int track_count = countTrackInFrame(frame_id);
    double track_score = static_cast<double>(track_count);

    // 暂时不计算平均视差
    double avg_parallax = calculateAverageParallax(frame_id);
    double parallax_score = avg_parallax;

    int covisibility_count = countCovisibilityFrames(frame_id);
    double covisibility_score = static_cast<double>(covisibility_count);

    return 0.5 * track_score + 0.5 * covisibility_score;
}

// 计算追踪点数
int Estimator::countTrackInFrame(int frame_id) {
    int count = 0;
    for (const auto& feat : f_manager.feature_) {
        if (feat.start_frame <= frame_id && feat.endFrame() >= frame_id) {
            count++;
        }
    }
    return count;
}

// 计算共视帧数权重
int Estimator::countCovisibilityFrames(int frame_id) {
    // 遍历所有特征点，找到与当前帧有共视关系的其他所有帧
    std::vector<int> covisibility_frames;
    int min_shared_features = 15;
    int strong_covisibility_count = 0;
    int start_frame = frame_id, end_frame = frame_id;
    for (const auto& feat : f_manager.feature_) {
        if (feat.start_frame <= frame_id && feat.endFrame() >= frame_id) {
            for (int j = feat.start_frame; j <= feat.endFrame(); j++) {
                covisibility_frames.emplace_back(j);
                if (j > end_frame) end_frame = j;
                if (j < start_frame) start_frame = j;
            }
        }
    }
    for (int i = start_frame; i <= end_frame; i++) {
        if (i == frame_id) continue;
        int cn = count(covisibility_frames.begin(), covisibility_frames.end(), i);
        if (cn >= min_shared_features) {
            strong_covisibility_count++;
        }
    }
    return strong_covisibility_count;
}

// 计算平均视差权重
double Estimator::calculateAverageParallax(int frame_id) {
    return 0;
}

void Estimator::slideWindow() {
    if (frame_count_ != window_size_) {
        return;
    }
    if (marginalization_flag_ == MARGIN_OLD) {
        // 边缘化R[0] 和 P[0]
        Ps.pop_front();
        Rs.pop_front();
    } else {
        headers_[frame_count_-1] = headers_[frame_count_];
        Ps[frame_count_-1] = Ps[frame_count_];
        Rs[frame_count_-1] = Rs[frame_count_];
        Vs[frame_count_-1] = Vs[frame_count_];
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
        // 1. 使用 PNP 估计当前帧
        if (!estimateCurrentPoseByPnP()) {
            Logger::error("PnP failed, 尝试通过八点法或者三角化求解相对位姿");
        }
    }

    // 三角化新的特征点

    // 局部优化
    optimization();

    // 移除外点
}

void Estimator::optimization() {

}

int Estimator::triangulateAllPoints(int ref_frame) {
    Logger::info("尝试使用参考帧位姿三角化特征点");
    int success_count = 0;
    for (auto& feat : f_manager.feature_) {
        if (feat.start_frame <= ref_frame && feat.endFrame() >= ref_frame) {
            Eigen::Vector3d point_3d;
            if (triangulatePoint(feat, point_3d)) {
                success_count++;
                feat.point = point_3d;
            }
        }
    }
    Logger::info("成功三角化 {} 个点", success_count);
    return success_count;
}

bool Estimator::triangulatePoint(const FeaturePerId& feature, Eigen::Vector3d& point_3d) {
    // 使用最小二乘法进行三角化
    // 已经知道位姿 和 点在图像上的投影，求解点的位置

    // 假设 P = K[R | t] 为 一个 3 x 4 的矩阵 设为 P = [p1^T, p2^T, p3^T]^T
    // 设 特征点的空间坐标 X = [x, y, z, 1]^T
    // (p_1 - u p_3)X = 0
    // (p_2 - v p_3)X = 0

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();

    int frame_length = feature.feature_per_frame.size();
    for (size_t i = 0; i < frame_length; i++) {
        int frame_idx = i + feature.start_frame;

        Eigen::Matrix3d R = Rs[frame_idx];
        Eigen::Vector3d t = Ps[frame_idx];
        Eigen::Vector3d pt = feature.feature_per_frame[i].point;

        Eigen::Vector3d u(pt.x() / pt.z(), pt.y() / pt.z(), 1.0);
        Eigen::Matrix<double, 3, 4> P;
        P.leftCols<3>() = R.transpose();
        P.rightCols<1>() = -R.transpose() * t;  // P = [R^T | -R^T*t]

        Eigen::Matrix<double, 2, 4> eq;
        eq.row(0) = u(0) * P.row(2) - P.row(0); // eq.x = x * p3 - p1
        eq.row(1) = u(1) * P.row(2) - P.row(1); // eq.y = y * p3 - p2

        A += eq.transpose() * eq;
    }

    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d point_homo = svd.matrixV().col(3);

    if (std::abs(point_homo(3)) < 1e-8) {
        return false;
    }

    point_3d = point_homo.head<3>() / point_homo(3);

    // 检查深度为正
    if (point_3d(2) < 0) {
        return false;
    }
    return true;
}

bool Estimator::globalBundleAdjustment(int ref_frame) {
    return true;
}