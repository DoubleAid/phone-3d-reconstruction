#include <ceres/ceres.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <cmath>
#include <random>

using namespace std;

class Pose2D {
public:
    double x;
    double y;
    double theta; // 旋转角度，单位：弧度

    Pose2D(double _x = 0, double _y = 0, double _theta = 0)
        : x(_x), y(_y), theta(_theta) {}
};

class Line {
public:
    double d_;
    Eigen::Vector2d normal_;

    Line(Eigen::Vector2d normal, double d) {
        normal_ = normal;
        d_ = d / normal.norm();
        normal_.normalize();
    }

    Line(const Line& line) {
        d_ = line.d_;
        normal_ = line.normal_;
    }

    double distance(const Eigen::Vector2d& point) const {
        return abs(normal_.dot(point) + d_);
    }

    double yIntercept(double x) const {
        if (abs(normal_(1)) < 1e-8) {
            throw std::runtime_error("Line is vertical; y-intercept is undefined.");
        }
        return (-normal_(0) * x - d_) / normal_(1);
    }
};

class PointToLineFactor : public ceres::SizedCostFunction<1, 3> {
public:
    PointToLineFactor(const Eigen::Vector2d& local_pt, const Line& line) : local_pt_(local_pt), line_(line) {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        // 这里的参数应该是相机位姿 --> 世界位姿的转换
        const double x = parameters[0][0];
        const double y = parameters[0][1];
        const double theta = parameters[0][2];

        // R_cw 表示的是世界坐标系到本地坐标系的旋转矩阵。 其中 theta 表示逆时针旋转的角度
        // R_cw = [cos -sin; sin cos]
        const double cos_theta = cos(theta);
        const double sin_theta = sin(theta);

        // 将相机点转化成世界点 P_w = R_wc P_c + t
        Eigen::Vector2d p_w;
        p_w(0) = cos_theta * local_pt_(0) - sin_theta * local_pt_(1) + x;
        p_w(1) = sin_theta * local_pt_(0) + cos_theta * local_pt_(1) + y;

        cout << "转换后的世界坐标：[" << p_w(0) << ", " << p_w(1) << "]" << endl;

        // 计算残差
        // *residuals = line_.distance(p_w);
        // 这里使用有符号距离计算残差，不然在近零点处导数会不连续
        double signed_dist = line_.normal_.dot(p_w) + line_.d_;
        *residuals = signed_dist;

        // 雅可比矩阵计算
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            // 我们需要计算 dr/dx, dr/dy, dr/dtheta, 即残差对位姿的导数
            // r = n^T * p_w + d
            // dr/dx = n_x
            jacobians[0][0] = line_.normal_(0); // dr/dx
            // dr/dy = n_y
            jacobians[0][1] = line_.normal_(1); // dr/dy
            // dr/dtheta = n^T * d(p_w)/dtheta
            // p_w = R_wc * p_c + t
            // dp_w/dtheta = d(R_wc)/dtheta * p_c = [-sinθ -cosθ; cosθ -sinθ] * p_c
            // 所以 dr/dtheta = n^T * dp_w/dtheta
            Eigen::Vector2d dR_dtheta_p;
            dR_dtheta_p(0) = -sin_theta * local_pt_(0) - cos_theta * local_pt_(1);
            dR_dtheta_p(1) = cos_theta * local_pt_(0) - sin_theta * local_pt_(1);
            
            jacobians[0][2] = line_.normal_.dot(dR_dtheta_p); // dr/dtheta
        }

        return true;
    }
//
private:
    Eigen::Vector2d local_pt_;
    Line line_;
};

vector<Eigen::Vector2d> generateNoisyPoints(const Pose2D& true_pose, const Line& line, int num_points = 10) {
    vector<Eigen::Vector2d> points;
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<double> dist(0.0, 0.01); // 高斯噪声，标准差0.1

    double cos_theta = cos(true_pose.theta);
    double sin_theta = sin(true_pose.theta);

    for (int i = 0; i < num_points; ++i) {
        double y_intercept = line.yIntercept(i);
        Eigen::Vector2d world_pt(i, y_intercept);
        // 添加噪声
        world_pt(0) += dist(gen);
        world_pt(1) += dist(gen);

        // 转换到局部坐标系
        Eigen::Vector2d local_pt;
        // 世界坐标系旋转theta == 局部点逆时针旋转 -theta
        local_pt(0) = cos_theta * (world_pt(0) - true_pose.x) + sin_theta * (world_pt(1) - true_pose.y);
        local_pt(1) = -sin_theta * (world_pt(0) - true_pose.x) + cos_theta * (world_pt(1) - true_pose.y);

        points.push_back(local_pt);
    }
    return points;
}

int main() {
    Line line(Eigen::Vector2d(2.0, 3.0), -15.0);

    // 真是位姿 (3, 1, -45度)
    Pose2D true_pose(3.0, 1.0, -M_PI / 4);

    auto oberved_points = generateNoisyPoints(true_pose, line, 10);

    // 创建优化问题
    ceres::Problem problem;
    // 初始估计值（故意给一个错误的初始值）
    double pose[3] = {2.5, 1.0, -M_PI / 6}; // 与真实值有偏差

    // 测试
    {
        double y = line.yIntercept(0);
        Eigen::Vector2d world_pt(0, y);
        Eigen::Vector2d local_pt;
        double cos_theta = cos(true_pose.theta);
        double sin_theta = sin(true_pose.theta);
        local_pt(0) = cos_theta * (world_pt(0) - true_pose.x) + sin_theta * (world_pt(1) - true_pose.y);
        local_pt(1) = -sin_theta * (world_pt(0) - true_pose.x) + cos_theta * (world_pt(1) - true_pose.y);
        cout << "世界坐标：[" << world_pt(0) << ", " << world_pt(1) << "], 相机坐标：[" << local_pt(0) << ", " << local_pt(1) << "]" << endl;
        PointToLineFactor factor(local_pt, line);
        double a[3] = {3.0, 1.0, -M_PI / 4};
        const double* para[1] = {a};
        double res = 0;
        double* jacobians[1] = {nullptr};
        double jacobians_value[3] = {0};
        jacobians[0] = jacobians_value;
        factor.Evaluate(para, &res, jacobians);
        cout << "残差值" << res << endl;
        cout << "参数值:[" << para[0][0] << ", " << para[0][1] << ", " << para[0][2] << "], jacobians : [" << jacobians[0][0] << ", " << jacobians[0][1] << ", " << jacobians[0][2] << "]" << endl;

        // 世界坐标：[0, 5], 相机坐标：[-4.94975, 0.707107]
        // 转换后的世界坐标：[0, 5]
        // 残差值0
        // 参数值:[3, 1, -0.785398], jacobians : [0.5547, 0.83205, -4.71495]
        // 
        // 这里可以看到，当角度对的时候，dr/dx 和 dr/dy 永远是大于 0 的，那么x, y 会倾向于向减少的方向移动，这就会导致位置不准
    }

    // 为每个观测点创建残差块
    for (const auto& local_pt : oberved_points) {
        // 创建代价函数
        ceres::CostFunction* cost_function = 
            new PointToLineFactor(local_pt, line);
        
        // 添加到问题中
        problem.AddResidualBlock(cost_function, nullptr, pose);
    }

    cout << "初始位姿估计: x=" << pose[0] << ", y=" << pose[1] 
         << ", theta=" << pose[2] * 180/M_PI << "度" << endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;    // 是否在控制台输出优化过程的详细信息
    options.max_num_iterations = 1000;              // 最大迭代次数，防止无限循环
    options.function_tolerance = 1e-8;              // 成本函数变化的收敛阈值

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    cout << summary.FullReport() << endl;
    cout << "优化后位姿估计: x=" << pose[0] << ", y=" << pose[1] 
         << ", theta=" << pose[2] * 180/M_PI << "度" << endl;

    return 0;
}
