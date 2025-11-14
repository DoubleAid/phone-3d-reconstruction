#include <ceres/ceres.h>
#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

class Line {
private:
    double d_;
    Eigen::Vector2d normal_;

public:
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
};

class PointToLineFactor : public ceres::SizedCostFunction<1, 3> {
public:
    PointToLineFactor(const Eigen::Vector2d& local_pt, const Line& line) : local_pt_(local_pt), line_(line) {}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        const double x = parameters[0][0];
        const double y = parameters[0][1];
        const double theta = parameters[0][2];

        // R_cw 表示的是世界坐标系到本地坐标系的旋转矩阵。 其中 theta 表示逆时针旋转的角度
        // R_cw = [cos -sin; sin cos]
        const double cos_theta = cos(theta);
        const double sin_theta = sin(theta);

        // 将局部点转化成本地点 P_local = R_cw P_w + t ==> P_w = R_wc(P_local - t)
        Eigen::Vector2d p_w;
        p_w(0) = cos_theta * (local_pt_(0) - x) + sin_theta * (local_pt_(1) - y);
        p_w(1) = -sin_theta * (local_pt_(0) - x) + cos_theta * (local_pt_(1) - y);
        
        // 计算残差
        *residuals = line_.distance(p_w);

        // 雅可比矩阵计算
        if (jacobians != nullptr && jacobians[0] != nullptr) {
            // 公式 res = nP_w + d = n (R_wc * P_l - R_wc * t) + d = normal[0]
            // dr/dx = -nR_wc   dr/dy = -nR_wc
        }
    }

private:
    Eigen::Vector2d local_pt_;
    Line line_;
};

// // 自定义残差仿函数：2D点到直线的距离
// class PointToLineFactor : public ceres::SizedCostFunction<1 /* 残差维度 */, 3 /* 参数块维度：x, y, theta */> {
//  public:
//   // 构造函数：传入局部点坐标和直线参数
//   PointToLineFactor(const Eigen::Vector2d& local_pt,
//                     const Eigen::Vector2d& line_normal, double line_d)
//       : local_pt_(local_pt), line_normal_(line_normal), line_d_(line_d) {}

//   // 核心函数：计算残差和（可选的）雅可比矩阵
//   virtual bool Evaluate(double const* const* parameters,
//                         double* residuals,
//                         double** jacobians) const {
//     // 1. 解包参数
//     const double x = parameters[0][0];
//     const double y = parameters[0][1];
//     const double theta = parameters[0][2];

//     // 2. 计算旋转矩阵 R = [cosθ -sinθ; sinθ cosθ]
//     const double cos_theta = cos(theta);
//     const double sin_theta = sin(theta);

//     // 3. 将局部点转换到世界坐标系
//     // p_w = R * p_l + t， 这点有点问题，应该求的是 R 的转置
//     Eigen::Vector2d p_w;
//     p_w(0) = cos_theta * local_pt_(0) - sin_theta * local_pt_(1) + x;
//     p_w(1) = sin_theta * local_pt_(0) + cos_theta * local_pt_(1) + y;

//     // 4. 计算残差：r = n · p_w + d
//     residuals[0] = line_normal_.dot(p_w) + line_d_;

//     // 5. 如果请求，计算雅可比矩阵（关于位姿 [x, y, theta]）
//     if (jacobians != nullptr && jacobians[0] != nullptr) {
//       // 获取雅可比矩阵指针
//       double* jacobian = jacobians[0];
      
//       // 残差对平移 t 的导数：dr/dx = n_x, dr/dy = n_y
//       jacobian[0] = line_normal_(0); // dr/dx
//       jacobian[1] = line_normal_(1); // dr/dy

//       // 残差对旋转角 theta 的导数：dr/dθ = n^T · (dR/dθ · p_l)
//       // dR/dθ = [-sinθ -cosθ; cosθ -sinθ]
//       Eigen::Vector2d dR_dtheta_p;
//       dR_dtheta_p(0) = -sin_theta * local_pt_(0) - cos_theta * local_pt_(1);
//       dR_dtheta_p(1) = cos_theta * local_pt_(0) - sin_theta * local_pt_(1);
//       jacobian[2] = line_normal_.dot(dR_dtheta_p); // dr/dθ
//     }

//     return true;
//   }

//  private:
//   const Eigen::Vector2d local_pt_;   // 局部坐标系下的点
//   const Eigen::Vector2d line_normal_; // 直线的单位法向量
//   const double line_d_;               // 直线参数 d
// };

// // 生成带噪声的观测点
// vector<Eigen::Vector2d> generateNoisyPoints(const Eigen::Vector2d& true_pose, 
//                                            double true_theta, 
//                                            int num_points = 10) {
//     vector<Eigen::Vector2d> points;
//     random_device rd;
//     mt19937 gen(rd());
//     normal_distribution<double> dist(0.0, 0.1); // 高斯噪声，标准差0.1
    
//     // 真实位姿的旋转矩阵
//     double cos_theta = cos(true_theta);
//     double sin_theta = sin(true_theta);
    
//     // 在局部坐标系下生成一些点（比如沿着x轴分布）
//     for (int i = 0; i < num_points; ++i) {
//         // 局部坐标系下的点（在机器人前方）
//         Eigen::Vector2d local_pt(i * 0.3, 0.0);
        
//         // 转换到世界坐标系（真实变换）
//         Eigen::Vector2d world_pt;
//         world_pt(0) = cos_theta * local_pt(0) - sin_theta * local_pt(1) + true_pose(0);
//         world_pt(1) = sin_theta * local_pt(0) + cos_theta * local_pt(1) + true_pose(1);
        
//         // 添加噪声
//         world_pt(0) += dist(gen);
//         world_pt(1) += dist(gen);
        
//         points.push_back(world_pt);
//     }
    
//     return points;
// }

// int main() {
//     // 直线方程: 2x - 3y + 8 = 0
//     // 法向量 n = (2, -3)，需要单位化
//     Eigen::Vector2d line_normal(2, -3);
//     line_normal.normalize(); // 单位化法向量
//     double line_d = 8.0 / sqrt(2 * 2 + (-3)*(-3)); // d需要相应缩放
    
//     cout << "直线方程: 2x - 3y + 8 = 0" << endl;
//     cout << "单位法向量: (" << line_normal(0) << ", " << line_normal(1) << ")" << endl;
//     cout << "缩放后的d: " << line_d << endl;
    
//     // 真实位姿：机器人位于(1, 2)，朝向30度
//     Eigen::Vector2d true_pose(1.0, 2.0);
//     double true_theta = M_PI / 6; // 30度
    
//     cout << "\n真实位姿: x=" << true_pose(0) << ", y=" << true_pose(1) 
//          << ", theta=" << true_theta * 180/M_PI << "度" << endl;
    
//     // 生成带噪声的观测点
//     auto observed_points = generateNoisyPoints(true_pose, true_theta, 12);
//     cout << "生成了 " << observed_points.size() << " 个带噪声的观测点" << endl;
    
//     // 创建优化问题
//     ceres::Problem problem;
    
//     // 初始估计值（故意给一个错误的初始值） - 这里正确定义pose变量
//     double pose[3] = {0.5, 1.0, 0.1}; // 与真实值有偏差
    
//     // 为每个观测点创建残差块
//     vector<Eigen::Vector2d> local_points;
//     for (int i = 0; i < observed_points.size(); ++i) {
//         // 在局部坐标系下，这些点都在机器人前方（我们假设观测到了这些点）
//         Eigen::Vector2d local_pt(i * 0.3, 0.0);
//         local_points.push_back(local_pt);
        
//         // 创建代价函数
//         ceres::CostFunction* cost_function = 
//             new PointToLineFactor(local_pt, line_normal, line_d);
        
//         // 添加到问题中
//         problem.AddResidualBlock(cost_function, nullptr, pose);
//     }
    
//     cout << "\n优化前位姿估计: x=" << pose[0] << ", y=" << pose[1] 
//          << ", theta=" << pose[2] * 180/M_PI << "度" << endl;
    
//     // 计算初始残差
//     double initial_cost = 0;
//     for (int i = 0; i < observed_points.size(); ++i) {
//         double theta = pose[2];
//         double cos_theta = cos(theta);
//         double sin_theta = sin(theta);
        
//         Eigen::Vector2d p_w;
//         p_w(0) = cos_theta * local_points[i](0) - sin_theta * local_points[i](1) + pose[0];
//         p_w(1) = sin_theta * local_points[i](0) + cos_theta * local_points[i](1) + pose[1];
        
//         double residual = line_normal.dot(p_w) + line_d;
//         initial_cost += residual * residual;
//     }
//     cout << "初始代价: " << initial_cost << endl;
    
//     // 配置求解器选项
//     ceres::Solver::Options options;
//     options.linear_solver_type = ceres::DENSE_QR;
//     options.minimizer_progress_to_stdout = true;
//     options.max_num_iterations = 100;
//     options.function_tolerance = 1e-8;
    
//     // 运行优化
//     ceres::Solver::Summary summary;
//     cout << "\n开始优化..." << endl;
//     ceres::Solve(options, &problem, &summary);
    
//     // 输出结果
//     cout << summary.FullReport() << endl;
//     cout << "优化后位姿估计: x=" << pose[0] << ", y=" << pose[1] 
//          << ", theta=" << pose[2] * 180/M_PI << "度" << endl;
    
//     // 验证结果：计算最终残差
//     double final_cost = 0;
//     for (int i = 0; i < observed_points.size(); ++i) {
//         double theta = pose[2];
//         double cos_theta = cos(theta);
//         double sin_theta = sin(theta);
        
//         Eigen::Vector2d p_w;
//         p_w(0) = cos_theta * local_points[i](0) - sin_theta * local_points[i](1) + pose[0];
//         p_w(1) = sin_theta * local_points[i](0) + cos_theta * local_points[i](1) + pose[1];
        
//         double residual = line_normal.dot(p_w) + line_d;
//         final_cost += residual * residual;
//     }
//     cout << "最终代价: " << final_cost << endl;
    
//     // 输出误差
//     cout << "\n位姿估计误差:" << endl;
//     cout << "x误差: " << pose[0] - true_pose(0) << endl;
//     cout << "y误差: " << pose[1] - true_pose(1) << endl;
//     cout << "角度误差: " << (pose[2] - true_theta) * 180/M_PI << "度" << endl;
    
//     // 显示一些观测点信息
//     cout << "\n前5个观测点的世界坐标:" << endl;
//     for (int i = 0; i < min(5, (int)observed_points.size()); ++i) {
//         cout << "点" << i+1 << ": (" << observed_points[i](0) << ", " << observed_points[i](1) << ")" << endl;
        
//         // 计算这些点到直线的真实距离
//         double true_distance = (2 * observed_points[i](0) - 3 * observed_points[i](1) + 8) / sqrt(13);
//         cout << "  到直线的距离: " << true_distance << endl;
//     }
    
//     return 0;
// }