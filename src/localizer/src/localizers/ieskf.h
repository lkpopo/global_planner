#pragma once
#include <Eigen/Eigen>
#include <sophus/so3.hpp>

using M3D = Eigen::Matrix3d;
using V3D = Eigen::Vector3d;
using M3F = Eigen::Matrix3f;
using V3F = Eigen::Vector3f;
using M2D = Eigen::Matrix2d;
using V2D = Eigen::Vector2d;
using M2F = Eigen::Matrix2f;
using V2F = Eigen::Vector2f;
using M4D = Eigen::Matrix4d;
using V4D = Eigen::Vector4d;
using M4F = Eigen::Matrix4f;
using V4F = Eigen::Vector4f;

using M12D = Eigen::Matrix<double, 12, 12>;
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;
using M21X12D = Eigen::Matrix<double, 21, 12>;

M3D Jr(const V3D &inp);
M3D JrInv(const V3D &inp);

struct SharedState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H;
    V12D b;
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
};
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    Input() = default;
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}
    Input(double a1, double a2, double a3, double g1, double g2, double g3) : acc(a1, a2, a3), gyro(g1, g2, g3) {}
};
struct State
{
    static double gravity;  // 重力加速度大小（标量），例如 9.81

    // 世界坐标系到IMU坐标系的旋转矩阵（姿态）
    // 表示IMU在世界系中的朝向
    M3D r_wi = M3D::Identity();

    // IMU在世界坐标系下的位置（平移向量）
    V3D t_wi = V3D::Zero();

    // IMU坐标系到Lidar坐标系的旋转外参
    // 用于将Lidar点云转换到IMU系
    M3D r_il = M3D::Identity();

    // IMU坐标系到Lidar坐标系的平移外参
    V3D t_il = V3D::Zero();

    // IMU在世界坐标系下的线速度
    V3D v = V3D::Zero();

    // 陀螺仪零偏（Bias of Gyroscope）
    V3D bg = V3D::Zero();

    // 加速度计零偏（Bias of Accelerometer）
    V3D ba = V3D::Zero();

    // 世界坐标系下的重力方向与大小（默认z轴负方向）
    V3D g = V3D(0.0, 0.0, -9.81);

    // 根据输入的重力方向向量初始化重力值
    // gravity_dir只决定方向，大小由静态成员gravity控制
    void initGravityDir(const V3D &gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

    // 状态增量更新（用于误差状态卡尔曼滤波器中应用更新量δx）
    void operator+=(const V21D &delta);

    // 计算两个状态之间的差（用于误差计算）
    V21D operator-(const State &other) const;

    // 友元函数，用于打印State内容，方便调试
    friend std::ostream &operator<<(std::ostream &os, const State &state);
};

using loss_func = std::function<void(State &, SharedState &)>;
using stop_func = std::function<bool(const V21D &)>;

class IESKF
{
public:
    IESKF() = default;

    void setMaxIter(size_t iter) { m_max_iter = iter; }   // 设置最大迭代次数
    void setLossFunction(loss_func func) { m_loss_func = func; } // 设置代价函数（如Huber、Cauchy等）
    void setStopFunction(stop_func func) { m_stop_func = func; } // 设置停止条件（比如收敛判断）

    void predict(const Input &inp, double dt, const M12D &Q);  // 预测步骤
    void update();                                             // 更新步骤

    State &x() { return m_x; }  // 当前状态
    M21D &P() { return m_P; }   // 当前协方差矩阵

private:
    size_t m_max_iter = 10;     // 最大迭代次数
    State m_x;                  // 当前系统状态（位置、速度、姿态等）
    M21D m_P;                   // 状态协方差矩阵
    loss_func m_loss_func;      // 鲁棒代价函数
    stop_func m_stop_func;      // 收敛判定函数
    M21D m_F;                   // 状态转移矩阵
    M21X12D m_G;                // 噪声传播矩阵
};
