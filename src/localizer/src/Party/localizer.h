#pragma once
#include <vector>
#include <functional>
#include <string>
#include <mutex>

namespace localizer
{
    struct Vec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Point
    {
        float x, y, z;
        float intensity;
        float curvature;
        float normal_x, normal_y, normal_z;
    };

    struct PointCloud
    {
        std::vector<Point> points;
        double timestamp;
    };

    struct IMUData
    {
        Vec3 acc;    // 加速度
        Vec3 gyro;   // 角速度
        double time; // 时间戳
        IMUData() = default;
        IMUData(const Vec3 &a, const Vec3 &g, double &t) : acc(a), gyro(g), time(t) {}
    };

    struct Pose
    {
        float x, y, z;          // 初始位置
        float yaw, pitch, roll; // 初始姿态
    };

    struct PoseStamped
    {
        // rotation: 3x3 行优先
        double R[9];
        // translation
        Vec3 t;
        double timestamp; // 时间戳
    };

    using PoseCallback = std::function<void(const PoseStamped &)>;
    using LogCallback = std::function<void(const std::string &)>;

    class Localizer
    {
    public:
        explicit Localizer(const std::string &config_path);

        /** 输入一帧 IMU 数据（生产者） */
        void feedIMU(const IMUData &imu);

        /** 输入一帧激光点云（生产者） */
        void feedLidar(const PointCloud &cloud);

        /** 强制设置位姿（重定位、回环等用） */
        void setPose(const Pose &pose);

        /** 设置异步回调 */
        void setPoseCallback(PoseCallback cb);

        void setLogCallback(LogCallback cb);

        /** 启动内部工作线程（可选）*/
        void startAsync();

        /** 停止线程（可选）*/
        void stopAsync();

    private:
    };

} // namespace localizer
