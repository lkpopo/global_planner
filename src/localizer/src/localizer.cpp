#include "localizer.h"
#include <thread>

namespace localizer{
Localizer::Localizer(const std::string &config_path)
{
    // TODO: 加载配置
}

void Localizer::feedIMU(const IMUData &imu)
{
    // TODO: 存入 IMU 队列
}

void Localizer::feedLidar(const PointCloud &cloud)
{
    // TODO: 存入点云队列
}

void Localizer::setPose(const Pose &pose)
{
    // TODO: 强制设置位姿
}

void Localizer::setPoseCallback(PoseCallback cb)
{

}

void Localizer::startAsync()
{

}

void Localizer::stopAsync()
{

}
}