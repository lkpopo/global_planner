#include "localizer.h"

Localizer::Localizer(const std::string &config_path)
{

}

void Localizer::feedIMU(const IMUData &imu)
{

}
void Localizer::feedLidar(const CloudType::Ptr &cloud, double timestamp)
{

}
bool Localizer::update()
{
    return false;
}
bool Localizer::getLatestPose(PoseStamped &pose) const
{
    return false;
}
void Localizer::setPose(const Pose &pose)
{

}