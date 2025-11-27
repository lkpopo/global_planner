#pragma once
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace global_planner
{
    struct Pose
    {
        Eigen::Vector3d position;
        Pose(Eigen::Vector3d _pos) : position(_pos) {}
        Pose(float _x, float _y, float _z)
        {
            position.x() = _x;
            position.y() = _y;
            position.z() = _z;
        }
    };

    struct Path
    {
        std::vector<Pose> poses;
    };

    struct GpsPoint
    {
        double lat;
        double lon;
        double alt;
    };
}
