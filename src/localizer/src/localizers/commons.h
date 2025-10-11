#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <mutex>
#include <queue>
#include "ieskf.h"

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

template <typename T>
using Vec = std::vector<T>;

bool esti_plane(PointVec &points, const double &thresh, V4D &out);

float sq_dist(const PointType &p1, const PointType &p2);

struct Config
{
    std::string cloud_topic;
    std::string odom_topic;
    std::string map_frame;
    std::string local_frame;
    double update_hz;

    int lidar_filter_num;
    double lidar_min_range;
    double lidar_max_range;
    double scan_resolution;
    double map_resolution;

    double cube_len;
    double det_range;
    double move_thresh;

    double na;
    double ng;
    double nba;
    double nbg;
    int imu_init_num;
    int near_search_num;
    int ieskf_max_iter;
    bool gravity_align;
    bool esti_il;
    M3D r_il;
    V3D t_il;

    double lidar_cov_inv;
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    double time;
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
};

struct Pose
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double offset;
    V3D acc;
    V3D gyro;
    V3D vel;
    V3D trans;
    M3D rot;
    Pose() = default;
    Pose(double t, const V3D &a, const V3D &g, const V3D &v, const V3D &p, const M3D &r) : offset(t), acc(a), gyro(g), vel(v), trans(p), rot(r) {}
};

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};