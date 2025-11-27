#pragma once
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <chrono>
#include <mutex>
#include <deque>


inline uint64_t now_ns()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch()
           ).count();
}


struct ICPConfig
{
    double refine_scan_resolution = 0.1;
    double refine_map_resolution = 0.1;
    double refine_score_thresh = 0.1;
    int refine_max_iteration = 10;

    double rough_scan_resolution = 0.25;
    double rough_map_resolution = 0.25;
    double rough_score_thresh = 0.2;
    int rough_max_iteration = 5;
};

class IESKF;
class IMUProcessor;
class LidarProcessor;
class ICPLocalizer;

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

using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

template <typename T>
using Vec = std::vector<T>;



struct PointCloud
{
    CloudType::Ptr points;
    double timestamp;
};

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;         // 加速度
    V3D gyro;        // 角速度
    double time; // 时间戳
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose_loc
{
    float x, y, z;          // 初始位置
    float yaw, pitch, roll; // 初始姿态
};

struct PoseStamped
{
    // 四元数
    Eigen::Quaterniond q;
    // translation
    V3D t;
    double timestamp; // 时间戳
};

struct Config
{
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

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_imu_received = false;
    bool message_lidar_received = false;
    bool service_received = false;
    uint64_t last_send_time = now_ns();
    double last_message_time;
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

enum BuilderStatus
{
    IMU_INIT,
    MAP_INIT,
    MAPPING
};

struct SyncPackage
{
    Vec<IMUData> imus;
    CloudType::Ptr cloud;
    double cloud_start_time = 0.0;
    double cloud_end_time = 0.0;
};


