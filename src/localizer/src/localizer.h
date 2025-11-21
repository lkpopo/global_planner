#pragma once
#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using V3D = Eigen::Vector3d;
using PointType = pcl::PointXYZINormal;
using CloudType = pcl::PointCloud<PointType>;
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

struct IMUData
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;     // 线速度
    V3D gyro;    // 角速度
    double time; // 时间戳
    IMUData() = default;
    IMUData(const V3D &a, const V3D &g, double &t) : acc(a), gyro(g), time(t) {}
};

struct Pose
{
    float x, y, z;          // 初始位置
    float yaw, pitch, roll; // 初始姿态
};

struct PoseStamped
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double timestamp; // 时间戳
};

class Localizer
{
public:
    /** 构造函数：传入 YAML 或结构体配置即可 */
    explicit Localizer(const std::string &config_path);

    /** 输入一帧 IMU 数据 */
    void feedIMU(const IMUData &imu);

    /** 输入一帧激光点云（已经转换成 PCL 或你的自定义点云格式） */
    void feedLidar(const CloudType::Ptr &cloud, double timestamp);

    /**
     * 处理一次数据（IMU + Lidar），返回是否成功输出位姿
     * 外部定时调用 update()
     */
    bool update();

    /** 获取当前位姿 (世界坐标系 → 机体坐标系) */
    bool getLatestPose(PoseStamped &pose) const;

    //设置位姿
    void setPose(const Pose &pose);


private:
    /** 一些内部状态 */
    // struct State
    // {
    //     std::mutex state_mutex;

    //     Eigen::Matrix3d last_r = Eigen::Matrix3d::Identity();
    //     Eigen::Vector3d last_t = Eigen::Vector3d::Zero();

    //     // offset = map - local
    //     Eigen::Matrix3d offset_r = Eigen::Matrix3d::Identity();
    //     Eigen::Vector3d offset_t = Eigen::Vector3d::Zero();

    //     bool lidar_inited = false;
    //     bool imu_inited = false;

    //     double last_imu_time = -1;
    //     double last_lidar_time = -1;
    // };
    // struct NodeState
    // {
    //     std::mutex message_mutex;
    //     std::mutex service_mutex;

    //     bool message_imu_received = false;
    //     bool message_lidar_received = false;
    //     bool service_received = false;
    //     rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    //     builtin_interfaces::msg::Time last_message_time;
    //     M3D last_r;                          // localmap_body_r
    //     V3D last_t;                          // localmap_body_t
    //     M3D last_offset_r = M3D::Identity(); // map_localmap_r
    //     V3D last_offset_t = V3D::Zero();     // map_localmap_t
    //     M4F initial_guess = M4F::Identity();
    // };

private:
    // bool isPose_set();
    // std::shared_ptr<IESKF> kf_;
    // std::shared_ptr<IMUProcessor> imu_processor_;
    // std::shared_ptr<LidarProcessor> lidar_processor_;
    // std::shared_ptr<ICPLocalizer> icp_;

    // mutable State state_;

    // // 缓存数据
    // std::deque<IMUData> imu_buffer_;
    // std::deque<std::pair<double, CloudType::Ptr>> lidar_buffer_;

    // bool syncPackage();
    // bool processIMU();
    // bool processLidar();
    // bool doLocalization();

    // SyncPackage current_pkg_;
};
