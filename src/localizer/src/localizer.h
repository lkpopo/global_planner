#pragma once
#include <vector>
#include <functional>
#include <string>
#include <mutex>
#include <queue>
#include <thread>
#include <atomic>


#include "localizer_utils.h"


namespace localizer
{
    using PoseCallback = std::function<void(const PoseStamped &)>;
    using LogCallback = std::function<void(const std::string &)>;

    class Localizer
    {
    public:
        explicit Localizer(const std::string &config_path);
        ~Localizer();

        /** 输入一帧 IMU 数据（生产者） */
        void feedIMU(const IMUData &imu);

        /** 输入一帧激光点云（生产者） */
        void feedLidar(const PointCloud &cloud);

        /** 强制设置位姿（重定位、回环等用） */
        void setPose(const Pose_loc &pose);
        bool setPointCloud(const std::string &pcd_path);

        /** 设置异步回调 */
        void setPoseCallback(PoseCallback cb);
        void setLogCallback(LogCallback cb);

        /** 启动内部工作线程（可选）*/
        void startAsync();

        /** 停止线程（可选）*/
        void stopAsync();
    
    private:
        void log(const std::string &msg);
        bool syncPackage();
        void localize();

    private:
        Config m_config;
        NodeState m_state;

        ICPConfig m_localizer_config;
        StateData m_state_data;
        SyncPackage m_package;

        BuilderStatus m_builder_status;

        std::string pcd_path;
        std::shared_ptr<IESKF> kf;
        std::shared_ptr<IMUProcessor> imu_processor;
        std::shared_ptr<LidarProcessor> lidar_processor;
        std::shared_ptr<ICPLocalizer> m_localizer;

        PoseCallback pose_callback_;
        LogCallback log_callback_;

        std::thread worker_thread_;
        std::atomic<bool> running_;
    };

} // namespace localizer
