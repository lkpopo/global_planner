#include "planner.h"
#include "A_star.h"

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <memory>

namespace global_planner
{
    struct Impl
    {
        Eigen::Vector3d loc2utm_T;
        Eigen::Quaterniond loc2utm_Q;
    };
    // 时间戳允许误差（秒），用于对齐时间戳
    static constexpr double TIME_SYNC_THRESHOLD = 0.05; // 50ms

    // 航点到达判定距离阈值
    static constexpr double WAYPOINT_REACHED_DIST = 0.5; // 0.5米

    // 用于坐标系转换的常量
    static constexpr double WGS84_A = 6378137.0;                // 长半轴
    static constexpr double WGS84_F = 1.0 / 298.257223563;      // 扁率
    static constexpr double WGS84_E2 = WGS84_F * (2 - WGS84_F); // 偏心率平方
    static constexpr double UTM_K0 = 0.9996;
    std::unique_ptr<Impl> impl_ = std::make_unique<Impl>();

    Eigen::Vector3d gpsToUtm(double lat, double lon, double alt, int &utm_zone_);
    Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw);

    planner::planner()
    {
        Astar_ptr = std::make_shared<Astar>();
        plannedWaypointsCallback_ = nullptr;
        realTimeUTMCallback_ = nullptr;
        // waypointReachedCallback_ = nullptr;
        taskStatusCallback_ = nullptr;
        task_status_ = IDLE;

        plan_thread_running_ = false;
    }
    planner::~planner()
    {
        stop();
    }

    bool planner::setMap(const std::string &map_path)
    {
        {
            if (map_loading_)
            {
                log("[setMap] Map is currently loading. Ignore this call.\n");
                return false;
            }
            map_loading_ = true;
        }

        // TODO: Implement this method
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(map_path, *cloud_) != 0)
        {
            log("Failed to load PCD: " + map_path + "\n");
            return false;
        }
        else
        {
            log("[GlobalPlannerUGV] PCD loaded, point size: " + std::to_string(cloud_->points.size()) + "\n");
            Astar_ptr->Occupy_map_ptr->map_update_gpcl(cloud_);
            log("[GlobalPlannerUGV] Occupancy map updated from point cloud.\n");
            {
                std::lock_guard<std::mutex> lock(map_mutex_);
                map_ready_ = true;
                map_loading_ = false;
            }

            map_cv_.notify_all(); //  唤醒等待地图加载的线程
            return true;
        }
    }

    bool planner::setConfig(const std::string &config_path)
    {
        // TODO: Implement this method
        log("[GlobalPlannerUGV] Astart init " + config_path + "\n");
        bool res = Astar_ptr->init(config_path);

        return res;
    }

    bool planner::setOffset(uavImu imu)
    {
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            imu_offset_ = imu;
            // if(task_status_==IDLE)
            // log("[planner] Set IMU offset: x=" + std::to_string(imu.x) +
            //     " y=" + std::to_string(imu.y) + " z=" + std::to_string(imu.z));
        }

        // 试图进行坐标转换计算
        if (task_status_ == IDLE)
            tryUpdateLocalToUTMTransform();
        return true;
    }

    bool planner::setUavAttitude(uavAttitude attitude)
    {
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            attitude_ = attitude;
            // if(task_status_==IDLE)
            // log("[planner] Set UAV attitude: yaw=" + std::to_string(attitude.yaw));
        }

        if (task_status_ == IDLE)
            tryUpdateLocalToUTMTransform();
        return true;
    }

    bool planner::setCurrLocation(Location location)
    {
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            curr_location_ = location;
            // if(task_status_==IDLE)
            // log("[planner] Set GPS: lat=" + std::to_string(location.la) +
            //     " lon=" + std::to_string(location.lo));
        }

        if (task_status_ == IDLE)
            tryUpdateLocalToUTMTransform();
        return true;
    }

    bool planner::planWaypointsPath()
    {
        full_path_.clear();

        Eigen::Vector3d current_start;
        auto curr_p = utm_waypoints_.front().location; // 起点
        int index = 0;
        // full_path_.push_back(curr_p);
        current_start << curr_p.x, curr_p.y, curr_p.z;

        for (size_t i = 1; i < utm_waypoints_.size(); ++i)
        {
            Eigen::Vector3d current_goal;
            auto goal_p = utm_waypoints_[i].location;
            current_goal << goal_p.x, goal_p.y, goal_p.z;

            if (!Astar_ptr->search(current_start, current_goal))
            {
                log("[planner] Failed to plan path segment to waypoint " + std::to_string(i));
                return false;
            }

            auto segment = Astar_ptr->getPath(); // 或 retrievePath() 返回最终路径
            segment.begin()->index = index++;
            (segment.end() - 1)->index = index;

            if (!full_path_.empty() && !segment.empty())
                segment.erase(segment.begin()); // 去掉重复起点
            full_path_.insert(full_path_.end(), segment.begin(), segment.end());

            current_start = current_goal;
        }
        if (plannedWaypointsCallback_)
            plannedWaypointsCallback_(full_path_);
        log("[planner] Full path planned successfully. Total nodes: " + std::to_string(full_path_.size()));

        return true;
    }

    void planner::startRealtimeThread()
    {
        stop_thread_ = false;

        realtime_nav_thread_ = std::thread([this]()
                                           {
        size_t reached_index = 0;
        while (!stop_thread_)
        {
            Eigen::Vector3d current_utm;

            {
                std::lock_guard<std::mutex> lk(data_mutex_);

                // 将IMU局部坐标转换为UTM
                Eigen::Vector3d local_xyz(imu_offset_->x, imu_offset_->y, imu_offset_->z);
                current_utm = impl_->loc2utm_Q * local_xyz + impl_->loc2utm_T;
            }

            // 回调实时UTM位置
            if (realTimeUTMCallback_)
            {
                UTM_Location utm;
                utm.x = current_utm(0);
                utm.y = current_utm(1);
                utm.z = current_utm(2);
                realTimeUTMCallback_(utm);
            }
            /*
            // 检查当前航点是否到达
            if (reached_index < utm_waypoints_.size())
            {
                const auto &wp = utm_waypoints_[reached_index].location;
                double dx = wp.x - current_utm(0);
                double dy = wp.y - current_utm(1);
                double dz = wp.z - current_utm(2);
                double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

                if (dist <= WAYPOINT_REACHED_DIST)
                {
                    // 回调航点到达
                    if (waypointReachedCallback_)
                    {
                        reachedPoint rp;
                        rp.index = static_cast<int>(reached_index);
                        rp.total = static_cast<int>(utm_waypoints_.size());
                        rp.wp.location.x = wp.x;
                        rp.wp.location.y = wp.y;
                        rp.wp.location.z = wp.z;
                        rp.wp.attitude = utm_waypoints_[reached_index].attitude;
                        rp.wp.gimbal = utm_waypoints_[reached_index].gimbal;

                        waypointReachedCallback_(rp);
                    }

                    reached_index++;
                }
            }
            */
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 控制循环频率
        }

        log("[planner] Realtime thread stopped."); });
    }

    void planner::startPlanThread()
    {
        if (plan_thread_running_)
        {
            log("[planner] Plan thread already running.");
            return;
        }

        plan_thread_running_ = true;

        plan_thread_ = std::thread([this]()
                                   {
        log("[planner] Waiting for map to be ready.");
        {
            std::unique_lock<std::mutex> lock(map_mutex_);
            map_cv_.wait(lock, [this] { return map_ready_; });
        }
        log("[planner] Planning thread started.");

        bool success = planWaypointsPath();

        if (!success)
        {
            log("[planner] Planning failed.");

            plan_thread_running_ = false;

            // 状态回调
            task_status_ = FAILED;
            if (taskStatusCallback_) taskStatusCallback_(task_status_);

            return;
        }

        // 规划成功
        task_status_ = COMPLETED;
        if (taskStatusCallback_) taskStatusCallback_(task_status_);

        log("[planner] Planning success, starting real-time thread.");

        // 自动启动实时变换线程
        // startRealtimeThread();

        plan_thread_running_ = false; });

        // plan_thread_.detach();
    }

    bool planner::setWaypoint(std::vector<waypoint> &waypoints)
    {
        std::lock_guard<std::mutex> lk(data_mutex_);

        // if (task_status_ != READY)
        // {
        //     log("[planner] Cannot set waypoints: Task not in READY state.\n");
        //     return false;
        // }

        original_waypoints_ = waypoints;

        // 转换航线点，转换到utm坐标系下面
        utm_waypoints_.clear();
        for (const auto &wp : original_waypoints_)
        {
            UTM_waypoint utm_wp;
            Eigen::Vector3d utm_coor = gpsToUtm(wp.location.la, wp.location.lo, wp.location.al, utm_zone_);
            utm_wp.location.x = utm_coor(0);
            utm_wp.location.y = utm_coor(1);
            utm_wp.location.z = utm_coor(2);
            utm_wp.attitude = wp.attitude;
            utm_wp.gimbal = wp.gimbal;
            utm_waypoints_.push_back(utm_wp);
        }
        task_status_ = IN_PROGRESS;
        if (taskStatusCallback_)
            taskStatusCallback_(task_status_);
        log("[planner] Waypoints set successfully. Total waypoints: " + std::to_string(utm_waypoints_.size()) + "\n");

        // 设置航线成功后开始路径规划
        startPlanThread();

        return true;
    }

    bool planner::exeTaskOperation(TaskOperation op)
    {
        // std::lock_guard<std::mutex> lk(data_mutex_);

        switch (op)
        {
        case START:
            if (task_status_ != READY)
            {
                log("[planner] Cannot START: Task not READY.");
                return false;
            }

            log("[planner] Please setWaypoint.");
            return true;

        case STOP:
            if (task_status_ != IN_PROGRESS && task_status_ != READY)
            {
                log("[planner] Cannot STOP: Task not running.");
                return false;
            }

            log("[planner] STOP operation triggered.");

            // 停止实时线程
            stop_thread_ = true;

            // 如果规划线程还在运行，也尝试停止
            plan_thread_running_ = false;

            task_status_ = IDLE;
            if (taskStatusCallback_)
                taskStatusCallback_(task_status_);

            return true;

        default:
            log("[planner] Unsupported TaskOperation.");
            return false;
        }

        return true;
    }

    void planner::setPlannedWaypointsCallback(PlannedWaypointsCallback callback)
    {
        plannedWaypointsCallback_ = callback;
        return;
    }

    void planner::setRealTimeUTMCallback(RealTimeUTMCallback callback)
    {
        realTimeUTMCallback_ = callback;
        return;
    }

    // void planner::setWaypointReachedCallback(WaypointReachedCallback callback)
    // {
    //     waypointReachedCallback_ = callback;
    //     return;
    // }

    void planner::setTaskStatusCallback(TaskStatusCallback callback)
    {
        taskStatusCallback_ = callback;
        return;
    }

    void planner::setLogCallback(LogCallback callback)
    {
        logCallback_ = callback;
        if (Astar_ptr)
        {
            Astar_ptr->setLogCallback(callback);
        }
    }

    void planner::log(const std::string &msg)
    {
        if (logCallback_)
        {
            logCallback_(msg);
        }
    }

    bool planner::tryUpdateLocalToUTMTransform()
    {
        std::lock_guard<std::mutex> lk(data_mutex_);
        if (!imu_offset_ || !attitude_ || !curr_location_)
        {
            // log("[planner] Timestamps not aligned.");
            return false;
        }

        double t1 = imu_offset_->timeStamp;
        double t2 = attitude_->timeStamp;
        double t3 = curr_location_->timeStamp;

        if (!timestampsClose(t1, t2, t3))
        {
            log("[planner] Timestamps not close enough for transform update.\n");
            return false;
        }
        log("[planner] Timestamps aligned. Computing local→UTM transform...");

        Eigen::Vector3d p_utm = gpsToUtm(curr_location_->la, curr_location_->lo, curr_location_->al, utm_zone_);
        Eigen::Matrix3d R = rpyToRotation(attitude_->roll, attitude_->pitch, attitude_->yaw);

        Eigen::Vector3d p_local(imu_offset_->x, imu_offset_->y, imu_offset_->z);
        impl_->loc2utm_T = p_utm - R * p_local;
        impl_->loc2utm_Q = Eigen::Quaterniond(R);

        task_status_ = READY;
        if (taskStatusCallback_)
            taskStatusCallback_(task_status_);
        startRealtimeThread();
        log("[planner] Updated local→UTM transform successfully.\n");

        return true;
    }

    bool planner::timestampsClose(double t1, double t2, double t3)
    {
        double t_min = std::min({t1, t2, t3});
        double t_max = std::max({t1, t2, t3});
        return (t_max - t_min) <= TIME_SYNC_THRESHOLD;
    }

    void planner::stop()
    {
        {
            std::lock_guard<std::mutex> lk(data_mutex_);
            stop_thread_ = true; // 通知实时线程退出
        }

        // 唤醒可能阻塞的 plan 线程
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            map_ready_ = true;
        }
        map_cv_.notify_all();

        // -------- join 两个线程 ----------
        if (realtime_nav_thread_.joinable())
            realtime_nav_thread_.join();

        if (plan_thread_.joinable())
            plan_thread_.join();
    }

    int lonToUTMZone(double lon)
    {
        return int((lon + 180.0) / 6.0) + 1;
    }

    Eigen::Vector3d gpsToUtm(double lat, double lon, double alt, int &utm_zone_)
    {
        // 度 → 弧度
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        // 确定 UTM  Zone
        int zone = lonToUTMZone(lon);
        utm_zone_ = zone;

        double lon0 = (zone - 1) * 6 - 180 + 3; // 中央经线
        double lon0_rad = lon0 * M_PI / 180.0;

        double N = WGS84_A / sqrt(1 - WGS84_E2 * sin(lat_rad) * sin(lat_rad));
        double T = tan(lat_rad) * tan(lat_rad);
        double C = WGS84_E2 / (1 - WGS84_E2) * cos(lat_rad) * cos(lat_rad);
        double A = cos(lat_rad) * (lon_rad - lon0_rad);

        double M = WGS84_A * ((1 - WGS84_E2 / 4 - 3 * WGS84_E2 * WGS84_E2 / 64 - 5 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 256) * lat_rad - (3 * WGS84_E2 / 8 + 3 * WGS84_E2 * WGS84_E2 / 32 + 45 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 1024) * sin(2 * lat_rad) + (15 * WGS84_E2 * WGS84_E2 / 256 + 45 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 1024) * sin(4 * lat_rad) - (35 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 3072) * sin(6 * lat_rad));

        double x = UTM_K0 * N * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * WGS84_E2) * pow(A, 5) / 120) + 500000.0;

        double y = UTM_K0 * (M + N * tan(lat_rad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + (61 - 58 * T + T * T + 600 * C - 330 * WGS84_E2) * pow(A, 6) / 720));

        if (lat < 0)
            y += 10000000.0;

        return Eigen::Vector3d(x, y, alt);
    }

    // RPY -> 旋转矩阵,函数里面用的是弧度，不是角度
    Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw)
    {
        // 如果是角度，需要通过如下变换转换为弧度
        //  double deg2rad = M_PI / 180.0;
        //  roll *= deg2rad;
        //  pitch *= deg2rad;
        //  yaw *= deg2rad;

        Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
        return (rz * ry * rx).toRotationMatrix();
    }

}