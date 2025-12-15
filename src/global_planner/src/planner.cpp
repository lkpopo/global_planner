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

    // WGS84 椭球体参数
    static constexpr double WGS84_A = 6378137.0;               // 长半轴
    static constexpr double WGS84_F = 1.0 / 298.257223563;     // 扁率
    static constexpr double WGS84_B = WGS84_A * (1 - WGS84_F); // 短半轴
    static constexpr double k0 = 0.9996;                       // UTM 比例因子

    std::unique_ptr<Impl> impl_ = std::make_unique<Impl>();

    global_planner::UTM_Location gpsToUtm(double lat, double lon, double alt);

    planner::planner()
    {
        Astar_ptr = std::make_shared<Astar>();
        plannedWaypointsCallback_ = nullptr;
        realTimeUTMCallback_ = nullptr;
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

            auto segment = Astar_ptr->getPath();
            Astar_ptr->reset(); // 重置以便下一次搜索

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

    bool planner::planWaypointsPath_withoutMap()
    {
        full_path_.clear();

        auto curr_p = utm_waypoints_.front().location; // 起点
        int index = 0;
        for (size_t i = 0; i < utm_waypoints_.size(); ++i)
        {
            UTM_Location goal_p = utm_waypoints_[i].location;
            goal_p.index = index++;
            full_path_.push_back(goal_p);
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
                std::lock_guard<std::mutex> lock(mutex_);
                global_planner::UTM_Location result;

                result.x = origin_utm_.x + imu_offset_->y; // East
                result.y = origin_utm_.y + imu_offset_->x; // North
                result.z = origin_utm_.z + imu_offset_->z; // Up
                if (realTimeUTMCallback_)
                {
                    realTimeUTMCallback_(result);
                }
            }

            
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

        if (plan_thread_.joinable())
        {
            log("[planner] Joining previous planning thread.");
            plan_thread_.join();
        }

        plan_thread_running_ = true;

        plan_thread_ = std::thread([this]()
                                   {
        log("[planner] Waiting for map to be ready.");
        {
            // 阻塞等待地图加载完成
            std::unique_lock<std::mutex> lock(map_mutex_);
            map_cv_.wait(lock, [this] { return map_ready_; });
        }
        log("[planner] Planning thread started.");

        bool success = planWaypointsPath();
        // bool success = planWaypointsPath_withoutMap();

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

        // 重新进入READY状态
        task_status_ = READY;
        if (taskStatusCallback_) taskStatusCallback_(task_status_);

        log("[planner] Planning success, starting real-time thread.");

        plan_thread_running_ = false; });
    }

    bool planner::setWaypoint(std::vector<waypoint> &waypoints)
    {
        {
            std::lock_guard<std::mutex> lk(data_mutex_);

            if (task_status_ != READY)
            {
                log("[planner] Cannot set waypoints: Task not in READY state(local→UTM transform not ready or the task is being planned.)");
                return false;
            }

            original_waypoints_ = waypoints;

            // 转换航线点，转换到utm坐标系下面
            utm_waypoints_.clear();
            for (const auto &wp : original_waypoints_)
            {
                UTM_waypoint utm_wp;
                global_planner::UTM_Location utm_coor = gpsToUtm(wp.location.la, wp.location.lo, wp.location.al);
                utm_wp.location = utm_coor;
                utm_wp.attitude = wp.attitude;
                utm_wp.gimbal = wp.gimbal;
                utm_waypoints_.push_back(utm_wp);
            }
            task_status_ = IN_PROGRESS;
            if (taskStatusCallback_)
                taskStatusCallback_(task_status_);
            log("[planner] Waypoints set successfully. Total waypoints: " + std::to_string(utm_waypoints_.size()) + "\n");
        }
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
            // do nothing!
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

            stop();
            log("[planner] All threads stopped.");

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
        if (!imu_offset_ || !curr_location_)
        {
            return false;
        }

        double t1 = imu_offset_->timeStamp;
        double t2 = curr_location_->timeStamp;

        if (!timestampsClose(t1, t2))
        {
            log("[planner] Timestamps not close enough for transform update.\n");
            return false;
        }
        log("[planner] Timestamps aligned. Computing local→UTM transform...");

        // 1. 将当前 GPS 转为 UTM
        global_planner::UTM_Location current_utm_gps = gpsToUtm(curr_location_->la, curr_location_->lo, curr_location_->al);

        // UTM X (东) = 当前UTM东 - SLAM的东向分量(即 slam_y)
        origin_utm_.x = current_utm_gps.x - imu_offset_->y;

        // UTM Y (北) = 当前UTM北 - SLAM的北向分量(即 slam_x)
        origin_utm_.y = current_utm_gps.y - imu_offset_->x;

        origin_utm_.z = current_utm_gps.z - imu_offset_->z;

        is_aligned_ = true;

        task_status_ = READY;
        if (taskStatusCallback_)
            taskStatusCallback_(task_status_);
        startRealtimeThread();
        log("[planner] Updated local→UTM transform successfully.\n");

        return true;
    }

    bool planner::timestampsClose(double t1, double t2)
    {
        double t_min = std::min({t1, t2});
        double t_max = std::max({t1, t2});
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

    global_planner::UTM_Location gpsToUtm(double lat, double lon, double alt)
    {
        global_planner::UTM_Location result;
        // result.north = (lat >= 0);
        result.z = alt; // 【新增】高度直接透传，不需要投影计算

        // 1. 计算 UTM 区域号 (Zone)
        int zone = (int)((lon + 180.0) / 6.0) + 1;

        // 计算该区域的中央子午线经度 (Central Meridian)
        double lon0 = -183.0 + (zone * 6.0);

        // 角度转弧度
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        double lon0_rad = lon0 * M_PI / 180.0;

        // 2. 椭球体计算中间变量
        double e = std::sqrt(1 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A)); // 第一偏心率
        double e2 = e * e;
        double e4 = e2 * e2;
        double e6 = e4 * e2;

        double N = WGS84_A / std::sqrt(1 - e2 * std::sin(lat_rad) * std::sin(lat_rad)); // 卯酉圈曲率半径
        double T = std::tan(lat_rad) * std::tan(lat_rad);
        double C = e2 * std::cos(lat_rad) * std::cos(lat_rad) / (1 - e2);
        double A = (lon_rad - lon0_rad) * std::cos(lat_rad);

        // 3. 计算子午线弧长 M
        double M = WGS84_A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat_rad - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * std::sin(2 * lat_rad) + (15 * e4 / 256 + 45 * e6 / 1024) * std::sin(4 * lat_rad) - (35 * e6 / 3072) * std::sin(6 * lat_rad));

        // 4. 计算 UTM 坐标 (x, y)
        // 东坐标 (Easting)
        result.x = k0 * N * (A + (1 - T + C) * A * A * A / 6 + (5 - 18 * T + T * T + 72 * C - 58 * e2) * A * A * A * A * A / 120) + 500000.0; // 加上 500km 伪东偏移

        // 北坐标 (Northing)
        result.y = k0 * (M + N * std::tan(lat_rad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 + (61 - 58 * T + T * T + 600 * C - 330 * e2) * A * A * A * A * A * A / 720));

        // 如果是南半球，加上 10,000km 伪北偏移
        // if (!result.north) {
        //     result.y += 10000000.0;
        // }

        return result;

        // return Eigen::Vector3d(x, y, alt);
    }

}