#pragma once

#include <string>
#include <vector>
#include <functional>
#include <mutex>
#include <atomic>
#include <thread>
#include <memory>
#include <optional>
#include <condition_variable>

namespace global_planner
{

    class Astar;

    // 定义枚举 当前任务状态
    enum TaskStatus
    {
        IDLE,        // 未准备好
        READY,       // 准备好
        IN_PROGRESS, // 进行中
        COMPLETED,   // 已经完成
        FAILED       // 失败
    };

    // 定义任务的操作类型
    enum TaskOperation
    {
        START, // READY之后才可以开始
        STOP,  // 停止任务后就变成了idle
        PAUSE,
        RESUME
    };

    // 实时的偏移xyz
    struct uavImu
    {
        double x = 0, y = 0, z = 0;
        double timeStamp = 0.0;
    };

    // 当前的gps坐标
    struct Location
    {
        double la = 0;
        double lo = 0;
        double al = 0;
        double timeStamp = 0.0;
    };

    // 当前的无人机姿态信息
    struct uavAttitude
    {
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
        double timeStamp = 0.0;
    };

    // 云台的姿态
    struct gimbalAttitude
    {
        double roll = 0;
        double pitch = 0;
        double yaw = 0;
    };

    // 无人机航点信息
    struct waypoint
    {
        Location location;
        uavAttitude attitude;
        gimbalAttitude gimbal;
    };

    // 规划好的航线点,UTM坐标系下的点
    struct UTM_Location
    {
        double x = 0, y = 0, z = 0;
        UTM_Location() {}
        UTM_Location(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    // UTM坐标系下的航点信息
    struct UTM_waypoint
    {
        UTM_Location location;
        uavAttitude attitude;
        gimbalAttitude gimbal;
    };

    // 返回到达点位时，的航点相关信息
    struct reachedPoint
    {
        UTM_waypoint wp;
        int index = 0;
        int total = 0;
    };

    using PlannedWaypointsCallback = std::function<void(const std::vector<UTM_Location> &)>;
    using RealTimeUTMCallback = std::function<void(const UTM_Location &)>;
    using WaypointReachedCallback = std::function<void(const reachedPoint &)>;
    using TaskStatusCallback = std::function<void(TaskStatus)>;
    using LogCallback = std::function<void(const std::string &)>;

    class planner
    {
    public:
        /*** === 子模块 === ***/
        std::shared_ptr<Astar> Astar_ptr;

        /*** === 回调函数 === ***/
        PlannedWaypointsCallback plannedWaypointsCallback_;
        RealTimeUTMCallback realTimeUTMCallback_;
        WaypointReachedCallback waypointReachedCallback_;
        TaskStatusCallback taskStatusCallback_;
        LogCallback logCallback_;

        /*** === 任务状态 === ***/
        std::atomic<TaskStatus> task_status_{IDLE};
        std::mutex state_mutex_; // 控制状态操作
        std::mutex data_mutex_;  // 控制数据输入

        /*** === 主线程相关 === ***/
        std::thread realtime_nav_thread_, plan_thread_;
        void startRealtimeThread();
        void startPlanThread();
        std::atomic<bool> stop_thread_{false};
        bool plan_thread_running_;

        std::mutex map_mutex_;
        std::condition_variable map_cv_;
        bool map_ready_ = false;
        bool map_loading_ = false;


        // local——>UTM坐标系转换
        bool tryUpdateLocalToUTMTransform();
        // UTM坐标转换的环带号
        int utm_zone_;
        // 判断时间戳是否接近
        bool timestampsClose(double t1, double t2, double t3);

        /*** === 输入数据 === ***/
        std::optional<Location> curr_location_;
        std::optional<uavImu> imu_offset_;
        std::optional<uavAttitude> attitude_;
        std::vector<waypoint> original_waypoints_;
        std::vector<UTM_waypoint> utm_waypoints_;

        /*** === 输出数据 === ***/
        std::vector<UTM_Location> full_path_;

        /*** === 封装的三方库变量 === ***/
        // struct Impl;
        // std::unique_ptr<Impl> impl_;

        /*** === 内部私有方法 === ***/
        void log(const std::string &msg);
        bool planWaypointsPath();

    public:
        planner();
        ~planner();
        // 地图路径
        bool setMap(const std::string &map_path);

        bool setConfig(const std::string &config_path);

        // 偏移xyz,实时给我的
        bool setOffset(uavImu imu);

        // 无人机的朝向,实时给我
        bool setUavAttitude(uavAttitude attitude);

        // 无人机当前位置,实时给我。
        bool setCurrLocation(Location location);

        // 设置无人机的航线点,这个地方可以设置一个线程锁，等我的task状态时ready的时候才可以执行
        // 或者直接返回设置失败false
        bool setWaypoint(std::vector<waypoint> &waypoints);

        // 设置返回规划好航点的callback
        void setPlannedWaypointsCallback(PlannedWaypointsCallback callback);

        // 设置返回实时UTM坐标系下位置的callback
        void setRealTimeUTMCallback(RealTimeUTMCallback callback);

        // 设置到达途径点时的callback,返回的时之前给我的航点信息
        void setWaypointReachedCallback(WaypointReachedCallback callback);

        // 设置获取当前航线状态的callback
        void setTaskStatusCallback(TaskStatusCallback callback);

        // 设置日志回调函数
        void setLogCallback(LogCallback callback);

        // 给外部的接口 用于开始和停止我的任务
        bool exeTaskOperation(TaskOperation status);

        // 所以我这边有一个主线程跟外部进行对接，然后收到start相关信号的时候开启一个新的线程执行路径规划。
    };
}