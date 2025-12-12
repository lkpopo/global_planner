#include <iostream>
#include "planner.h" // 你库里的头文件路径记得改
#include <iomanip>   // 包含设置精度的函数
#include "WaylineManager.h"
#include <Eigen/Dense>
#include "utils.hpp"
#include <unistd.h>

global_planner::Location g_sim_loc = {39.9042, 116.4074, 50.0, 0.0};
global_planner::uavAttitude g_sim_att = {0, 0, 0, 0.0};
global_planner::uavImu g_sim_imu = {0, 0, 0, 0.0};
std::mutex g_sensor_mutex; // 保护传感器数据读写
std::atomic<bool> g_sim_running{true};      // 模拟器开关
std::atomic<bool> g_program_running{true};   // 程序主循环开关

// ================= 辅助函数：传感器模拟线程 =================
void sensor_simulation_thread(global_planner::planner *gp)
{
    while (g_program_running) {
        if (g_sim_running) {
            // 获取当前系统时间作为统一时间戳
            double current_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()
            ).count() / 1000.0;

            // 1. 更新全局模拟数据
            {
                std::lock_guard<std::mutex> lk(g_sensor_mutex);
                g_sim_loc.timeStamp = current_time;
                g_sim_att.timeStamp = current_time;
                g_sim_imu.timeStamp = current_time;
            }

            // 2. 注入数据到 planner
            // planner 内部会检查这三个数据的时间戳是否同步 (<50ms)
            gp->setCurrLocation(g_sim_loc);
            gp->setUavAttitude(g_sim_att);
            gp->setOffset(g_sim_imu);
        }

        // 模拟 20Hz 频率 (50ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

int main()
{
    global_planner::planner loc;

    loc.setLogCallback([](const std::string &msg)
                       { std::cout << "[LOG] " << msg << std::endl; });

    loc.setPlannedWaypointsCallback([](const std::vector<global_planner::UTM_Location> &path)
                                    {
                                        for (auto p : path)
                                        {
                                            std::cout << p.x << "," << p.y << "," << p.z << std::endl;
                                        } });

    // bool res = loc.setConfig("/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/data/config.yaml");
    // bool res = loc.setConfig("/home/dji/dpf/global_planner/src/config.yaml");

    //启动设置线程
    std::thread sim_thread(sensor_simulation_thread, &loc);
    
    std::string path = "/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/data/bridge.pcd";
    // std::string path = "/home/dji/dpf/bridge.pcd";
    // loc.setMap(path);
    WaylineManager wayline("/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/data/linepoints.kmz");
    // WaylineManager wayline("/home/dji/dpf/新建航点飞行7.kmz");

    if (!wayline.load())
    {
        return -1;
    }
    auto waypoints = wayline.getWaypoints();
    for (const auto &wp : waypoints)
    {
        // Convert GPS coordinates to UTM
        int zone;
        Eigen::Vector3d Utm_coor = gpsToUtm(wp.location.la, wp.location.lo, wp.location.al, zone);

        std::cout << "Waypoint: "
                  << "Lon: " << wp.location.lo << ", "
                  << "Lat: " << wp.location.la << ", "
                  << "Alt: " << wp.location.al << ", "
                  << "UAV Yaw: " << wp.attitude.yaw << ", "
                  << "UAV Roll: " << wp.attitude.roll << ", "
                  << "UAV Pitch: " << wp.attitude.pitch << ", "
                  << "Gimbal Roll: " << wp.gimbal.roll << ", "
                  << "Gimbal Pitch: " << wp.gimbal.pitch << ", "
                  << "Gimbal Yaw: " << wp.gimbal.yaw
                  << std::endl;
        std::cout << "Converted UTM: "
                  << "X: " << Utm_coor(0) << ", "
                  << "Y: " << Utm_coor(1) << ", "
                  << "Z: " << Utm_coor(2) << std::endl;
    }

    loc.setWaypoint(waypoints);

    // sleep(2); // 等待一秒以确保前面的设置完成
    // loc.setWaypoint(waypoints);
    loc.stop();
    g_program_running=false;
    if (sim_thread.joinable()) {
        sim_thread.join();
    }
    sleep(5);

    std::cout << "Test OK!" << std::endl;
    return 0;
}