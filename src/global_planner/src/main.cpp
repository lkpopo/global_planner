#include <iostream>
#include "planner.h" // 你库里的头文件路径记得改
#include <iomanip>   // 包含设置精度的函数
#include "WaylineManager.h"
#include <Eigen/Dense>
#include "utils.hpp"
#include <unistd.h>

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

    // bool res = loc.setConfig("/home/dji/dpf/global_planner/src/config.yaml");
    // loc.setMap("/home/dji/dpf/bridge.pcd");
    WaylineManager wayline("/home/dpf/Workspace/global_planner/src/global_planner/data/linepoints.kmz");

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

    std::cout << "Test OK!" << std::endl;
    return 0;
}