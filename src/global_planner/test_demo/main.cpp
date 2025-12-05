#include "WaylineManager.h"
#include "utils.hpp"
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h> // 包含 getMinMax3D
#include <iostream>

int main()
{
    std::string map_path = "/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/test_demo/bridge.pcd";
    WaylineManager wayline("/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/test_demo/linepoints.kmz");

    if (!wayline.load())
    {
        return -1;
    }
    auto waypoints = wayline.getWaypoints();
    std::vector<global_planner::UTM_Location> utm_points;
    int zone;
    for (const auto &wp : waypoints)
    {
        // Convert GPS coordinates to UTM
        Eigen::Vector3d Utm_coor = gpsToUtm(wp.location.la, wp.location.lo, wp.location.al, zone);
        utm_points.emplace_back(Utm_coor(0), Utm_coor(1), Utm_coor(2));

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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(map_path, *cloud_) != 0)
    {
        std::cout<<"Failed to load PCD: " + map_path << std::endl;
        return false;
    }

    Eigen::Vector4f min_pt, max_pt;
    // 使用 pcl::getMinMax3D 计算点云的最小和最大点
    pcl::getMinMax3D(*cloud_, min_pt, max_pt);

    std::cout << "\n--- PCD Map Bounding Box (UTM) ---" << std::endl;
    std::cout << "Min Point (X, Y, Z): ("
              << min_pt[0] << ", " << min_pt[1] << ", " << min_pt[2] << ")" << std::endl;
    std::cout << "Max Point (X, Y, Z): ("
              << max_pt[0] << ", " << max_pt[1] << ", " << max_pt[2] << ")" << std::endl;
    std::cout << "-----------------------------------" << std::endl;

    int points_inside_box = 0;
    int points_outside_box = 0;

    std::cout << "\n--- Waypoint Bounding Box Check ---" << std::endl;
    for (size_t i = 0; i < utm_points.size(); ++i)
    {
        const auto& utm_p = utm_points[i];
        
        // 假设 global_planner::UTM_Location 有 X, Y, Z 成员
        double p_x = utm_p.x;
        double p_y = utm_p.y;
        double p_z = utm_p.z;

        bool is_inside = true;

        // X 轴检查
        if (p_x < min_pt[0] || p_x > max_pt[0]) {
            is_inside = false;
        }

        // Y 轴检查
        if (p_y < min_pt[1] || p_y > max_pt[1]) {
            is_inside = false;
        }

        // Z 轴检查
        // 注意：Z轴的检查可能需要根据你的应用需求调整，
        // 比如是否允许无人机飞高于或低于地图的Z轴范围。
        if (p_z < min_pt[2] || p_z > max_pt[2]) {
            is_inside = false;
        }
        
        if (is_inside) {
            std::cout << "Waypoint " << i << " (X:" << p_x << ", Y:" << p_y <<", Z:" << p_z << ") is **INSIDE** the map boundary." << std::endl;
            points_inside_box++;
        } else {
            std::cout << "Waypoint " << i << " (X:" << p_x << ", Y:" << p_y << ", Z:" << p_z << ") is **OUTSIDE** the map boundary." << std::endl;
            points_outside_box++;
        }
    }

    std::cout << "\n--- Summary ---" << std::endl;
    std::cout << "Total Waypoints: " << utm_points.size() << std::endl;
    std::cout << "Inside Boundary: " << points_inside_box << std::endl;
    std::cout << "Outside Boundary: " << points_outside_box << std::endl;

    return 0; // 成功退出
}

