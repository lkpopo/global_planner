#include <iostream>
#include "planner.h" // 你库里的头文件路径记得改
#include <iomanip>   // 包含设置精度的函数
#include "WaylineManager.h"

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
                                        }
                                    });

    bool res = loc.setConfig("/home/zxhc/Workspace/ROS2_WS/global_planner/src/config.yaml");
    if (!res)
    {
        std::cout << "failed init config" << std::endl;
        return -1;
    }
    std::string path = "/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/test_demo/bridge.pcd";
    loc.setMap(path);
    std::string map_path = "/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/test_demo/bridge.pcd";
    WaylineManager wayline("/home/zxhc/Workspace/ROS2_WS/global_planner/src/global_planner/test_demo/linepoints.kmz");

    if (!wayline.load())
    {
        return -1;
    }
    auto waypoints = wayline.getWaypoints();

    loc.setWaypoint(waypoints);

    loc.plan_thread_.join();
    loc.exeTaskOperation(global_planner::STOP);

    // 纬度-经度-海拔
    // loc.setStart(Eigen::Vector3d( 33.30569241579879, 117.53958862130553, 240.0)); // 示例起点 UTM 坐标
    // loc.setGoal(Eigen::Vector3d(33.30508034861615, 117.54136865442646, 245.0));  // 示例终点 UTM 坐标
    // loc.planAsync();
    // loc.planning_thread_.join();
    // global_planner::Path p;
    // p.poses.emplace_back(Eigen::Vector3d(550231.876833,3685306.749988,240.000000));
    // p.poses.emplace_back(Eigen::Vector3d(550397.939351,3685239.749988,245.000000));
    // auto gps_ls = loc.convertPathToGPS(p);
    // for(auto gps:gps_ls)
    // {
    //     std::cout << "设置 setprecision(15): " << std::setprecision(8)<<"gps lat: "<<gps.lat<<".  lon: "<<gps.lon<<".  alt: "<<gps.alt<<std::endl;
    // }
    std::cout << "Test OK!" << std::endl;
    return 0;
}