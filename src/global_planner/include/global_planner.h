#ifndef GLOBAL_PLANNER_UGV
#define GLOBAL_PLANNER_UGV

#include <rclcpp/rclcpp.hpp>
#include <boost/format.hpp>

#include <Eigen/Eigen>
#include <iostream>
#include <algorithm>
#include <iostream>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "A_star.h"
#include "occupy_map.h"
#include "printf_utils.h"

#include "angles/angles.h"
#include <tf2/utils.h> // getYaw

using namespace std;

#define MIN_DIS 0.1

namespace global_planner
{

class GlobalPlannerUGV : public rclcpp::Node
{
  public:
    GlobalPlannerUGV()
    : Node("global_planner")
    {
    }

    ~GlobalPlannerUGV() {}

    void init();

private:

    // 【订阅】目标点（手动模式）
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    // 【订阅】传感器数据 - 全局点云、局部点云、Scan
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Gpointcloud_sub;

    // 【发布】规划路径
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_cmd_pub;
    // 【定时器】主循环定时器、路径追踪定时器、目标追踪定时器
    rclcpp::TimerBase::SharedPtr mainloop_timer;
    // 三种状态机
    enum EXEC_STATE
    {
      INIT,
      WAIT_GOAL,
      PLAN
    };
    EXEC_STATE exec_state;

    int counter_search;

    // A星规划器
    Astar::Ptr Astar_ptr;
    // A星规划器状态
    int astar_state;
    // pcd点云路径
    std::string pcd_path;
    
    // 规划得到的路径
    nav_msgs::msg::Path path_cmd;

    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, goal_pos;

    // 规划器状态
    bool get_goal; 
    bool path_ok;
    
    // 回调函数
    void goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void Gpointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void mainloop_cb();
   
    // 【获取当前时间函数】 单位：秒
    float get_time_in_sec(const rclcpp::Time& begin_time);
    void printf_exec_state();

};

}

#endif
