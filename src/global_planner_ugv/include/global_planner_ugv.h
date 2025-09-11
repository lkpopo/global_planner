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

namespace global_planner_ugv
{

class GlobalPlannerUGV : public rclcpp::Node
{
  public:
    GlobalPlannerUGV()
    : Node("global_planner_ugv")
    {
    }

    ~GlobalPlannerUGV() {}

    void init();

private:

    // 【订阅】目标点（手动模式）
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub;
    // 【订阅】传感器数据 - 全局点云、局部点云、Scan
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Gpointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr Lpointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub;

    // 【发布】规划路径
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_cmd_pub;
    // 【定时器】主循环定时器、路径追踪定时器、目标追踪定时器
    rclcpp::TimerBase::SharedPtr mainloop_timer;
    rclcpp::TimerBase::SharedPtr track_path_timer;
    rclcpp::TimerBase::SharedPtr send_nei_odom_timer;
    // 五种状态机
    enum EXEC_STATE
    {
      INIT,
      WAIT_GOAL,
      PLAN,
      PATH_TRACKING,
      RETURN,
      STOP
    };
    EXEC_STATE exec_state;
    // 无人车名字                             
    string ugv_name;    
    float k_p,k_aoivd; 
    // 是否仿真模式
    bool sim_mode;    
    // 无人车编号                         
    int ugv_id;                                     
    double depth;
    double angle_y;
    // 手动给定目标点模式 或 自动目标点模式
    bool manual_mode;
    // 传感器输入flag
    int map_input_source;
    // 路径重规划时间
    double replan_time;
    double track_frequency;
    int counter_search;
    
    Eigen::Vector3d state_nei[10];
    bool get_nei_state[10];


    // A星规划器
    Astar::Ptr Astar_ptr;
    // A星规划器状态
    int astar_state;

    float ugv_state[3];

    std::string pcd_path;
    
    //yaw
    float ugv_yaw;
    // 规划得到的路径
    nav_msgs::msg::Path path_cmd;
    // 路经点开始id
    int start_point_index;
    // 路经点总数
    int Num_total_wp;
    // 当前执行ID
    int cur_id;
    // 距离上一次重置，移动的距离（重规划路径时重置）
    float distance_walked;
    float yaw_ref;
    // 上一次重置时，无人车的位置
    Eigen::Vector3d ugv_pos_last;
    // 规划初始状态及终端状态
    Eigen::Vector3d start_pos, goal_pos, goal_vel;
    // 返航位置
    Eigen::Vector3d return_pos;
    // 规划器状态
    bool get_goal; 
    bool in_return_mode;
    bool path_ok;
    bool rotate_in_place;  // vinson add
    bool start_move; // vinson add
    bool get_target_pos;
    // 目标位置
    geometry_msgs::msg::PoseStamped target_pos;
    // 上一条轨迹开始时间
    rclcpp::Time tra_start_time;    
    
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
