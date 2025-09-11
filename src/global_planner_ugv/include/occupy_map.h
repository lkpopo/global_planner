#ifndef _OCCUPY_MAP_H
#define _OCCUPY_MAP_H

#include <iostream>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/occupancy_grid.h>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <map>
#include "printf_utils.h"
using namespace std;

namespace global_planner_ugv
{

class Occupy_map
{
    public:
        Occupy_map(){}
        // 全局地图点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_point_cloud_map;
        // 全局膨胀点云指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_ugv_pcl;
        // 考虑变指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inflate_vis_;
        // 临时指针
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_point_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_ptr;
        // 地图边界点云
        pcl::PointCloud<pcl::PointXYZ> border;
        // VoxelGrid过滤器用于下采样
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        // OutlierRemoval用于去除离群值
	    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // laserscan2pointcloud2 中间变量
        sensor_msgs::msg::PointCloud2 input_laser_scan;
        // laserscan2pointcloud2 投影器
        laser_geometry::LaserProjection projector_;
        // 上一帧odom
        double f_x, f_y, f_z, f_roll, f_pitch, f_yaw;
        // 局部地图滑窗，指示器以及大小
        int st_it, queue_size;
        // flag：展示地图边界
        bool show_border;
        bool sim_mode;
        // 地图是否占据容器， 从编程角度来讲，这就是地图变为单一序列化后的索引
        std::vector<int> occupancy_buffer_;  // 0 is free, 1 is occupied
        // 代价地图
        std::vector<double> cost_map_;  // cost
        // 地图分辨率
        double resolution_, inv_resolution_;
        string node_name;
        // 膨胀参数
        double inflate_;
        double odom_inflate_;
        // 地图原点,地图尺寸
        Eigen::Vector3d origin_, map_size_3d_, min_range_, max_range_;
        // 占据图尺寸 = 地图尺寸 / 分辨率
        Eigen::Vector3i grid_size_;
        int swarm_num_ugv;                                  // 集群数量
        string ugv_name;                                // 无人机名字
        int ugv_id;                                     // 无人机编号
        bool has_global_point;
        bool get_gpcl,get_lpcl,get_laser;
        Eigen::Vector3d enum_p[100], enum_p_ugv[1000], enum_p_cost[1000];
        int ifn;
        int inflate_index, inflate_index_ugv, cost_index, cost_inflate;
        // 发布点云用于rviz显示
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pcl_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inflate_pcl_pub_;
        rclcpp::TimerBase::SharedPtr pcl_pub_timer_;
        rclcpp::Node::SharedPtr node_;


        double ugv_height;
        //初始化
        void init(rclcpp::Node::SharedPtr node);
        // 地图更新函数 - 输入：全局点云
        void map_update_gpcl(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> & global_point);
        // 地图膨胀
        void inflate_point_cloud(void);
        // 判断当前点是否在地图内
        bool isInMap(Eigen::Vector3d pos);
        // 设置占据
        void setOccupancy(Eigen::Vector3d &pos, int occ);
        // 设置代价
        void updateCostMap(Eigen::Vector3d &pos, double cost);
        // 由位置计算索引
        void posToIndex(Eigen::Vector3d &pos, Eigen::Vector3i &id);
        // 由索引计算位置
        void indexToPos(Eigen::Vector3i &id, Eigen::Vector3d &pos);
        // 根据位置返回占据状态
        int getOccupancy(Eigen::Vector3d &pos);
        // 根据索引返回占据状态
        int getOccupancy(Eigen::Vector3i &id);
        // 根据索引返回代价
        double getCost(Eigen::Vector3d &pos);
        // 检查安全
        bool check_safety(Eigen::Vector3d &pos, double check_distance/*, Eigen::Vector3d& map_point*/);
        // void pub_pcl_cb(const ros::TimerEvent& e);
        void pub_pcl_cb();

        // 定义该类的指针
        typedef std::shared_ptr<Occupy_map> Ptr;
};

}

#endif