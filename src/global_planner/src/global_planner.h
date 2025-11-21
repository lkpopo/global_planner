#ifndef GLOBAL_PLANNER_UGV
#define GLOBAL_PLANNER_UGV

#include <iostream>
#include <algorithm>
#include <iostream>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>

// #include "A_star.h"
// #include "occupy_map.h"
// #include "printf_utils.h"

using namespace std;

struct Vec3
{
  float x, y, z;
};

struct Pose
{
  Vec3 position;
};

struct Path
{
  std::vector<Pose> poses;
};

namespace global_planner
{

  class GlobalPlannerUGV
  {
  public:
    GlobalPlannerUGV() {}

    ~GlobalPlannerUGV() {}

    // 配置初始化
    bool initFromConfig(const std::string& config_path);

    // 设置地图
    bool setPointCloud(const std::string& pcd_path);

    // 设置起点/终点
    void setStart(const Vec3& s);
    void setGoal(const Vec3& g);

    // 执行规划
    bool plan();

    // 获取结果
    Path getPath() const;

  private:
    // 初始点和目标点坐标
    Vec3 start_pos, goal_pos;

    // 点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;

    // 规划得到的路径
    Path path_res;

    // A*规划器
    // Astar::Ptr Astar_ptr;

    // A*规划器状态
    int astar_state;
  };

}

#endif
