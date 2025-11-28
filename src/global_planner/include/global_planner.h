#ifndef GLOBAL_PLANNER_UGV
#define GLOBAL_PLANNER_UGV

#include <iostream>
#include <algorithm>
#include <iostream>
#include <thread>
#include <memory>
#include "planer_utils.h"

namespace global_planner
{
  class Astar;

  using LogCallback = std::function<void(const std::string &)>;
  using PathCallback = std::function<void(const Path &)>;

  class GlobalPlannerUGV
  {
  public:
    GlobalPlannerUGV();
    ~GlobalPlannerUGV();

    bool initFromConfig(const std::string &config_path);
    bool setPointCloud(const std::string &pcd_path);

    void setStart(const Eigen::Vector3d &s);
    void setGoal(const Eigen::Vector3d &g);

    // 异步规划接口
    void setPathCallback(PathCallback cb);
    void setLogCallback(LogCallback cb);

    void planAsync(); // 异步调用 plan()
    std::vector<GpsPoint> convertPathToGPS(const Path &path) const;

  private:
    void log(const std::string &msg);
    std::string vec3ToString(const Eigen::Vector3d &v);
    Path planSyncInternal();
    Eigen::Vector3d gpsToUtm(double lat, double lon, double alt);

  private:
    std::string config_path_;

    // A星规划器
    std::shared_ptr<Astar> Astar_ptr;
    Path path_result_;
    Eigen::Vector3d start_pos_;
    Eigen::Vector3d goal_pos_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    std::string pcd_path_;
    bool set_start_;
    bool set_goal_;

    PathCallback path_callback_;
    LogCallback log_callback_;

    int utm_zone_;

    // ====== 状态 ======
    std::string last_error_;

  public:
    std::thread planning_thread_;
  };

} // namespace global_planner

#endif
