#ifndef GLOBAL_PLANNER_UGV
#define GLOBAL_PLANNER_UGV

#include <iostream>
#include <algorithm>
#include <iostream>
#include <thread>
#include <yaml-cpp/yaml.h>

using namespace std;

namespace global_planner
{
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
  struct GpsPoint
  {
    double lat;
    double lon;
    double alt;
  };

  using LogCallback = std::function<void(const std::string &)>;
  using PathCallback = std::function<void(const Path &)>;

  class GlobalPlannerUGV
  {
  public:
    GlobalPlannerUGV();
    ~GlobalPlannerUGV();

    bool initFromConfig(const std::string &config_path);
    bool setPointCloud(const std::string &pcd_path);

    void setStart(const Vec3 &s);
    void setGoal(const Vec3 &g);

    // 异步规划接口
    void setPathCallback(PathCallback cb);

    void setLogCallback(LogCallback cb);

    std::vector<GpsPoint> convertPathToGPS(const Path &path) const;

    void planAsync(); // 异步调用 plan()

  private:
  };

} // namespace global_planner

#endif
