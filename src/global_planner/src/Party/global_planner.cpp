#include "global_planner.h"

namespace global_planner
{

    GlobalPlannerUGV::GlobalPlannerUGV()
    {

    }

    GlobalPlannerUGV::~GlobalPlannerUGV()
    {
        // stop();
    }

    bool GlobalPlannerUGV::initFromConfig(const std::string &config_path)
    {
        // TODO: 读取配置
        return true;
    }

    bool GlobalPlannerUGV::setPointCloud(const std::string &pcd_path)
    {
        return true;
    }

    void GlobalPlannerUGV::setPathCallback(PathCallback cb)
    {

    }

    void GlobalPlannerUGV::setLogCallback(LogCallback cb)
    {
        
    }

    void GlobalPlannerUGV::setStart(const Vec3 &s)
    {

    }

    std::vector<GpsPoint> GlobalPlannerUGV::convertPathToGPS(const Path& path) const
    {

    }

    void GlobalPlannerUGV::setGoal(const Vec3 &g)
    {
    }

    void GlobalPlannerUGV::planAsync()
    {
    }


} // namespace global_planner
