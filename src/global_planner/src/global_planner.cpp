#include "global_planner.h"

namespace global_planner
{
    // 初始化函数
    bool GlobalPlannerUGV::initFromConfig(const std::string& config_path)
    {
    }

    bool GlobalPlannerUGV::setPointCloud(const std::string& pcd_path)
    {
    
    }

    void GlobalPlannerUGV::setStart(const Vec3& s)
    {
        start_pos = s;
    }

    void GlobalPlannerUGV::setGoal(const Vec3& g)
    {
        goal_pos = g;
    }

    bool GlobalPlannerUGV::plan()
    {
        return true;
    }

    Path GlobalPlannerUGV::getPath() const
    {
        Path path;
        return path;
    }
}