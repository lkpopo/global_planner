#include "planner.h"

namespace global_planner
{

    planner::planner()
    {
    }
    planner::~planner()
    {
    }

    bool planner::setMap(const std::string &map_path)
    {
        return true;
    }

    bool planner::setConfig(const std::string &config_path)
    {
        return true;
    }

    bool planner::setOffset(uavImu imu)
    {
        return true;
    }

    bool planner::setUavAttitude(uavAttitude attitude)
    {
        return true;
    }

    bool planner::setCurrLocation(Location location)
    {
        return true;
    }

    bool planner::setWaypoint(std::vector<waypoint> &waypoints)
    {
        return true;
    }

    bool planner::exeTaskOperation(TaskOperation status)
    {
        // TODO: Implement this method

        return true;
    }

    void planner::setPlannedWaypointsCallback(PlannedWaypointsCallback callback)
    {
    }

    void planner::setRealTimeUTMCallback(RealTimeUTMCallback callback)
    {
    }

    void planner::setWaypointReachedCallback(WaypointReachedCallback callback)
    {
    }

    void planner::setTaskStatusCallback(TaskStatusCallback callback)
    {
    }

    void planner::setLogCallback(LogCallback callback)
    {
    }

}