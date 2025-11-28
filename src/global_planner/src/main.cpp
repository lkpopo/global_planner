#include <iostream>
#include "global_planner.h"   // 你库里的头文件路径记得改



int main() {
    global_planner::GlobalPlannerUGV loc;
    
    loc.setLogCallback([](const std::string &msg){
    std::cout << "[LOG] " << msg << std::endl;
});
    loc.setPathCallback([](const global_planner::Path& path) {
        std::cout << "Path poses size = " << path.poses.size() << std::endl;
    });
    bool res = loc.initFromConfig("/home/zxhc/Workspace/Test/global_planner/src/config.yaml");
    if(!res)
    {
        std::cout<<"failed init config"<<std::endl;
        return -1;
    }
    std::string path="/home/zxhc/Workspace/Test/Test_code/test.pcd";
    loc.setPointCloud(path);
    loc.setStart(Eigen::Vector3d( 33.30569241579879, 117.53958862130553, 240.0)); // 示例起点 UTM 坐标
    loc.setGoal(Eigen::Vector3d(33.30508034861615, 117.54136865442646, 245.0));  // 示例终点 UTM 坐标
    loc.planAsync();
    loc.planning_thread_.join();
    std::cout << "Test OK!" << std::endl;
    return 0;
}