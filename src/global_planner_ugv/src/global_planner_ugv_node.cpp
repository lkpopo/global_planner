#include <rclcpp/rclcpp.hpp>
#include "global_planner_ugv.h"

using namespace global_planner_ugv;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 创建继承自 Node 的 GlobalPlannerUGV 对象
    auto ugv_global_planner = std::make_shared<global_planner_ugv::GlobalPlannerUGV>();

    // init() 不需要参数，或者可以直接在构造函数里完成初始化
    ugv_global_planner->init();

    // 直接 spin 这个节点
    rclcpp::spin(ugv_global_planner);
    rclcpp::shutdown();
    return 0;
}
