#include "global_planner.h"

namespace global_planner
{
    // 初始化函数
    void GlobalPlannerUGV::init()
    {
        // 读取参数
        // A星算法 重规划频率
        pcd_path = this->declare_parameter<std::string>("global_planner/pcd_path", "");
        start_pos[0] = this->declare_parameter("start_pos_x", 0.0);
        start_pos[1] = this->declare_parameter("start_pos_y", 0.0);
        start_pos[2] = this->declare_parameter("start_pos_z", 0.0);

        // Astar algorithm
        Astar_ptr = std::make_shared<Astar>();
        Astar_ptr->init(this->shared_from_this());

        goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/global_planner/goal",
            rclcpp::QoS(10),
            std::bind(&GlobalPlannerUGV::goal_cb, this, std::placeholders::_1));

        // 【发布】路径用于显示（rviz显示）
        path_cmd_pub = this->create_publisher<nav_msgs::msg::Path>("/global_planner/path_cmd",
            rclcpp::QoS(10));

        // 【定时器】主循环执行
        mainloop_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GlobalPlannerUGV::mainloop_cb, this));

        if (!pcd_path.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Loading static global map from PCD: %s", pcd_path.c_str());
            const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
            {
                RCLCPP_FATAL(this->get_logger(), "No PCD file path provided! Set parameter 'global_planner/pcd_path'.");
                rclcpp::shutdown();
                return;
            }
            else
            {
                // 更新到占据地图
                Astar_ptr->Occupy_map_ptr->map_update_gpcl(cloud);
                RCLCPP_INFO(this->get_logger(), "Global map initialized from PCD file.");
            }

            // 规划器状态参数初始化
            exec_state = EXEC_STATE::INIT;
            get_goal = false;
            path_ok = false;
            counter_search = 0;
        }
    }

    void GlobalPlannerUGV::goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 2D定高飞行
        goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;

        get_goal = true;

        cout << GREEN << " Global_Planner_UGV: Get a new manual goal: [" << goal_pos(0) << ", " << goal_pos(1) << ", " << goal_pos(2) << " ]" << TAIL << endl;
    }

    // 主循环
    void GlobalPlannerUGV::mainloop_cb()
    {
        static int exec_num = 0;
        exec_num++;

        switch (exec_state)
        {
        case EXEC_STATE::INIT:

            rclcpp::sleep_for(std::chrono::seconds(1));
            exec_state = EXEC_STATE::WAIT_GOAL;
            break;

        case EXEC_STATE::WAIT_GOAL:
            // 等待目标点，不执行路径追踪逻辑
            path_ok = false;

            // 等待手动输入的目标值
            if (!get_goal)
            {
                if (exec_num == 100)
                {
                    cout << YELLOW << " Global_Planner_UGV: Waiting for a new goal, subscirbe to  /global_planner/goal" << TAIL << endl;
                }
            }
            else
            {
                // 获取到目标点后，生成新轨迹
                exec_state = EXEC_STATE::PLAN;
                get_goal = false;
            }

            break;

        case EXEC_STATE::PLAN:
            // 重置规划器
            Astar_ptr->reset();
            // 使用规划器执行搜索，返回搜索结果
            astar_state = Astar_ptr->search(start_pos, goal_pos);

            // 未寻找到路径
            if (astar_state == Astar::NO_PATH)
            {
                // 找不到路径：返回等待目标点，若在自动目标点模式，则会前往下一个目标点
                if (counter_search > 50)
                {
                    path_ok = false;
                    exec_state = EXEC_STATE::WAIT_GOAL;
                    counter_search = 0;
                }
                counter_search++;
                cout << RED << " Global_Planner_UGV: Main loop Planning [ Planner can't find path ]" << TAIL << endl;
            }
            else
            {
                path_ok = true;
                counter_search = 0;
                path_cmd = Astar_ptr->get_ros_path();

                // 发布路劲用于rviz显示
                path_cmd_pub->publish(path_cmd);
            }

            break;
        }
    }

    // 【获取当前时间函数】 单位：秒
    float GlobalPlannerUGV::get_time_in_sec(const rclcpp::Time &begin_time)
    {
        rclcpp::Time time_now = this->now();      // 获取当前时间
        return (time_now - begin_time).seconds(); // 返回浮点秒
    }

    void GlobalPlannerUGV::printf_exec_state()
    {
        switch (exec_state)
        {
        case EXEC_STATE::INIT:
            cout << GREEN << " Global_Planner_UGV: Main loop Exec_state: [ INIT ]." << TAIL << endl;
            break;
        case EXEC_STATE::WAIT_GOAL:
            cout << GREEN << " Global_Planner_UGV: Main loop Exec_state: [ WAIT_GOAL ]." << TAIL << endl;
            break;
        case EXEC_STATE::PLAN:
            cout << GREEN << " Global_Planner_UGV: Main loop Exec_state: [ PLAN ]." << TAIL << endl;
            break;
        }
    }
}