#include "global_planner_ugv.h"

namespace global_planner_ugv
{
    // 初始化函数
    void GlobalPlannerUGV::init()
    {
        // 读取参数
        // 无人车编号 1号无人车则为1
        ugv_id = this->declare_parameter<int>("global_planner_ugv.ugv_id", 1);
        // A星算法 重规划频率
        replan_time = this->declare_parameter<double>("global_planner_ugv.replan_time", 1.0);
        track_frequency = this->declare_parameter<double>("global_planner_ugv.track_frequency", 0.1);
        pcd_path = this->declare_parameter<std::string>("global_planner_ugv/pcd_path", "");
        start_pos[0] = this->declare_parameter("start_pos_x", 0.0);
        start_pos[1] = this->declare_parameter("start_pos_y", 0.0);
        start_pos[2] = this->declare_parameter("start_pos_z", 0.0);
        ugv_name = "/ugv" + std::to_string(ugv_id);

        // Astar algorithm
        Astar_ptr = std::make_shared<Astar>();
        Astar_ptr->init(this->shared_from_this());

        RCLCPP_INFO(this->get_logger(), "Manual goal mode, subscribe to %s/prometheus/global_planner_ugv/goal", ugv_name.c_str());
        goal_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            ugv_name + "/prometheus/global_planner_ugv/goal",
            rclcpp::QoS(10),
            std::bind(&GlobalPlannerUGV::goal_cb, this, std::placeholders::_1));

        // 【发布】路径用于显示（rviz显示）
        path_cmd_pub = this->create_publisher<nav_msgs::msg::Path>(
            ugv_name + "/prometheus/global_planner_ugv/path_cmd",
            rclcpp::QoS(10));

        // 【定时器】主循环执行
        mainloop_timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&GlobalPlannerUGV::mainloop_cb, this));

        if (!pcd_path.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Loading static global map from PCD: %s", pcd_path.c_str());
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1)
            {
                RCLCPP_FATAL(this->get_logger(), "No PCD file path provided! Set parameter 'global_planner_ugv/pcd_path'.");
                rclcpp::shutdown();
                return;
            }
            else
            {
                // 转换为 ROS2 消息
                sensor_msgs::msg::PointCloud2 msg;
                pcl::toROSMsg(*cloud, msg);
                msg.header.frame_id = "map"; // 给个固定的坐标系
                msg.header.stamp = this->now();

                // 更新到占据地图
                Astar_ptr->Occupy_map_ptr->map_update_gpcl(std::make_shared<sensor_msgs::msg::PointCloud2>(msg));
                RCLCPP_INFO(this->get_logger(), "Global map initialized from PCD file.");
            }

            // 规划器状态参数初始化
            exec_state = EXEC_STATE::INIT;
            get_goal = false;
            path_ok = false;
            in_return_mode = false;
            rotate_in_place = false;
            start_move = false;
            get_target_pos = false;
            counter_search = 0;
            yaw_ref = 0.0;
        }
    }

    void GlobalPlannerUGV::goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 2D定高飞行
        goal_pos << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
        yaw_ref = msg->pose.orientation.w;
        goal_vel.setZero();

        get_goal = true;
        rotate_in_place = true;
        start_move = false;

        cout << GREEN << ugv_name + " Global_Planner_UGV: Get a new manual goal: [" << goal_pos(0) << ", " << goal_pos(1) << ", " << goal_pos(2) << " ]" << TAIL << endl;
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
            // 起点位置设置为返航点
            return_pos = start_pos;
            cout << GREEN << ugv_name + " Global_Planner_UGV: Return point is set as:" << return_pos(0) << ", " << return_pos(1) << ", " << return_pos(2) << TAIL << endl;
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
                    cout << YELLOW << ugv_name + " Global_Planner_UGV: Waiting for a new goal, subscirbe to " << ugv_name << "/prometheus/global_planner_ugv/goal" << TAIL << endl;
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
                cout << RED << ugv_name + " Global_Planner_UGV: Main loop Planning [ Planner can't find path ]" << TAIL << endl;
            }
            else
            {
                path_ok = true;
                counter_search = 0;
                path_cmd = Astar_ptr->get_ros_path();
                // 路径中航点数目
                Num_total_wp = path_cmd.poses.size();
                cur_id = 1;
                tra_start_time = this->now();

                // 发布路劲用于rviz显示
                path_cmd_pub->publish(path_cmd);
                // cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Planning [ Get a new path ]" << TAIL <<endl;
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
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ INIT ]." << TAIL << endl;
            break;
        case EXEC_STATE::WAIT_GOAL:
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ WAIT_GOAL ]." << TAIL << endl;
            break;
        case EXEC_STATE::PLAN:
            cout << GREEN << ugv_name + " Global_Planner_UGV: Main loop Exec_state: [ PLAN ]." << TAIL << endl;
            break;
        }
    }
}