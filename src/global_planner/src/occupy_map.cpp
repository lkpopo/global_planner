#include <occupy_map.h>

namespace global_planner
{
    // 初始化函数
    void Occupy_map::init(rclcpp::Node::SharedPtr node)
    {
        node_ = node;
        // ---------------- 参数读取 ----------------
        cost_inflate = node->declare_parameter<int>("global_planner/cost_inflate", 5);

        origin_(0) = node->declare_parameter<double>("map/origin_x", -5.0);
        origin_(1) = node->declare_parameter<double>("map/origin_y", -5.0);
        origin_(2) = node->declare_parameter<double>("map/origin_z", -0.5);

        map_size_3d_(0) = node->declare_parameter<double>("map/map_size_x", 100.0);
        map_size_3d_(1) = node->declare_parameter<double>("map/map_size_y", 100.0);
        map_size_3d_(2) = node->declare_parameter<double>("map/map_size_z", 20.0);

        inflate_ = node->declare_parameter<double>("map/inflate", 0.3);

        node->get_parameter<double>("map/resolution", resolution_);

        // 发布 地图rviz显示
        global_pcl_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/global_planner/global_pcl", 10);
        // 发布膨胀后的点云
        inflate_pcl_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/global_planner/global_inflate_pcl", 10);

        // 主循环执行定时器
        pcl_pub_timer_ = node->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&Occupy_map::pub_pcl_cb, this));

        // 全局地图点云指针
        global_point_cloud_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
        // 膨胀点云指针
        cloud_inflate_vis_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        // 传入点云指针（临时指针）
        input_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        this->inv_resolution_ = 1.0 / resolution_;
        for (int i = 0; i < 3; ++i)
        {
            // 占据图尺寸 = 地图尺寸 / 分辨率
            grid_size_(i) = ceil(map_size_3d_(i) / resolution_);
        }

        // 占据容器的大小 = 占据图尺寸 x*y*z
        occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
        cost_map_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
        fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);
        fill(cost_map_.begin(), cost_map_.end(), 0.0);

        min_range_ = origin_;
        max_range_ = origin_ + map_size_3d_;

        get_gpcl = false;
        // 生成地图边界：点云形式
        double dist = 0.1;                                      // 每多少距离一个点
        int numdist_x = (max_range_(0) - min_range_(0)) / dist; // x的点数
        int numdist_y = (max_range_(1) - min_range_(1)) / dist; // y的点数
        int numdist = 2 * (numdist_x + numdist_y);              // 总点数

        // 膨胀格子数 = 膨胀距离/分辨率
        // ceil返回大于或者等于指定表达式的最小整数
        ifn = ceil(inflate_ * inv_resolution_);

        inflate_index = 0;
        // 三维关键点膨胀
        for (int x = -ifn; x <= ifn; x++)
        {
            for (int y = -ifn; y <= ifn;)
            {
                for (int z = -ifn; z <= ifn;)
                {
                    // 只取关键点，不全填充
                    enum_p[inflate_index++] << x * resolution_, y * resolution_, z * resolution_;

                    // Z方向跳跃策略
                    if (z == -ifn || z == ifn)
                        z++;
                    else
                        z += 2 * ifn;
                }
                // Y方向跳跃策略
                if (x == -ifn || x == ifn)
                    y++;
                else
                    y += 2 * ifn;
            }
        }

        cost_index = 0;

        for (int x = -ifn - cost_inflate; x <= ifn + cost_inflate; x++)
        {
            for (int y = -ifn - cost_inflate; y <= ifn + cost_inflate;)
            {
                for (int z = -ifn - cost_inflate; z <= ifn + cost_inflate;)
                {
                    int tmp_dis = x * x + y * y + z * z;
                    if (tmp_dis <= (ifn + cost_inflate) * (ifn + cost_inflate))
                    {
                        enum_p_cost[cost_index++] << x * resolution_, y * resolution_, z * resolution_;
                    }

                    // Z方向跳跃策略
                    if (z == -ifn - cost_inflate || z == ifn + cost_inflate)
                        z++;
                    else
                        z += 2 * ifn + 2 * cost_inflate;
                }

                // Y方向跳跃策略
                if (x == -ifn - cost_inflate || x == ifn + cost_inflate)
                    y++;
                else
                    y += 2 * ifn + 2 * cost_inflate;
            }
        }
        printf("cost map %d %d\n", cost_inflate, cost_index);
    }

    // 地图更新函数 - 输入：全局点云
    void Occupy_map::map_update_gpcl(const std::shared_ptr<const sensor_msgs::msg::PointCloud2> &global_point)
    {
        // 全局地图只更新一次
        if (get_gpcl)
        {
            return;
        }

        get_gpcl = true;
        has_global_point = true;
        pcl::fromROSMsg(*global_point, *input_point_cloud);
        global_point_cloud_map = input_point_cloud;
        inflate_point_cloud();
    }

    // 当global_planning节点接收到点云消息更新时，进行设置点云指针并膨胀
    // Astar规划路径时，采用的是此处膨胀后的点云（setOccupancy只在本函数中使用）
    // 路径规划的时候是用的occupancy_buffer_，cloud_inflate_vis_仅用作显示点云数据查看
    void Occupy_map::inflate_point_cloud(void)
    {

        // cout << BLUE << "22222 " << TAIL <<endl;

        if (!has_global_point)
        {
            return;
        }

        if (get_gpcl)
        {
            // occupancy_buffer_清零，不需要清0
            fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);
            fill(cost_map_.begin(), cost_map_.end(), 0.0);
        }

        // 记录开始时间
        rclcpp::Time time_start = node_->now();

        // 转化为PCL的格式进行处理
        pcl::PointCloud<pcl::PointXYZ> latest_global_cloud_ = *global_point_cloud_map;

        if ((int)latest_global_cloud_.points.size() == 0)
        {
            return;
        }

        cloud_inflate_vis_->clear();

        pcl::PointXYZ pt_inf;
        Eigen::Vector3d p3d, p3d_inf, p3d_cost;

        // 遍历全局点云中的所有点
        for (size_t i = 0; i < latest_global_cloud_.points.size(); ++i)
        {
            // 取出第i个点
            p3d(0) = latest_global_cloud_.points[i].x;
            p3d(1) = latest_global_cloud_.points[i].y;
            p3d(2) = latest_global_cloud_.points[i].z;

            // 若取出的点不在地图内（膨胀时只考虑地图范围内的点）
            if (!isInMap(p3d))
            {
                continue;
            }

            // cost map update
            for (int j = 0; j < cost_index; j++)
            {
                p3d_cost(0) = p3d(0) + enum_p_cost[j](0);
                p3d_cost(1) = p3d(1) + enum_p_cost[j](1);
                p3d_cost(2) = p3d(2);
                this->updateCostMap(p3d_cost, 1.0 / enum_p_cost[j](2));
            }

            // 根据膨胀距离，膨胀该点
            for (int i = 0; i < inflate_index; i++)
            {
                p3d_inf(0) = p3d(0) + enum_p[i](0);
                p3d_inf(1) = p3d(1) + enum_p[i](1);
                p3d_inf(2) = p3d(2) + enum_p[i](2);

                // 若膨胀的点不在地图内（膨胀时只考虑地图范围内的点）
                if (!isInMap(p3d_inf))
                {
                    continue;
                }

                pt_inf.x = p3d_inf(0);
                pt_inf.y = p3d_inf(1);
                pt_inf.z = p3d_inf(2);
                cloud_inflate_vis_->push_back(pt_inf);
                // 设置膨胀后的点被占据（不管他之前是否被占据）
                this->setOccupancy(p3d_inf, 1);
            }
        }

        static int exec_num = 0;
        exec_num++;

        // 此处改为根据循环时间计算的数值
        if (exec_num == 50)
        {
            // 膨胀地图效率与地图大小有关
            cout << YELLOW << "Occupy map: inflate global point take " << (node_->now() - time_start).seconds() << " [s]. " << TAIL << endl;
            exec_num = 0;
        }
        else if (get_gpcl)
        {
            // cout << YELLOW << "Occupy map: inflate global point take " << (ros::Time::now()-time_start).toSec() <<" [s]. " << TAIL <<endl;
        }
    }

    void Occupy_map::pub_pcl_cb()
    {
        // 发布未膨胀点云
        sensor_msgs::msg::PointCloud2 global_env_;
        // 假设此时收到的就是全局pcl
        pcl::toROSMsg(*global_point_cloud_map, global_env_);
        global_env_.header.frame_id = "map";
        global_pcl_pub_->publish(global_env_);

        // 发布膨胀点云
        sensor_msgs::msg::PointCloud2 map_inflate_vis;
        pcl::toROSMsg(*cloud_inflate_vis_, map_inflate_vis);
        map_inflate_vis.header.frame_id = "map";
        inflate_pcl_pub_->publish(map_inflate_vis);
    }

    void Occupy_map::setOccupancy(Eigen::Vector3d &pos, int occ)
    {
        if (occ != 1 && occ != 0)
        {
            // cout << RED << "Occupy map: occ value error " << TAIL <<endl;
            return;
        }

        if (!isInMap(pos))
        {
            return;
        }

        Eigen::Vector3i id;
        posToIndex(pos, id);

        // 设置为占据/不占据 索引是如何索引的？ [三维地图 变 二维数组]
        // 假设10*10*10米，分辨率1米，buffer大小为 1000 （即每一个占格都对应一个buffer索引）
        // [0.1,0.1,0.1] 对应索引为[0,0,0]， buffer索引为 0
        // [9.9,9.9,9.9] 对应索引为[9,9,9]， buffer索引为 900+90+9 = 999
        occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)] = occ;
    }

    void Occupy_map::updateCostMap(Eigen::Vector3d &pos, double cost)
    {
        if (!isInMap(pos))
        {
            return;
        }

        Eigen::Vector3i id;
        posToIndex(pos, id);

        if (cost_map_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)] >= cost)
            return;
        cost_map_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)] = cost;
    }

    bool Occupy_map::isInMap(Eigen::Vector3d pos)
    {
        // min_range就是原点，max_range就是原点+地图尺寸
        // 比如设置0,0,0为原点，[0,0,0]点会被判断为不在地图里
        // 　同时　对于２Ｄ情况，超出飞行高度的数据也会认为不在地图内部
        if (pos(0) < min_range_(0) + 1e-4 || pos(1) < min_range_(1) + 1e-4 || pos(2) < min_range_(2) + 1e-4)
        {
            return false;
        }

        if (pos(0) > max_range_(0) - 1e-4 || pos(1) > max_range_(1) - 1e-4 || pos(2) > max_range_(2) - 1e-4)
        {
            return false;
        }

        return true;
    }

    void Occupy_map::posToIndex(Eigen::Vector3d &pos, Eigen::Vector3i &id)
    {
        for (int i = 0; i < 3; ++i)
        {
            id(i) = floor((pos(i) - origin_(i)) * inv_resolution_);
        }
    }

    void Occupy_map::indexToPos(Eigen::Vector3i &id, Eigen::Vector3d &pos)
    {
        for (int i = 0; i < 3; ++i)
        {
            pos(i) = (id(i) + 0.5) * resolution_ + origin_(i);
        }
    }

    int Occupy_map::getOccupancy(Eigen::Vector3d &pos)
    {
        if (!isInMap(pos))
        {
            return -1;
        }

        Eigen::Vector3i id;
        posToIndex(pos, id);

        return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
    }

    double Occupy_map::getCost(Eigen::Vector3d &pos)
    {
        if (!isInMap(pos))
        {
            return -1;
        }

        Eigen::Vector3i id;
        posToIndex(pos, id);

        return cost_map_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
    }

    int Occupy_map::getOccupancy(Eigen::Vector3i &id)
    {
        if (id(0) < 0 || id(0) >= grid_size_(0) || id(1) < 0 || id(1) >= grid_size_(1) || id(2) < 0 ||
            id(2) >= grid_size_(2))
        {
            return -1;
        }

        return occupancy_buffer_[id(0) * grid_size_(1) * grid_size_(2) + id(1) * grid_size_(2) + id(2)];
    }
}
