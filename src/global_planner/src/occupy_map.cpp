#include <occupy_map.h>
#include <thread>
#include <omp.h>

namespace global_planner
{
    // 初始化函数
    void Occupy_map::init()
    {
        // 全局地图点云指针
        global_point_cloud_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
        // inflate_=0.5;
        // cost_inflate=5;
        inv_resolution_ = 1.0 / resolution_;

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
    void Occupy_map::map_update_gpcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr global_point)
    {
        global_point_cloud_map = global_point;
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*global_point_cloud_map, min_pt, max_pt);

        // double margin = 1000.0+ifn;
        // Eigen::Vector3d min3d(min_pt.x(), min_pt.y(), min_pt.z());
        // Eigen::Vector3d max3d(max_pt.x(), max_pt.y(), max_pt.z());
        // map 尺寸
        origin_(0) = min_pt.x() - ifn;
        origin_(1) = min_pt.y() - ifn;
        origin_(2) = min_pt.z() - ifn;
        map_size_3d_(0) = max_pt.x() - min_pt.x();
        map_size_3d_(1) = max_pt.y() - min_pt.y();
        map_size_3d_(2) = max_pt.z() - min_pt.z();
        // origin_ = min3d - Eigen::Vector3d(margin, margin, margin);
        // map_size_3d_ = (max3d - min3d) + 2.0 * Eigen::Vector3d(margin, margin, margin);

        std::cout << "origin: " << origin_.x() << " " << origin_.y() << " " << origin_.z() << std::endl;
        std::cout << "map size: " << map_size_3d_.x() << " " << map_size_3d_.y() << " " << map_size_3d_.z() << std::endl;

        for (int i = 0; i < 3; ++i)
        {
            // 占据图尺寸 = 地图尺寸 / 分辨率
            grid_size_(i) = ceil(ifn * map_size_3d_(i) / resolution_);
            std::cout << "grid size " << i << " is " << grid_size_(i) << std::endl;
        }

        // 占据容器的大小 = 占据图尺寸 x*y*z
        occupancy_buffer_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
        cost_map_.resize(grid_size_(0) * grid_size_(1) * grid_size_(2));
        fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0.0);
        fill(cost_map_.begin(), cost_map_.end(), 0.0);

        min_range_ = origin_;
        max_range_ = origin_ + map_size_3d_;

        std::cout << "[Occupy_map] Map updated. Size: "
                  << map_size_3d_.x() << " x " << map_size_3d_.y() << " x " << map_size_3d_.z()
                  << ", Grid size: " << grid_size_.x() << " x " << grid_size_.y() << " x " << grid_size_.z() << std::endl;
        inflate_point_cloud();

        double margin = 500.0;
        // min_range_ = origin_ - Eigen::Vector3d(margin, margin, 0);
        // max_range_ = origin_ + map_size_3d_ + Eigen::Vector3d(margin, margin, 0);
    }

    // 当global_planning节点接收到点云消息更新时，进行设置点云指针并膨胀
    // Astar规划路径时，采用的是此处膨胀后的点云（setOccupancy只在本函数中使用）
    // 路径规划的时候是用的occupancy_buffer_
    void Occupy_map::inflate_point_cloud(void)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud = *global_point_cloud_map;
        if (cloud.empty())
            return;

        // 开线程数量（也可不设，让OMP自动选择）
        omp_set_num_threads(std::thread::hardware_concurrency());

#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < cloud.size(); ++i)
        {
            Eigen::Vector3d p3d, p3d_inf, p3d_cost;

            p3d << cloud[i].x, cloud[i].y, cloud[i].z;

            if (!isInMap(p3d))
                continue;

            // 更新 cost map
            for (int j = 0; j < cost_index; j++)
            {
                p3d_cost.x() = p3d.x() + enum_p_cost[j][0];
                p3d_cost.y() = p3d.y() + enum_p_cost[j][1];
                p3d_cost.z() = p3d.z();

                updateCostMap(p3d_cost, 1.0 / enum_p_cost[j][2]);
            }

            // 膨胀
            for (int k = 0; k < inflate_index; k++)
            {
                p3d_inf = p3d + enum_p[k];

                if (!isInMap(p3d_inf))
                    continue;

                setOccupancy(p3d_inf, 1);
            }
        }

        std::cout << "[Occupy_map] Inflation completed (OpenMP)." << std::endl;
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

    void Occupy_map::log(const std::string &msg)
    {
        if (log_cb_)
        {
            log_cb_(msg);
        }
    }
}
