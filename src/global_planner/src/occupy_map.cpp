#include <occupy_map.h>
#include <thread>
#include <time.h>

namespace global_planner
{
    inline double computeCost(const std::vector<float> &dist_out)
    {
        if (dist_out.empty())
            return 0.0;

        // 取最近点距离
        double d2 = *std::min_element(dist_out.begin(), dist_out.end());
        double d = sqrt(d2);

        // 避免除 0
        if (d < 1e-6)
            d = 1e-6;

        // cost 越近越大
        return 1.0 / d;
    }

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
        /*
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
                            if (y == -ifn || y == ifn)
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
                            if (y == -ifn - cost_inflate || y == ifn + cost_inflate)
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
                // printf("cost map %d %d\n", cost_inflate, cost_index);
                for (int i = 0; i < cost_index; i++)
                {
                    std::cout <<"enum_p_cost is "<< enum_p_cost[i].x() << ","<< enum_p_cost[i].y() << ","<< enum_p_cost[i].z()<<std::endl;
                }
                // std::cout << std::endl;

                std::cout << "cost_index is " << cost_index << std::endl;
                std::cout << "inflate_index is " << inflate_index << std::endl;
                */
    }

    // 地图更新函数 - 输入：全局点云
    void Occupy_map::map_update_gpcl(const pcl::PointCloud<pcl::PointXYZ>::Ptr global_point)
    {
        global_point_cloud_map = global_point;
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*global_point_cloud_map, min_pt, max_pt);

        // map 尺寸
        origin_(0) = min_pt.x() - ifn;
        origin_(1) = min_pt.y() - ifn;
        origin_(2) = min_pt.z() - ifn;
        map_size_3d_(0) = max_pt.x() - min_pt.x();
        map_size_3d_(1) = max_pt.y() - min_pt.y();
        map_size_3d_(2) = max_pt.z() - min_pt.z();

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
    }

    // 当global_planning节点接收到点云消息更新时，进行设置点云指针并膨胀
    // Astar规划路径时，采用的是此处膨胀后的点云（setOccupancy只在本函数中使用）
    // 路径规划的时候是用的occupancy_buffer_

    void Occupy_map::inflate_point_cloud()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud = *global_point_cloud_map;
        if (cloud.empty())
            return;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(global_point_cloud_map);

        size_t t0 = clock();

        int Nx = grid_size_(0), Ny = grid_size_(1), Nz = grid_size_(2);

        double search_radius = ifn;   // 你膨胀的范围（例如 0.5m）
        double cost_radius = 2 * ifn; // cost map 范围

        std::vector<uint8_t> occ_mark(Nx * Ny * Nz, 0);
        std::vector<uint8_t> visited(Nx * Ny * Nz, 0);
        std::vector<double> cost_mark(Nx * Ny * Nz, 0.0);

        // 全局或局部 static 原子计数器
        std::atomic<int> progress_counter(0);
        int total_voxels = Nx * Ny * Nz;
#pragma omp parallel for schedule(static)
        for (int ix = 0; ix < Nx; ix++)
        {
            for (int iy = 0; iy < Ny; iy++)
            {
                for (int iz = 0; iz < Nz; iz++)
                {
                    int idx = ix * Ny * Nz + iy * Nz + iz;
                    if (visited[idx])
                        continue;
                    Eigen::Vector3i id(ix, iy, iz);
                    Eigen::Vector3d pos;
                    indexToPos(id, pos);
                    pcl::PointXYZ searchPoint(pos.x(), pos.y(), pos.z());

                    // -------- KD-tree 查询 ----------
                    std::vector<int> idx_out;
                    std::vector<float> dist_out;

                    if (kdtree.radiusSearch(searchPoint, search_radius, idx_out, dist_out) > 0)
                    {
                        visited[idx] = 1;
                        occ_mark[idx] = 1;
                    }

                    // cost 查询
                    if (kdtree.radiusSearch(searchPoint, cost_radius, idx_out, dist_out) > 0)
                    {
                        // visited[idx] = 1;
                        cost_mark[idx] = computeCost(dist_out);
                    }

                    // ----------- 进度打印部分 -----------
                    int finished = ++progress_counter;
                    if (finished % 50000 == 0) // 每 5 万个格子打印一次
                    {
                        double percent = 100.0 * finished / total_voxels;
                        log("\r[Occupy_map] Inflate Progress: %.2f%% (%d / %d)", percent, finished, total_voxels);
                        fflush(stdout);
                    }
                }
            }
        }
        std::cout << "map over!" << std::endl;
#pragma omp parallel for
        for (int i = 0; i < Nx * Ny * Nz; i++)
        {
            if (occ_mark[i])
                occupancy_buffer_[i] = 1;

            if (cost_mark[i] > 0)
                cost_map_[i] = cost_mark[i];
        }

        double t = (clock() - t0) * 1.0 / CLOCKS_PER_SEC;
        log("[Occupy_map] Inflation optimized in " + std::to_string(t) + " seconds.");
    }

    void Occupy_map::setOccupancy(Eigen::Vector3d &pos, int occ)
    {
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
