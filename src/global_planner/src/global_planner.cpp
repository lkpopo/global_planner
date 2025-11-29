#include "global_planner.h"
#include "planer_common.h"


namespace global_planner
{
    static constexpr double WGS84_A = 6378137.0;                // 长半轴
    static constexpr double WGS84_F = 1.0 / 298.257223563;      // 扁率
    static constexpr double WGS84_E2 = WGS84_F * (2 - WGS84_F); // 偏心率平方
    static constexpr double UTM_K0 = 0.9996;

    int lonToUTMZone(double lon)
    {
        return int((lon + 180.0) / 6.0) + 1;
    }

    GlobalPlannerUGV::GlobalPlannerUGV()
    {
        Astar_ptr = std::make_shared<Astar>();
        
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        set_start_ = false;
        set_goal_ = false;
        path_callback_ = nullptr;
        log_callback_ = nullptr;
        utm_zone_ = 50;
    }

    GlobalPlannerUGV::~GlobalPlannerUGV()
    {
        // stop();
    }

    bool GlobalPlannerUGV::initFromConfig(const std::string &config_path)
    {
        log("[GlobalPlannerUGV] Astart init " + config_path + "\n");
        bool res = Astar_ptr->init(config_path);

        return res;
    }

    bool GlobalPlannerUGV::setPointCloud(const std::string &pcd_path)
    {
        cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pcd_path_ = pcd_path;
        if (pcl::io::loadPCDFile(pcd_path, *cloud_) != 0)
        {
            log("Failed to load PCD: " + pcd_path + "\n");
            return false;
        }
        else
        {
            log("[GlobalPlannerUGV] PCD loaded, point size: " + std::to_string(cloud_->points.size()) + "\n");
            Astar_ptr->Occupy_map_ptr->map_update_gpcl(cloud_);
            log("[GlobalPlannerUGV] Occupancy map updated from point cloud.\n");
            return true;
        }
    }

    void GlobalPlannerUGV::setPathCallback(PathCallback cb)
    {
        path_callback_ = cb;
    }

    void GlobalPlannerUGV::setLogCallback(LogCallback cb)
    {
        log_callback_ = cb;
    }

    void GlobalPlannerUGV::setStart(const Eigen::Vector3d &s)
    {
        log("[GlobalPlannerUGV] Setting start position...\n");

        start_pos_ = gpsToUtm(s.x(), s.y(), s.z());
        set_start_ = true;
        log("[GlobalPlannerUGV] Start position set: " + vec3ToString(start_pos_) + "\n");
    }

    void GlobalPlannerUGV::setGoal(const Eigen::Vector3d &g)
    {
        goal_pos_ = gpsToUtm(g.x(), g.y(), g.z());
        set_goal_ = true;
        log("[GlobalPlannerUGV] Goal position set: " + vec3ToString(goal_pos_) + "\n");
    }

    Path GlobalPlannerUGV::planSyncInternal()
    {
        Astar_ptr->reset();
        bool res = Astar_ptr->search(start_pos_, goal_pos_);
        if (res)
        {
            path_result_ = Astar_ptr->getPath();
            return path_result_;
        }
        return Path();
    }

    void GlobalPlannerUGV::planAsync()
    {
        if (!set_goal_ || !set_start_)
        {
            log("Goal_pos or start_pos hasn't set!\n");
            return;
        }

        log("[GlobalPlannerUGV] Starting async planning...\n");
         std::thread([this]()
                    {
            Path path_result  = this->planSyncInternal();
            // 规划成功返回路径，否则路径是空的
            if(path_callback_)
            {
                path_callback_(path_result);
            } })
            .detach();
    }

    void GlobalPlannerUGV::log(const std::string &msg)
    {
        if (log_callback_)
            log_callback_(msg);
    }

    std::string GlobalPlannerUGV::vec3ToString(const Eigen::Vector3d &v)
    {
        return std::to_string(v.x()) + "," + std::to_string(v.y()) + "," + std::to_string(v.z());
    }

    std::vector<GpsPoint> GlobalPlannerUGV::convertPathToGPS(const Path &path) const
    {
        std::vector<GpsPoint> gps_list;
        gps_list.reserve(path.poses.size());

        int zone = utm_zone_;

        for (const auto &p : path.poses)
        {
            double x = p.position.x() - 500000.0;
            double y = p.position.y();

            double M = y / UTM_K0;
            double mu = M / (WGS84_A * (1 - WGS84_E2 / 4 - 3 * WGS84_E2 * WGS84_E2 / 64 - 5 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 256));

            double e1 = (1 - sqrt(1 - WGS84_E2)) / (1 + sqrt(1 - WGS84_E2));

            double J1 = (3 * e1 / 2 - 27 * pow(e1, 3) / 32);
            double J2 = (21 * e1 * e1 / 16 - 55 * pow(e1, 4) / 32);
            double J3 = (151 * pow(e1, 3) / 96);
            double J4 = (1097 * pow(e1, 4) / 512);

            double fp = mu + J1 * sin(2 * mu) + J2 * sin(4 * mu) + J3 * sin(6 * mu) + J4 * sin(8 * mu);

            double C1 = WGS84_E2 / (1 - WGS84_E2) * cos(fp) * cos(fp);
            double T1 = tan(fp) * tan(fp);
            double N1 = WGS84_A / sqrt(1 - WGS84_E2 * sin(fp) * sin(fp));
            double R1 = N1 * (1 - WGS84_E2) / (1 - WGS84_E2 * sin(fp) * sin(fp));
            double D = x / (N1 * UTM_K0);

            double lat = fp - (N1 * tan(fp) / R1) *
                                  (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * WGS84_E2) * pow(D, 4) / 24 + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * WGS84_E2 - 3 * C1 * C1) * pow(D, 6) / 720);

            double lon0 = (zone - 1) * 6 - 180 + 3; // 中央经线
            double lon = (D - (1 + 2 * T1 + C1) * pow(D, 3) / 6 +
                          (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * WGS84_E2 + 24 * T1 * T1) * pow(D, 5) / 120) /
                         cos(fp);

            lon = lon0 + lon * 180 / M_PI;
            lat = lat * 180 / M_PI;

            gps_list.push_back({lat, lon, p.position.z()});
        }

        return gps_list;
    }

    // 纬度-经度-海拔
    Eigen::Vector3d GlobalPlannerUGV::gpsToUtm(double lat, double lon, double alt)
    {
        // 度 → 弧度
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        // 确定 UTM  Zone
        int zone = lonToUTMZone(lon);
        utm_zone_ = zone;

        double lon0 = (zone - 1) * 6 - 180 + 3; // 中央经线
        double lon0_rad = lon0 * M_PI / 180.0;

        double N = WGS84_A / sqrt(1 - WGS84_E2 * sin(lat_rad) * sin(lat_rad));
        double T = tan(lat_rad) * tan(lat_rad);
        double C = WGS84_E2 / (1 - WGS84_E2) * cos(lat_rad) * cos(lat_rad);
        double A = cos(lat_rad) * (lon_rad - lon0_rad);

        double M = WGS84_A * ((1 - WGS84_E2 / 4 - 3 * WGS84_E2 * WGS84_E2 / 64 - 5 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 256) * lat_rad - (3 * WGS84_E2 / 8 + 3 * WGS84_E2 * WGS84_E2 / 32 + 45 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 1024) * sin(2 * lat_rad) + (15 * WGS84_E2 * WGS84_E2 / 256 + 45 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 1024) * sin(4 * lat_rad) - (35 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 3072) * sin(6 * lat_rad));

        double x = UTM_K0 * N * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * WGS84_E2) * pow(A, 5) / 120) + 500000.0;

        double y = UTM_K0 * (M + N * tan(lat_rad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + (61 - 58 * T + T * T + 600 * C - 330 * WGS84_E2) * pow(A, 6) / 720));

        if (lat < 0)
            y += 10000000.0;

        return Eigen::Vector3d(x, y, alt);
    }

} // namespace global_planner
