#include "localizer.h"
#include "localizer_common.h"
#include <thread>

namespace localizer
{
    Localizer::Localizer(const std::string &config_path)
    {
        // 初始化成员变量
        kf = std::make_shared<IESKF>();
        m_builder_status = BuilderStatus::IMU_INIT;
        imu_processor = std::make_shared<IMUProcessor>(m_config, kf);
        lidar_processor = std::make_shared<LidarProcessor>(m_config, kf);
        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);
        running_ = false;

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            log("FAIL TO LOAD YAML FILE!");
            return;
        }
        m_config.update_hz = config["update_hz"].as<double>();
        m_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_config.scan_resolution = config["scan_resolution"].as<double>();
        m_config.map_resolution = config["map_resolution"].as<double>();
        m_config.cube_len = config["cube_len"].as<double>();
        m_config.det_range = config["det_range"].as<double>();
        m_config.move_thresh = config["move_thresh"].as<double>();
        m_config.na = config["na"].as<double>();
        m_config.ng = config["ng"].as<double>();
        m_config.nba = config["nba"].as<double>();
        m_config.nbg = config["nbg"].as<double>();
        m_config.imu_init_num = config["imu_init_num"].as<int>();
        m_config.near_search_num = config["near_search_num"].as<int>();
        m_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_config.gravity_align = config["gravity_align"].as<bool>();
        m_config.esti_il = config["esti_il"].as<bool>();

        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();

        m_localizer_config.rough_scan_resolution = config["rough_scan_resolution"].as<double>();
        m_localizer_config.rough_map_resolution = config["rough_map_resolution"].as<double>();
        m_localizer_config.rough_max_iteration = config["rough_max_iteration"].as<int>();
        m_localizer_config.rough_score_thresh = config["rough_score_thresh"].as<double>();

        m_localizer_config.refine_scan_resolution = config["refine_scan_resolution"].as<double>();
        m_localizer_config.refine_map_resolution = config["refine_map_resolution"].as<double>();
        m_localizer_config.refine_max_iteration = config["refine_max_iteration"].as<int>();
        m_localizer_config.refine_score_thresh = config["refine_score_thresh"].as<double>();

        // 启动定位线程
        startAsync();
    }

    Localizer::~Localizer()
    {
        stopAsync();
    }

    void Localizer::feedIMU(const IMUData &imu)
    {
        // TODO: 存入 IMU 队列
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = imu.time;
        if (timestamp < m_state_data.last_imu_time)
        {
            log("IMU timestamp error!\n");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(imu);
        m_state_data.last_imu_time = timestamp;
    }

    void Localizer::feedLidar(const PointCloud &cloud)
    {
        // TODO: 存入点云队列
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        if (cloud.timestamp < m_state_data.last_lidar_time)
        {
            log("Lidar Message is out of order!\n");
            std::deque<std::pair<double, CloudType::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(cloud.timestamp, cloud.points);
        m_state_data.last_lidar_time = cloud.timestamp;
    }

    void Localizer::setPose(const Pose_loc &pose)
    {
        // TODO: 强制设置位姿
        float x = pose.x;
        float y = pose.y;
        float z = pose.z;
        float yaw = pose.yaw;
        float roll = pose.roll;
        float pitch = pose.pitch;
        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
        {
            std::lock_guard<std::mutex> lock(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
        }
        return;
    }

    bool Localizer::setPointCloud(const std::string &pcd_path)
    {
        if (!pcd_path.empty())
        {
            bool load_flag = m_localizer->loadMap(pcd_path);
            if (!load_flag)
            {
                log("Failed to load PCD: " + pcd_path + "\n");
                return false;
            }
            log("Global map initialized from PCD: " + pcd_path + "\n");
            return true;
        }
        return false;
    }

    void Localizer::setPoseCallback(PoseCallback cb)
    {
        pose_callback_ = cb;
    }

    void Localizer::setLogCallback(LogCallback cb)
    {
        log_callback_ = cb;
    }

    bool Localizer::syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            // std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
            //           { return p1.curvature < p2.curvature; });
            m_package.cloud_end_time = m_state_data.lidar_buffer.front().first;
            // m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;

        return true;
    }

    void Localizer::localize()
    {
        if (!syncPackage())
            return;

        if (m_builder_status == BuilderStatus::IMU_INIT)
        {
            if (imu_processor->initialize(m_package))
                m_builder_status = BuilderStatus::MAP_INIT;
            return;
        }
        imu_processor->undistort(m_package);
        if (m_builder_status == BuilderStatus::MAP_INIT)
        {
            CloudType::Ptr cloud_world = LidarProcessor::transformCloud(m_package.cloud, lidar_processor->r_wl(), lidar_processor->t_wl());
            lidar_processor->initCloudMap(cloud_world->points);
            m_state.message_lidar_received = true;
            m_builder_status = BuilderStatus::MAPPING;
            return;
        }
        lidar_processor->process(m_package);
        m_state.last_message_time = m_package.cloud_end_time;
        if (!m_state.message_lidar_received)
            return;

        Eigen::Quaterniond q(kf->x().r_wi);
        m_state.last_r = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z()).toRotationMatrix();
        m_state.last_t = V3D(kf->x().t_wi.x(), kf->x().t_wi.y(), kf->x().t_wi.z());
        CloudType::Ptr body_cloud = lidar_processor->transformCloud(m_package.cloud, kf->x().r_il, kf->x().t_il);
        /*--------------------------------------------------------------------------------------
        ----------------------------------- Start Localize -------------------------------------
        --------------------------------------------------------------------------------------*/
        uint64_t diff = now_ns() - m_state.last_send_time;
        bool update = diff > (1.0 / m_config.update_hz) && m_state.message_lidar_received;
        if (!update)
        {
            return;
        }
        m_state.last_send_time = now_ns();
        M4F initial_guess = M4F::Identity();

        // initial_guess第一次赋值是人为设置好的
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
        }
        else
        {
            // 猜测的初始位姿，非常不准确
            std::lock_guard<std::mutex>(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        uint64_t current_time;
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            current_local_r = m_state.last_r;
            current_local_t = m_state.last_t;
            current_time = m_state.last_message_time;
            m_localizer->setInput(body_cloud);
        }

        bool result = m_localizer->align(initial_guess);
        if (result)
        {
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            if (m_state.service_received)
            {
                std::lock_guard<std::mutex>(m_state.service_mutex);
                m_state.service_received = false;
            }
        }
        if (pose_callback_)
        {
            Eigen::Quaterniond q(m_state.last_offset_r);
            V3D t = m_state.last_offset_t;
            PoseStamped pose_stamped;
            pose_stamped.timestamp = current_time;
            pose_stamped.t= t;
            pose_stamped.q = q;
            pose_callback_(pose_stamped);
        }
    }

    void Localizer::startAsync()
    {
        if (running_)
        {
            log("Async thread already running.");
            return;
        }

        running_ = true;

        worker_thread_ = std::thread([this]()
                                     {

        using namespace std::chrono_literals;
        log("Localizer async thread started.");

        while (running_)
        {
           localize();
           std::this_thread::sleep_for(10ms); // 控制频率（你可按需求调整）
        }

        log("Localizer async thread exiting..."); });
    }

    void Localizer::stopAsync()
    {
        if (!running_)
            return;

        running_ = false;

        if (worker_thread_.joinable())
            worker_thread_.join();

        log("Localizer async thread stopped.");
    }

    void Localizer::log(const std::string &msg)
    {
        if (log_callback_)
            log_callback_(msg);
    }
}