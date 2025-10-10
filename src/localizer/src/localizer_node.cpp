#include <queue>
#include <vector>
#include <mutex>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "localizers/commons.h"
#include "localizers/icp_localizer.h"
#include "localizers/imu_processor.h"
#include "localizers/lidar_processor.h"
#include "interface/srv/relocalize.hpp"
#include "interface/srv/is_valid.hpp"
#include <yaml-cpp/yaml.h>

#include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"

using namespace std::chrono_literals;

struct NodeState
{
    std::mutex message_mutex;
    std::mutex service_mutex;

    bool message_imu_received = false;
    bool message_lidar_received = false;
    bool service_received = false;
    bool localize_success = false;
    rclcpp::Time last_send_tf_time = rclcpp::Clock().now();
    builtin_interfaces::msg::Time last_message_time;
    M3D last_r;                          // localmap_body_r
    V3D last_t;                          // localmap_body_t
    M3D last_offset_r = M3D::Identity(); // map_localmap_r
    V3D last_offset_t = V3D::Zero();     // map_localmap_t
    M4F initial_guess = M4F::Identity();
};

class LocalizerNode : public rclcpp::Node
{
public:
    // 初始化构造函数
    LocalizerNode() : Node("localizer_node")
    {
        RCLCPP_INFO(this->get_logger(), "Localizer Node Started");
        loadParameters();
        
        kf = std::make_shared<IESKF>();
        m_builder_status=BuilderStatus::IMU_INIT;
        imu_processor = std::make_shared<IMUProcessor>(m_config, kf);
        lidar_processor = std::make_shared<LidarProcessor>(m_config, kf);

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_config.odom_topic, 10, std::bind(&LocalizerNode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(m_config.cloud_topic, 10, std::bind(&LocalizerNode::lidarCB, this, std::placeholders::_1));

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_localizer = std::make_shared<ICPLocalizer>(m_localizer_config);

        m_reloc_srv = this->create_service<interface::srv::Relocalize>("relocalize", std::bind(&LocalizerNode::relocCB, this, std::placeholders::_1, std::placeholders::_2));

        m_reloc_check_srv = this->create_service<interface::srv::IsValid>("relocalize_check", std::bind(&LocalizerNode::relocCheckCB, this, std::placeholders::_1, std::placeholders::_2));

        m_map_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map_cloud", 10);
        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10);

        m_timer = this->create_wall_timer(20ms, std::bind(&LocalizerNode::timerCB, this));

        if (!pcd_path.empty())
        {
            RCLCPP_INFO(this->get_logger(), "Loading static global map from PCD: %s", pcd_path.c_str());
            bool load_flag = m_localizer->loadMap(pcd_path);
            if (!load_flag)
            {
                RCLCPP_FATAL(this->get_logger(), "No PCD file path provided! Set parameter 'pcd_path'.");
                rclcpp::shutdown();
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Global map initialized from PCD file.");
            // publishMapCloud(rclcpp::Clock().now());
        }
        else
        {
            RCLCPP_FATAL(this->get_logger(), "No PCD file path provided! Set parameter 'pcd_path'.");
            rclcpp::shutdown();
            return;
        }
    }

    // 从yaml文件中加载参数
    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);
        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        pcd_path = config["pcd_path"].as<std::string>();

        m_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_config.odom_topic = config["odom_topic"].as<std::string>();
        m_config.map_frame = config["map_frame"].as<std::string>();
        m_config.local_frame = config["local_frame"].as<std::string>();
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
    }

    // IMU和点云的回调函数
    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * 10.0,
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        CloudType::Ptr cloud = Utils::livox2PCL(msg, m_config.lidar_filter_num, m_config.lidar_min_range, m_config.lidar_max_range);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    // 同步IMU和点云数据
    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
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

    void timerCB()
    {
        if (!syncPackage())
        {
            // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for IMU and Lidar data...");
            return;
        }

        
        if (m_builder_status == BuilderStatus::IMU_INIT)
        {
            if(imu_processor->initialize(m_package))
                m_builder_status = BuilderStatus::MAP_INIT;
            return ;
        }
        imu_processor->undistort(m_package);

        if (m_builder_status == BuilderStatus::MAP_INIT)
        {
            CloudType::Ptr cloud_world = LidarProcessor::transformCloud(m_package.cloud, lidar_processor->r_wl(), lidar_processor->t_wl());
            lidar_processor->initCloudMap(cloud_world->points);
            m_state.message_lidar_received = true;
            m_builder_status = BuilderStatus::MAPPING;
            return ;
        }
        lidar_processor->process(m_package);
        m_state.last_message_time = Utils::getTime(m_package.cloud_end_time);
        if(!m_state.message_lidar_received)
            return;

        Eigen::Quaterniond q(kf->x().r_wi);
        m_state.last_r = Eigen::Quaterniond(q.w(),q.x(),q.y(),q.z()).toRotationMatrix();
        m_state.last_t = V3D(kf->x().t_wi.x(), kf->x().t_wi.y(), kf->x().t_wi.z());

        CloudType::Ptr body_cloud = lidar_processor->transformCloud(m_package.cloud, kf->x().r_il, kf->x().t_il);
        // RCLCPP_INFO(this->get_logger(), "body_cloud size before filtering: %zu", body_cloud->size());
        publishCloud(m_body_cloud_pub, body_cloud, m_package.cloud_end_time);
        
        //--------------------------------------------------------------------------------------
        //----------------------------------- Start Localize -----------------------------------
        //--------------------------------------------------------------------------------------
        
        rclcpp::Duration diff = rclcpp::Clock().now() - m_state.last_send_tf_time;
        bool update_tf = diff.seconds() > (1.0 / m_config.update_hz) && m_state.message_lidar_received;
        if (!update_tf)
        {
            sendBroadCastTF(m_state.last_message_time);
            return;
        }

        m_state.last_send_tf_time = rclcpp::Clock().now();

        M4F initial_guess = M4F::Identity();

        // 如果通过回调服务给定了初始位姿，就用给定的
        if (m_state.service_received)
        {
            std::lock_guard<std::mutex>(m_state.service_mutex);
            initial_guess = m_state.initial_guess;
        }
        else
        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            initial_guess.block<3, 3>(0, 0) = (m_state.last_offset_r * m_state.last_r).cast<float>();
            initial_guess.block<3, 1>(0, 3) = (m_state.last_offset_r * m_state.last_t + m_state.last_offset_t).cast<float>();
        }

        M3D current_local_r;
        V3D current_local_t;
        builtin_interfaces::msg::Time current_time;
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
            // RCLCPP_INFO(this->get_logger(), "Localize Success");
            M3D map_body_r = initial_guess.block<3, 3>(0, 0).cast<double>();
            V3D map_body_t = initial_guess.block<3, 1>(0, 3).cast<double>();
            m_state.last_offset_r = map_body_r * current_local_r.transpose();
            m_state.last_offset_t = -map_body_r * current_local_r.transpose() * current_local_t + map_body_t;
            if (!m_state.localize_success && m_state.service_received)
            {
                std::lock_guard<std::mutex>(m_state.service_mutex);
                m_state.localize_success = true;
                m_state.service_received = false;
            }
        }
        sendBroadCastTF(current_time);
        publishMapCloud(current_time);
    }

    // 发布TF
    void sendBroadCastTF(builtin_interfaces::msg::Time &time)
    {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = m_config.map_frame;
        transformStamped.child_frame_id = m_config.local_frame;
        transformStamped.header.stamp = time;
        Eigen::Quaterniond q(m_state.last_offset_r);
        V3D t = m_state.last_offset_t;
        transformStamped.transform.translation.x = t.x();
        transformStamped.transform.translation.y = t.y();
        transformStamped.transform.translation.z = t.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(transformStamped);

        geometry_msgs::msg::TransformStamped transformStamped2;
        transformStamped2.header.frame_id = "lidar";  // 发布的父坐标系
        transformStamped2.child_frame_id = "body";  // 发布的子坐标系
        transformStamped2.header.stamp = time;

        Eigen::Quaterniond q2(kf->x().r_wi);
        V3D t2 = kf->x().t_wi;
        transformStamped2.transform.translation.x = t2.x();
        transformStamped2.transform.translation.y = t2.y();
        transformStamped2.transform.translation.z = t2.z();
        transformStamped2.transform.rotation.x = q2.x();
        transformStamped2.transform.rotation.y = q2.y();
        transformStamped2.transform.rotation.z = q2.z();
        transformStamped2.transform.rotation.w = q2.w();
        m_tf_broadcaster->sendTransform(transformStamped2);
    }


    // 重定位服务回调函数，一般用于给定初始的位姿信息，或者改变当前的位姿信息
    void relocCB(const std::shared_ptr<interface::srv::Relocalize::Request> request, std::shared_ptr<interface::srv::Relocalize::Response> response)
    {
        float x = request->x;
        float y = request->y;
        float z = request->z;
        float yaw = request->yaw;
        float roll = request->roll;
        float pitch = request->pitch;

        Eigen::AngleAxisd yaw_angle = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd roll_angle = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_angle = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());

        {
            std::lock_guard<std::mutex>(m_state.message_mutex);
            m_state.initial_guess.setIdentity();
            m_state.initial_guess.block<3, 3>(0, 0) = (yaw_angle * roll_angle * pitch_angle).toRotationMatrix().cast<float>();
            m_state.initial_guess.block<3, 1>(0, 3) = V3F(x, y, z);
            m_state.service_received = true;
            m_state.localize_success = false;
        }

        response->success = true;
        response->message = "relocalize success";
        return;
    }

    // 重定位状态查询服务回调函数
    void relocCheckCB(const std::shared_ptr<interface::srv::IsValid::Request> request, std::shared_ptr<interface::srv::IsValid::Response> response)
    {
        std::lock_guard<std::mutex>(m_state.service_mutex);
        if (request->code == 1)
            response->valid = true;
        else
            response->valid = m_state.localize_success;
        return;
    }

    // 发布全局地图点云
    void publishMapCloud(builtin_interfaces::msg::Time &time)
    {
        // sleep(1); // 防止rviz还没启动起来，接受不到点云
        
        CloudType::Ptr map_cloud = m_localizer->refineMap(); // 返回下采样后的点云，用于显示
        if (map_cloud->size() < 1)
        {
            RCLCPP_WARN(this->get_logger(), "Map cloud is empty, skip publishing.");
            return;
        }
        
        sensor_msgs::msg::PointCloud2 map_cloud_msg;
        pcl::toROSMsg(*map_cloud, map_cloud_msg);
        map_cloud_msg.header.frame_id = m_config.map_frame;
        map_cloud_msg.header.stamp = time;
        m_map_cloud_pub->publish(map_cloud_msg);
    }

    // 发布机体坐标系下的点云
    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, const double &time)
    {
        // RCLCPP_INFO(this->get_logger(), "===================");
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        // RCLCPP_INFO(this->get_logger(), "-------------------");
        cloud_msg.header.frame_id = "body";
        cloud_msg.header.stamp = Utils::getTime(time);
        // RCLCPP_INFO(this->get_logger(), "++++++++++++++++++");
        pub->publish(cloud_msg);
        // RCLCPP_INFO(this->get_logger(), "Publish body cloud, size: %zu", cloud->size());
    }

private:
    Config m_config;
    NodeState m_state;

    ICPConfig m_localizer_config;
    StateData m_state_data;
    SyncPackage m_package;
    BuilderStatus m_builder_status;
    std::string pcd_path;
    std::shared_ptr<IESKF> kf;
    std::shared_ptr<IMUProcessor> imu_processor;
    std::shared_ptr<LidarProcessor> lidar_processor;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;
    std::shared_ptr<ICPLocalizer> m_localizer;
    rclcpp::TimerBase::SharedPtr m_timer;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    rclcpp::Service<interface::srv::Relocalize>::SharedPtr m_reloc_srv;
    rclcpp::Service<interface::srv::IsValid>::SharedPtr m_reloc_check_srv;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_map_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizerNode>());
    rclcpp::shutdown();
    return 0;
}
