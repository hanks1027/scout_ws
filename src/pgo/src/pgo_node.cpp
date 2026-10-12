#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <queue>
#include <filesystem>
#include <mutex>
#include <memory>

#include "pgos/commons.h"
#include "pgos/simple_pgo.h"
#include "interface/srv/save_maps.hpp"

#include <pcl/io/io.h>
#include <fstream>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

struct NodeConfig
{
    std::string cloud_topic = "/cloud_registered";
    std::string odom_topic  = "/Odometry";
    std::string map_frame   = "map";
    std::string local_frame = "camera_init";
};

struct NodeState
{
    std::mutex message_mutex;
    std::queue<CloudWithPose> cloud_buffer;
    double last_message_time = 0.0;
};

class PGONode : public rclcpp::Node
{
public:
    PGONode() : Node("pgo_node")
    {
        RCLCPP_INFO(this->get_logger(), "PGO node started");

        // -----------------------------------------------------------------
        // 1. 创建回调组，实现回调之间的并发执行
        //    - 消息同步回调：MutuallyExclusive（只处理同步/入队，轻量）
        //    - 主处理回调（timerCB）：MutuallyExclusive 或 Reentrant
        //      （耗时的 PGO 计算，单独一组，避免阻塞订阅和 TF）
        //    - TF 广播定时器：Reentrant，独立线程
        //    - Service 回调：独立组，避免 save 时阻塞主流程
        // -----------------------------------------------------------------
        m_sync_cb_group    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_process_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_tf_cb_group      = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        m_service_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        loadParameters();

        m_pgo = std::make_shared<SimplePGO>(m_pgo_config);

        // -----------------------------------------------------------------
        // 2. message_filters 订阅：显式指定 callback_group
        // -----------------------------------------------------------------
        rclcpp::SubscriptionOptions sub_opts;
        sub_opts.callback_group = m_sync_cb_group;

        rclcpp::QoS qos = rclcpp::QoS(10);
        m_cloud_sub.subscribe(this, m_node_config.cloud_topic, qos.get_rmw_qos_profile(), sub_opts);
        m_odom_sub.subscribe(this, m_node_config.odom_topic, qos.get_rmw_qos_profile(), sub_opts);

        m_loop_marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pgo/loop_markers", 10000);

        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        m_sync = std::make_shared<message_filters::Synchronizer<
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>>(
            message_filters::sync_policies::ApproximateTime<
                sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>(10),
            m_cloud_sub, m_odom_sub);
        m_sync->setAgePenalty(0.1);
        m_sync->registerCallback(
            std::bind(&PGONode::syncCB, this, std::placeholders::_1, std::placeholders::_2));

        // -----------------------------------------------------------------
        // 3. 主处理定时器：跑 PGO 计算（重活），挂到独立的 callback group
        // -----------------------------------------------------------------
        m_timer = this->create_wall_timer(
            50ms,
            std::bind(&PGONode::timerCB, this),
            m_process_cb_group);

        // -----------------------------------------------------------------
        // 4. TF 广播定时器：独立高频发 TF，永不阻塞
        // -----------------------------------------------------------------
        m_tf_timer = this->create_wall_timer(
            20ms,        // 50Hz
            std::bind(&PGONode::tfTimerCB, this),
            m_tf_cb_group);

        // -----------------------------------------------------------------
        // 5. Service 单独回调组
        // -----------------------------------------------------------------
        m_save_map_srv = this->create_service<interface::srv::SaveMaps>(
            "/pgo/save_maps",
            std::bind(&PGONode::saveMapsCB, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            m_service_cb_group);
    }

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

        m_node_config.cloud_topic = config["cloud_topic"].as<std::string>();
        m_node_config.odom_topic  = config["odom_topic"].as<std::string>();
        m_node_config.map_frame   = config["map_frame"].as<std::string>();
        m_node_config.local_frame = config["local_frame"].as<std::string>();

        m_pgo_config.key_pose_delta_deg      = config["key_pose_delta_deg"].as<double>();
        m_pgo_config.key_pose_delta_trans    = config["key_pose_delta_trans"].as<double>();
        m_pgo_config.loop_search_radius      = config["loop_search_radius"].as<double>();
        m_pgo_config.loop_time_tresh         = config["loop_time_tresh"].as<double>();
        m_pgo_config.loop_score_tresh        = config["loop_score_tresh"].as<double>();
        m_pgo_config.loop_submap_half_range  = config["loop_submap_half_range"].as<int>();
        m_pgo_config.submap_resolution       = config["submap_resolution"].as<double>();
        m_pgo_config.min_loop_detect_duration= config["min_loop_detect_duration"].as<double>();
    }

    // ---------------------------------------------------------------------
    // 消息同步回调（运行在 m_sync_cb_group）
    //   - 只做轻量级的入队，不做点云处理
    // ---------------------------------------------------------------------
    void syncCB(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
                const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg)
    {
        CloudWithPose cp;
        cp.pose.setTime(cloud_msg->header.stamp.sec, cloud_msg->header.stamp.nanosec);

        // 轻量的 int 检查放在锁外
        if (cp.pose.second < m_state.last_message_time)
        {
            RCLCPP_WARN(this->get_logger(), "Received out of order message");
            return;
        }

        cp.pose.r = Eigen::Quaterniond(
                        odom_msg->pose.pose.orientation.w,
                        odom_msg->pose.pose.orientation.x,
                        odom_msg->pose.pose.orientation.y,
                        odom_msg->pose.pose.orientation.z)
                        .toRotationMatrix();
        cp.pose.t = V3D(odom_msg->pose.pose.position.x,
                        odom_msg->pose.pose.position.y,
                        odom_msg->pose.pose.position.z);

        // 点云拷贝较耗时，放锁外
        cp.cloud = CloudType::Ptr(new CloudType);
        pcl::fromROSMsg(*cloud_msg, *cp.cloud);

        {
            std::lock_guard<std::mutex> lk(m_state.message_mutex);
            m_state.last_message_time = cp.pose.second;
            m_state.cloud_buffer.push(cp);
        }
    }

    // ---------------------------------------------------------------------
    // TF 广播定时器回调（运行在 m_tf_cb_group，50Hz 独立线程）
    //   - 只读 m_map_to_caminit，不参与 PGO 计算
    //   - stamp 使用 now()，避免 TF buffer 时间戳错位
    // ---------------------------------------------------------------------
    void tfTimerCB()
    {
        Eigen::Isometry3d T;
        {
            std::lock_guard<std::mutex> lk(m_tf_mutex);
            T = m_map_to_caminit;
        }

        geometry_msgs::msg::TransformStamped ts;
        ts.header.stamp    = this->now();          // 关键：用 now()
        ts.header.frame_id = m_node_config.map_frame;
        ts.child_frame_id  = m_node_config.local_frame;

        ts.transform.translation.x = T.translation().x();
        ts.transform.translation.y = T.translation().y();
        ts.transform.translation.z = T.translation().z();

        Eigen::Quaterniond q(T.rotation());
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();

        m_tf_broadcaster->sendTransform(ts);
    }

    // ---------------------------------------------------------------------
    // 内部：把 PGO 的 offsetR/offsetT 更新到共享变量
    // ---------------------------------------------------------------------
    void updateMapToCaminit()
    {
        Eigen::Quaterniond q(m_pgo->offsetR());
        V3D t = m_pgo->offsetT();

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear()        = q.toRotationMatrix();
        T.translation()   = t;

        std::lock_guard<std::mutex> lk(m_tf_mutex);
        m_map_to_caminit = T;
    }

    // ---------------------------------------------------------------------
    // 主处理回调（运行在 m_process_cb_group）
    //   - 与 syncCB / TF 广播分离，即使卡住也不会阻断 TF
    // ---------------------------------------------------------------------
    void timerCB()
    {
        CloudWithPose cp;
        bool has_data = false;
        {
            std::lock_guard<std::mutex> lk(m_state.message_mutex);
            if (m_state.cloud_buffer.empty())
                return;
            // 取队首处理
            cp = m_state.cloud_buffer.front();
            m_state.cloud_buffer.pop();
            has_data = true;
        }

        if (!has_data)
            return;

        if (!m_pgo->addKeyPose(cp))
        {
            // 未形成新关键帧：也要更新 TF（offset 可能已变化）
            updateMapToCaminit();
            return;
        }

        m_pgo->searchForLoopPairs();
        m_pgo->smoothAndUpdate();

        // 更新共享的 map->camera_init（TF 定时器负责广播）
        updateMapToCaminit();

        // 回环可视化：用 now() 即可
        publishLoopMarkers(this->now());
    }

    void publishLoopMarkers(const builtin_interfaces::msg::Time &time)
    {
        if (m_loop_marker_pub->get_subscription_count() == 0)
            return;
        if (m_pgo->historyPairs().size() == 0)
            return;

        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker nodes_marker;
        visualization_msgs::msg::Marker edges_marker;

        nodes_marker.header.frame_id = m_node_config.map_frame;
        nodes_marker.header.stamp    = time;
        nodes_marker.ns              = "pgo_nodes";
        nodes_marker.id              = 0;
        nodes_marker.type            = visualization_msgs::msg::Marker::SPHERE_LIST;
        nodes_marker.action          = visualization_msgs::msg::Marker::ADD;
        nodes_marker.pose.orientation.w = 1.0;
        nodes_marker.scale.x = 0.3;
        nodes_marker.scale.y = 0.3;
        nodes_marker.scale.z = 0.3;
        nodes_marker.color.r = 1.0;
        nodes_marker.color.g = 0.8;
        nodes_marker.color.b = 0.0;
        nodes_marker.color.a = 1.0;

        edges_marker.header.frame_id = m_node_config.map_frame;
        edges_marker.header.stamp    = time;
        edges_marker.ns              = "pgo_edges";
        edges_marker.id              = 1;
        edges_marker.type            = visualization_msgs::msg::Marker::LINE_LIST;
        edges_marker.action          = visualization_msgs::msg::Marker::ADD;
        edges_marker.pose.orientation.w = 1.0;
        edges_marker.scale.x = 0.1;
        edges_marker.color.r = 0.0;
        edges_marker.color.g = 0.8;
        edges_marker.color.b = 0.0;
        edges_marker.color.a = 1.0;

        std::vector<KeyPoseWithCloud> &poses = m_pgo->keyPoses();
        std::vector<std::pair<size_t, size_t>> &pairs = m_pgo->historyPairs();

        for (size_t i = 0; i < pairs.size(); i++)
        {
            size_t i1 = pairs[i].first;
            size_t i2 = pairs[i].second;

            geometry_msgs::msg::Point p1, p2;
            p1.x = poses[i1].t_global.x();
            p1.y = poses[i1].t_global.y();
            p1.z = poses[i1].t_global.z();

            p2.x = poses[i2].t_global.x();
            p2.y = poses[i2].t_global.y();
            p2.z = poses[i2].t_global.z();

            nodes_marker.points.push_back(p1);
            nodes_marker.points.push_back(p2);
            edges_marker.points.push_back(p1);
            edges_marker.points.push_back(p2);
        }

        marker_array.markers.push_back(nodes_marker);
        marker_array.markers.push_back(edges_marker);
        m_loop_marker_pub->publish(marker_array);
    }

    // ---------------------------------------------------------------------
    // Service 回调（运行在 m_service_cb_group）
    //   注意：与 timerCB 可能并发执行，PGO 内部结构的读取需自行保证安全。
    //   如果 SimplePGO 不是线程安全的，建议在访问 keyPoses() 前加锁。
    // ---------------------------------------------------------------------
    void saveMapsCB(const std::shared_ptr<interface::srv::SaveMaps::Request> request,
                    std::shared_ptr<interface::srv::SaveMaps::Response> response)
    {
        if (!std::filesystem::exists(request->file_path))
        {
            response->success = false;
            response->message = request->file_path + " IS NOT EXISTS!";
            return;
        }

        if (m_pgo->keyPoses().size() == 0)
        {
            response->success = false;
            response->message = "NO POSES!";
            return;
        }

        std::filesystem::path p_dir(request->file_path);
        std::filesystem::path patches_dir   = p_dir / "patches";
        std::filesystem::path poses_txt_path= p_dir / "poses.txt";
        std::filesystem::path map_path      = p_dir / "map.pcd";

        if (request->save_patches)
        {
            if (std::filesystem::exists(patches_dir))
                std::filesystem::remove_all(patches_dir);
            std::filesystem::create_directories(patches_dir);

            if (std::filesystem::exists(poses_txt_path))
                std::filesystem::remove(poses_txt_path);

            RCLCPP_INFO(this->get_logger(), "Patches Path: %s", patches_dir.string().c_str());
        }

        RCLCPP_INFO(this->get_logger(), "SAVE MAP TO %s", map_path.string().c_str());

        std::ofstream txt_file(poses_txt_path);
        CloudType::Ptr ret(new CloudType);

        for (size_t i = 0; i < m_pgo->keyPoses().size(); i++)
        {
            CloudType::Ptr body_cloud = m_pgo->keyPoses()[i].body_cloud;

            if (request->save_patches)
            {
                std::string patch_name = std::to_string(i) + ".pcd";
                std::filesystem::path patch_path = patches_dir / patch_name;
                pcl::io::savePCDFileBinary(patch_path.string(), *body_cloud);

                Eigen::Quaterniond q(m_pgo->keyPoses()[i].r_global);
                V3D t = m_pgo->keyPoses()[i].t_global;
                txt_file << patch_name << " "
                         << t.x() << " " << t.y() << " " << t.z() << " "
                         << q.w() << " " << q.x() << " " << q.y() << " " << q.z()
                         << std::endl;
            }

            CloudType::Ptr world_cloud(new CloudType);
            pcl::transformPointCloud(*body_cloud, *world_cloud,
                                     m_pgo->keyPoses()[i].t_global,
                                     Eigen::Quaterniond(m_pgo->keyPoses()[i].r_global));
            *ret += *world_cloud;
        }

        txt_file.close();
        pcl::io::savePCDFileBinary(map_path.string(), *ret);

        response->success = true;
        response->message = "SAVE SUCCESS!";
    }

private:
    NodeConfig m_node_config;
    Config     m_pgo_config;
    NodeState  m_state;

    std::shared_ptr<SimplePGO> m_pgo;

    // Timers
    rclcpp::TimerBase::SharedPtr m_timer;      // 主处理
    rclcpp::TimerBase::SharedPtr m_tf_timer;   // TF 广播

    // Publishers / Service
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_loop_marker_pub;
    rclcpp::Service<interface::srv::SaveMaps>::SharedPtr m_save_map_srv;

    // message_filters
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry>       m_odom_sub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<message_filters::Synchronizer<
        message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::PointCloud2, nav_msgs::msg::Odometry>>> m_sync;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr m_sync_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_process_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_tf_cb_group;
    rclcpp::CallbackGroup::SharedPtr m_service_cb_group;

    // map -> camera_init，供 TF 定时器以 now() 广播
    Eigen::Isometry3d m_map_to_caminit = Eigen::Isometry3d::Identity();
    std::mutex        m_tf_mutex;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PGONode>();

    // 使用多线程执行器：让不同 callback group 真正并发执行
    rclcpp::executors::MultiThreadedExecutor executor(
        rclcpp::ExecutorOptions(), /* number_of_threads = */ 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}