// [260525] Gazebo Sim Livox Mid360 CSV-pattern simulator.

#ifndef LIVOX_MID360_GZ_SIM__LIVOX_MID360_GZ_PLUGIN_HPP_
#define LIVOX_MID360_GZ_SIM__LIVOX_MID360_GZ_PLUGIN_HPP_

#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace livox_mid360_gz_sim
{

class LivoxMid360GzPlugin
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
public:
  LivoxMid360GzPlugin();
  ~LivoxMid360GzPlugin() override;

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & eventMgr) override;

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override;

private:
  struct CsvRay
  {
    double azimuth{0.0};
    double zenith{0.0};
    uint8_t line{0};
  };

  struct PointSample
  {
    float x{0.0F};
    float y{0.0F};
    float z{0.0F};
    uint8_t line{0};
    uint8_t reflectivity{100};
  };

  struct CollisionPrimitive
  {
    enum class Type { Box, Plane };
    Type type{Type::Box};
    gz::math::Pose3d pose;
    gz::math::Vector3d size;
    gz::math::Vector3d normal{0.0, 0.0, 1.0};
    double boundingRadius{0.0};
  };

  bool LoadCsv(const std::string & fileName);
  bool IsModelDescendant(
    gz::sim::Entity entity,
    const gz::sim::EntityComponentManager & ecm) const;
  bool RaycastCollision(
    const gz::math::Vector3d & origin,
    const gz::math::Vector3d & direction,
    const std::vector<CollisionPrimitive> & collisions,
    double & range) const;
  std::vector<CollisionPrimitive> CollectCollisionPrimitives(
    const gz::sim::EntityComponentManager & ecm) const;
  void PublishScan(
    const std::vector<PointSample> & points,
    int64_t scanStartNs,
    int64_t scanDurationNs);
  void PublishGroundTruthOdometry(const gz::math::Pose3d & pose, int64_t stampNs);
  void PublishPointCloud2(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const std::vector<PointSample> & points,
    int64_t stampNs,
    const std::string & frameId);

  gz::sim::Entity modelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity lidarLinkEntity_{gz::sim::kNullEntity};
  gz::sim::Model model_;

  std::vector<CsvRay> pattern_;
  int64_t patternStartIndex_{0};

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr customPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloudPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr groundTruthCloudPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr groundTruthOdomPub_;

  std::string linkName_{"mid360_link"};
  std::string nodeName_{"livox_mid360_gz_sim"};
  std::string frameId_{"laser_link"};
  std::string topic_{"/livox/lidar"};
  std::string pointsTopic_{"/livox/lidar/points"};
  std::string groundTruthPointsTopic_{"/ground_truth/livox/points"};
  std::string groundTruthFrameId_{"camera_init"};
  std::string groundTruthOdomTopic_{"/ground_truth/odometry"};
  std::string csvFileName_;
  bool enableGroundTruth_{true};

  int samples_{20000};
  int downsample_{1};
  double updateRate_{10.0};
  double minRange_{0.2};
  double maxRange_{50.0};
  int noiseSeed_{42};
  double rangeNoiseStddev_{0.01};
  double rangeBias_{0.0};
  double rangeDriftPerSecond_{0.0002};
  double dropoutRate_{0.01};
  std::mt19937 rng_{42};
  std::normal_distribution<double> rangeNoise_{0.0, 0.01};
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};
  gz::math::Pose3d initialLidarPose_;
  bool initialLidarPoseSet_{false};
  gz::math::Pose3d lastLidarPose_;
  int64_t lastPubNs_{-1};
};

}  // namespace livox_mid360_gz_sim

#endif  // LIVOX_MID360_GZ_SIM__LIVOX_MID360_GZ_PLUGIN_HPP_
