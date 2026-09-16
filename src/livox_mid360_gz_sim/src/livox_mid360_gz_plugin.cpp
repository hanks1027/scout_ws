// [260525] Gazebo Sim Livox Mid360 CSV-pattern simulator.

#include "livox_mid360_gz_sim/livox_mid360_gz_plugin.hpp"
#include "livox_mid360_gz_sim/livox_scan_timing.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Geometry.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include <sdf/Box.hh>
#include <sdf/Plane.hh>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#ifndef GZ_ADD_PLUGIN
#define GZ_ADD_PLUGIN IGNITION_ADD_PLUGIN
#endif

namespace livox_mid360_gz_sim
{
namespace
{
constexpr double kDegToRad = M_PI / 180.0;
std::string SdfString(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  const std::string & fallback)
{
  return sdf->HasElement(name) ? sdf->Get<std::string>(name) : fallback;
}

double SdfDouble(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  const double fallback)
{
  return sdf->HasElement(name) ? sdf->Get<double>(name) : fallback;
}

int SdfInt(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  const int fallback)
{
  return sdf->HasElement(name) ? sdf->Get<int>(name) : fallback;
}

bool SdfBool(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  const bool fallback)
{
  return sdf->HasElement(name) ? sdf->Get<bool>(name) : fallback;
}
}  // namespace

LivoxMid360GzPlugin::LivoxMid360GzPlugin() = default;

LivoxMid360GzPlugin::~LivoxMid360GzPlugin() = default;

void LivoxMid360GzPlugin::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager & /*eventMgr*/)
{
  modelEntity_ = entity;
  model_ = gz::sim::Model(modelEntity_);

  linkName_ = SdfString(sdf, "link_name", linkName_);
  nodeName_ = SdfString(sdf, "node_name", nodeName_);
  frameId_ = SdfString(sdf, "frame_id", frameId_);
  topic_ = SdfString(sdf, "topic", topic_);
  pointsTopic_ = SdfString(sdf, "points_topic", pointsTopic_);
  groundTruthPointsTopic_ = SdfString(
    sdf, "ground_truth_points_topic", groundTruthPointsTopic_);
  groundTruthFrameId_ = SdfString(
    sdf, "ground_truth_frame_id", groundTruthFrameId_);
  groundTruthOdomTopic_ = SdfString(
    sdf, "ground_truth_odom_topic", groundTruthOdomTopic_);
  enableGroundTruth_ = SdfBool(sdf, "enable_ground_truth", enableGroundTruth_);
  csvFileName_ = SdfString(sdf, "csv_file_name", csvFileName_);
  samples_ = SdfInt(sdf, "samples", samples_);
  downsample_ = std::max(1, SdfInt(sdf, "downsample", downsample_));
  updateRate_ = SdfDouble(sdf, "update_rate", updateRate_);
  minRange_ = SdfDouble(sdf, "min_range", minRange_);
  maxRange_ = SdfDouble(sdf, "max_range", maxRange_);
  noiseSeed_ = SdfInt(sdf, "noise_seed", noiseSeed_);
  rangeNoiseStddev_ = std::max(
    0.0, SdfDouble(sdf, "range_noise_stddev", rangeNoiseStddev_));
  rangeBias_ = SdfDouble(sdf, "range_bias", rangeBias_);
  rangeDriftPerSecond_ = SdfDouble(
    sdf, "range_drift_per_second", rangeDriftPerSecond_);
  dropoutRate_ = std::clamp(
    SdfDouble(sdf, "dropout_rate", dropoutRate_), 0.0, 1.0);
  rng_.seed(static_cast<std::mt19937::result_type>(noiseSeed_));
  rangeNoise_ = std::normal_distribution<double>(0.0, rangeNoiseStddev_);

  lidarLinkEntity_ = model_.LinkByName(ecm, linkName_);
  if (lidarLinkEntity_ == gz::sim::kNullEntity) {
    std::cerr << "[LivoxMid360GzPlugin] Link not ready yet: "
              << linkName_ << std::endl;
  } else if (!ecm.Component<gz::sim::components::WorldPose>(lidarLinkEntity_)) {
    ecm.CreateComponent(lidarLinkEntity_, gz::sim::components::WorldPose());
  }

  if (!LoadCsv(csvFileName_)) {
    return;
  }

  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  node_ = std::make_shared<rclcpp::Node>(nodeName_);
  customPub_ = node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(
    topic_, rclcpp::QoS(20));
  cloudPub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
    pointsTopic_, rclcpp::SensorDataQoS());
  if (enableGroundTruth_) {
    groundTruthCloudPub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      groundTruthPointsTopic_, rclcpp::SensorDataQoS());
    groundTruthOdomPub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      groundTruthOdomTopic_, rclcpp::QoS(20));
  }

  std::cout << "[LivoxMid360GzPlugin] Loaded " << pattern_.size()
            << " CSV rays from " << csvFileName_ << std::endl;
  std::cout << "[LivoxMid360GzPlugin] Publishing " << topic_
            << " (livox_ros_driver2/CustomMsg), " << pointsTopic_
            << " (PointCloud2), frame_id=" << frameId_ << std::endl;
  std::cout << "[LivoxMid360GzPlugin] Ground truth "
            << (enableGroundTruth_ ? groundTruthPointsTopic_ : "disabled")
            << ", frame_id=" << groundTruthFrameId_
            << "; noise seed=" << noiseSeed_
            << ", sigma=" << rangeNoiseStddev_
            << " m, bias=" << rangeBias_
            << " m, drift=" << rangeDriftPerSecond_
            << " m/s, dropout=" << dropoutRate_ << std::endl;
}

bool LivoxMid360GzPlugin::LoadCsv(const std::string & fileName)
{
  std::ifstream file(fileName);
  if (!file.is_open()) {
    std::cerr << "[LivoxMid360GzPlugin] Cannot read CSV file: "
              << fileName << std::endl;
    return false;
  }

  std::string line;
  std::getline(file, line);

  pattern_.clear();
  pattern_.reserve(800000);
  uint64_t index = 0;
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }

    std::stringstream lineStream(line);
    std::string timeStr;
    std::string azimuthStr;
    std::string zenithStr;
    if (!std::getline(lineStream, timeStr, ',') ||
      !std::getline(lineStream, azimuthStr, ',') ||
      !std::getline(lineStream, zenithStr, ','))
    {
      continue;
    }

    try {
      CsvRay ray;
      ray.azimuth = std::stod(azimuthStr) * kDegToRad;
      ray.zenith = std::stod(zenithStr) * kDegToRad - M_PI_2;
      ray.line = static_cast<uint8_t>(index % 4);
      pattern_.push_back(ray);
      ++index;
    } catch (const std::exception &) {
      continue;
    }
  }

  return !pattern_.empty();
}

bool LivoxMid360GzPlugin::IsModelDescendant(
  const gz::sim::Entity entity,
  const gz::sim::EntityComponentManager & ecm) const
{
  auto current = entity;
  while (current != gz::sim::kNullEntity) {
    if (current == modelEntity_) {
      return true;
    }
    auto parent = ecm.Component<gz::sim::components::ParentEntity>(current);
    if (!parent) {
      return false;
    }
    current = parent->Data();
  }
  return false;
}

bool LivoxMid360GzPlugin::RaycastCollision(
  const gz::math::Vector3d & origin,
  const gz::math::Vector3d & direction,
  const std::vector<CollisionPrimitive> & collisions,
  double & range) const
{
  bool hit = false;
  double best = maxRange_ - minRange_;

  for (const auto & collision : collisions) {
    double candidate = -1.0;

    if (collision.type == CollisionPrimitive::Type::Box) {
      const auto centerToOrigin = collision.pose.Pos() - origin;
      const double closestAlongRay = centerToOrigin.Dot(direction);
      if (closestAlongRay + collision.boundingRadius < 0.0 ||
        closestAlongRay - collision.boundingRadius > best)
      {
        continue;
      }

      const double centerDistanceSq = centerToOrigin.SquaredLength();
      const double closestDistanceSq =
        centerDistanceSq - closestAlongRay * closestAlongRay;
      const double radiusSq =
        collision.boundingRadius * collision.boundingRadius;
      if (closestDistanceSq > radiusSq) {
        continue;
      }

      const auto half = collision.size * 0.5;
      const auto localOrigin =
        collision.pose.Rot().RotateVectorReverse(origin - collision.pose.Pos());
      const auto localDirection =
        collision.pose.Rot().RotateVectorReverse(direction);

      double tMin = -std::numeric_limits<double>::infinity();
      double tMax = std::numeric_limits<double>::infinity();
      const double minBounds[3] = {-half.X(), -half.Y(), -half.Z()};
      const double maxBounds[3] = {half.X(), half.Y(), half.Z()};
      const double o[3] = {localOrigin.X(), localOrigin.Y(), localOrigin.Z()};
      const double d[3] = {
        localDirection.X(), localDirection.Y(), localDirection.Z()};

      bool intersects = true;
      for (int axis = 0; axis < 3; ++axis) {
        if (std::abs(d[axis]) < 1e-9) {
          if (o[axis] < minBounds[axis] || o[axis] > maxBounds[axis]) {
            intersects = false;
            break;
          }
          continue;
        }

        double t1 = (minBounds[axis] - o[axis]) / d[axis];
        double t2 = (maxBounds[axis] - o[axis]) / d[axis];
        if (t1 > t2) {
          std::swap(t1, t2);
        }
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax) {
          intersects = false;
          break;
        }
      }

      if (!intersects) {
        continue;
      }
      candidate = tMin >= 0.0 ? tMin : tMax;
    } else if (collision.type == CollisionPrimitive::Type::Plane) {
      const auto normal =
        collision.pose.Rot().RotateVector(collision.normal).Normalize();
      const double denom = normal.Dot(direction);
      if (std::abs(denom) < 1e-9) {
        continue;
      }
      candidate = normal.Dot(collision.pose.Pos() - origin) / denom;
    }

    if (candidate > 0.0 && candidate < best) {
      best = candidate;
      hit = true;
    }
  }

  if (!hit) {
    return false;
  }

  range = minRange_ + best;
  return range > minRange_ && range < maxRange_;
}

std::vector<LivoxMid360GzPlugin::CollisionPrimitive>
LivoxMid360GzPlugin::CollectCollisionPrimitives(
  const gz::sim::EntityComponentManager & ecm) const
{
  std::vector<CollisionPrimitive> collisions;
  ecm.Each<
    gz::sim::components::Collision,
    gz::sim::components::ParentEntity>(
    [&](const gz::sim::Entity & entity,
    const gz::sim::components::Collision *,
    const gz::sim::components::ParentEntity * parentComp)
    {
      if (IsModelDescendant(entity, ecm)) {
        return true;
      }

      const auto geometryComp =
      ecm.Component<gz::sim::components::Geometry>(entity);
      if (!geometryComp) {
        return true;
      }

      const auto pose = gz::sim::worldPose(entity, ecm);

      const auto & geometry = geometryComp->Data();
      const bool isPlane =
      geometry.Type() == sdf::GeometryType::PLANE && geometry.PlaneShape();

      if (geometry.Type() == sdf::GeometryType::BOX && geometry.BoxShape()) {
        CollisionPrimitive primitive;
        primitive.type = CollisionPrimitive::Type::Box;
        primitive.pose = pose;
        primitive.size = geometry.BoxShape()->Size();
        primitive.boundingRadius = primitive.size.Length() * 0.5;
        collisions.push_back(primitive);
      } else if (isPlane) {
        CollisionPrimitive primitive;
        primitive.type = CollisionPrimitive::Type::Plane;
        primitive.pose = pose;
        primitive.normal = geometry.PlaneShape()->Normal();
        collisions.push_back(primitive);
      }
      return true;
    });
  return collisions;
}

void LivoxMid360GzPlugin::PostUpdate(
  const gz::sim::UpdateInfo & info,
  const gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || !customPub_ || pattern_.empty()) {
    return;
  }

  if (lidarLinkEntity_ == gz::sim::kNullEntity) {
    lidarLinkEntity_ = model_.LinkByName(ecm, linkName_);
    if (lidarLinkEntity_ == gz::sim::kNullEntity) {
      return;
    }
    std::cout << "[LivoxMid360GzPlugin] Found LiDAR link: "
              << linkName_ << std::endl;
  }

  const int64_t nowNs =
    std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
  const int64_t periodNs = updateRate_ > 0.0 ?
    static_cast<int64_t>(1000000000.0 / updateRate_) : 0;
  if (lastPubNs_ >= 0 && nowNs <= lastPubNs_) {
    // [260716] A Gazebo time reset invalidates the scan-start pose and timestamp.
    lastPubNs_ = -1;
    initialLidarPoseSet_ = false;
  }
  if (periodNs > 0 && lastPubNs_ >= 0 && nowNs - lastPubNs_ < periodNs) {
    return;
  }

  auto link = gz::sim::Link(lidarLinkEntity_);
  auto poseOpt = link.WorldPose(ecm);
  if (!poseOpt) {
    return;
  }

  const auto & pose = *poseOpt;
  if (!initialLidarPoseSet_) {
    initialLidarPose_ = pose;
    initialLidarPoseSet_ = true;
    lastLidarPose_ = pose;
    lastPubNs_ = nowNs;
    // [260716] The first pose seeds the scan interval; publishing starts at the next pose.
    return;
  }
  const int64_t scanStartNs = lastPubNs_;
  const int64_t scanDurationNs = nowNs - scanStartNs;
  const auto collisions = CollectCollisionPrimitives(ecm);
  const int usableSamples = std::max(1, (samples_ + downsample_ - 1) / downsample_);
  std::vector<PointSample> points;
  points.reserve(static_cast<size_t>(usableSamples));
  std::vector<PointSample> groundTruthPoints;
  groundTruthPoints.reserve(static_cast<size_t>(usableSamples));

  const int64_t endIndex = patternStartIndex_ + samples_;
  std::size_t pointIndex = 0U;
  for (int64_t k = patternStartIndex_; k < endIndex; k += downsample_) {
    const auto & ray = pattern_[static_cast<size_t>(k % pattern_.size())];
    const double scanFraction = ScanPointFraction(pointIndex, usableSamples);
    const auto samplePosition = lastLidarPose_.Pos() +
      (pose.Pos() - lastLidarPose_.Pos()) * scanFraction;
    const auto sampleRotation = gz::math::Quaterniond::Slerp(
      scanFraction, lastLidarPose_.Rot(), pose.Rot());
    const gz::math::Pose3d samplePose(samplePosition, sampleRotation);
    const gz::math::Quaterniond rayRot(0.0, ray.zenith, ray.azimuth);
    const auto localDir = rayRot.RotateVector(gz::math::Vector3d::UnitX);
    const auto worldDir = samplePose.Rot().RotateVector(localDir).Normalize();
    const auto queryOrigin = samplePose.Pos() + worldDir * minRange_;

    double idealRange = 0.0;
    // [260526] Keep scheduled no-hit rays as zero-range points so Livox
    // offset_time follows the original firing order, matching the ROS1 plugin.
    const bool hit = RaycastCollision(queryOrigin, worldDir, collisions, idealRange);

    if (hit) {
      const auto worldPoint =
        samplePose.Pos() + samplePose.Rot().RotateVector(localDir * idealRange);
      const auto initialFramePoint = initialLidarPose_.Rot().RotateVectorReverse(
        worldPoint - initialLidarPose_.Pos());
      PointSample groundTruthSample;
      groundTruthSample.x = static_cast<float>(initialFramePoint.X());
      groundTruthSample.y = static_cast<float>(initialFramePoint.Y());
      groundTruthSample.z = static_cast<float>(initialFramePoint.Z());
      groundTruthSample.line = ray.line;
      groundTruthPoints.push_back(groundTruthSample);
    }

    double measuredRange = 0.0;
    if (hit && uniform_(rng_) >= dropoutRate_) {
      const int64_t pointStampNs = scanStartNs + static_cast<int64_t>(
        ScanPointOffsetNs(pointIndex, usableSamples, scanDurationNs));
      const double pointTimeSeconds = static_cast<double>(pointStampNs) / 1.0e9;
      const double timeVaryingBias =
        rangeBias_ + rangeDriftPerSecond_ * pointTimeSeconds;
      measuredRange = idealRange + timeVaryingBias;
      if (rangeNoiseStddev_ > 0.0) {
        measuredRange += rangeNoise_(rng_);
      }
      if (measuredRange <= minRange_ || measuredRange >= maxRange_) {
        measuredRange = 0.0;
      }
    }

    const auto point = localDir * measuredRange;
    PointSample sample;
    sample.x = static_cast<float>(point.X());
    sample.y = static_cast<float>(point.Y());
    sample.z = static_cast<float>(point.Z());
    sample.line = ray.line;
    points.push_back(sample);
    ++pointIndex;
  }

  patternStartIndex_ += samples_;
  if (patternStartIndex_ >= static_cast<int64_t>(pattern_.size())) {
    patternStartIndex_ %= static_cast<int64_t>(pattern_.size());
  }

  // [260716] Header is scan begin and the last point offset reaches the current pose.
  PublishScan(points, scanStartNs, scanDurationNs);
  PublishGroundTruthOdometry(pose, nowNs);
  PublishPointCloud2(cloudPub_, points, scanStartNs, frameId_);
  PublishPointCloud2(
    groundTruthCloudPub_, groundTruthPoints, nowNs, groundTruthFrameId_);
  lastLidarPose_ = pose;
  lastPubNs_ = nowNs;
}

void LivoxMid360GzPlugin::PublishGroundTruthOdometry(
  const gz::math::Pose3d & pose,
  const int64_t stampNs)
{
  if (!groundTruthOdomPub_ || !initialLidarPoseSet_) {
    return;
  }

  const auto relativePose = initialLidarPose_.Inverse() * pose;
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = groundTruthFrameId_;
  msg.header.stamp.sec = static_cast<int32_t>(stampNs / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(stampNs % 1000000000LL);
  msg.child_frame_id = frameId_;
  msg.pose.pose.position.x = relativePose.Pos().X();
  msg.pose.pose.position.y = relativePose.Pos().Y();
  msg.pose.pose.position.z = relativePose.Pos().Z();
  msg.pose.pose.orientation.x = relativePose.Rot().X();
  msg.pose.pose.orientation.y = relativePose.Rot().Y();
  msg.pose.pose.orientation.z = relativePose.Rot().Z();
  msg.pose.pose.orientation.w = relativePose.Rot().W();
  groundTruthOdomPub_->publish(msg);
}

void LivoxMid360GzPlugin::PublishScan(
  const std::vector<PointSample> & points,
  const int64_t scanStartNs,
  const int64_t scanDurationNs)
{
  livox_ros_driver2::msg::CustomMsg msg;
  msg.header.frame_id = frameId_;
  msg.header.stamp.sec = static_cast<int32_t>(scanStartNs / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(scanStartNs % 1000000000LL);
  msg.timebase = static_cast<uint64_t>(scanStartNs);
  msg.lidar_id = 1;
  msg.point_num = static_cast<uint32_t>(points.size());
  msg.points.reserve(points.size());

  for (size_t i = 0; i < points.size(); ++i) {
    livox_ros_driver2::msg::CustomPoint point;
    point.offset_time = ScanPointOffsetNs(i, points.size(), scanDurationNs);
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.reflectivity = points[i].reflectivity;
    point.tag = 0x10;
    point.line = points[i].line;
    msg.points.push_back(point);
  }

  customPub_->publish(msg);
}

void LivoxMid360GzPlugin::PublishPointCloud2(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
  const std::vector<PointSample> & points,
  const int64_t stampNs,
  const std::string & frameId)
{
  if (!publisher || publisher->get_subscription_count() == 0) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frameId;
  cloud.header.stamp.sec = static_cast<int32_t>(stampNs / 1000000000LL);
  cloud.header.stamp.nanosec = static_cast<uint32_t>(stampNs % 1000000000LL);

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
    4,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iterX(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iterY(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iterZ(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> iterIntensity(cloud, "intensity");
  for (const auto & point : points) {
    *iterX = point.x;
    *iterY = point.y;
    *iterZ = point.z;
    *iterIntensity = static_cast<float>(point.reflectivity);
    ++iterX;
    ++iterY;
    ++iterZ;
    ++iterIntensity;
  }

  publisher->publish(cloud);
}

}  // namespace livox_mid360_gz_sim

GZ_ADD_PLUGIN(
  livox_mid360_gz_sim::LivoxMid360GzPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
