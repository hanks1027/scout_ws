#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <unordered_set>
#include <mutex>
#include <cmath>

class ArmBoundingBoxNode : public rclcpp::Node {
public:
  ArmBoundingBoxNode() : Node("auto_arm_bounding_box_node") {
    this->declare_parameter<std::string>("target_frame", "base_link");
    // 将膨胀边距默认值从 0.5m 修改为 0.09m (9cm)
    this->declare_parameter<double>("margin", 0.09); 

    this->get_parameter("target_frame", target_frame_);
    this->get_parameter("margin", margin_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 监听动态 /tf
    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, std::bind(&ArmBoundingBoxNode::tf_callback, this, std::placeholders::_1));

    // 监听静态 /tf_static
    tf_static_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS(10).transient_local(),
      std::bind(&ArmBoundingBoxNode::tf_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/arm_bounding_box", 10);
    points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/arm_link_points", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ArmBoundingBoxNode::update_and_publish, this));

    RCLCPP_INFO(this->get_logger(), "全量机器人外接矩形节点已启动 [基准坐标系: %s, 膨胀边距: %.2fm]", 
                target_frame_.c_str(), margin_);
  }

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(frames_mutex_);
    for (const auto & transform : msg->transforms) {
      if (!transform.header.frame_id.empty()) {
        known_frames_.insert(transform.header.frame_id);
      }
      if (!transform.child_frame_id.empty()) {
        known_frames_.insert(transform.child_frame_id);
      }
    }
  }

  void update_and_publish() {
    std::unordered_set<std::string> local_frames;
    {
      std::lock_guard<std::mutex> lock(frames_mutex_);
      local_frames = known_frames_;
    }

    if (local_frames.empty()) return;

    std::vector<cv::Point2f> points_2d;
    std::vector<geometry_msgs::msg::Point> points_3d_msg;

    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (const auto & frame_name : local_frames) {
      try {
        auto tf = tf_buffer_->lookupTransform(target_frame_, frame_name, tf2::TimePointZero);
        
        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;
        double z = tf.transform.translation.z;

        points_2d.emplace_back(x, y);

        geometry_msgs::msg::Point pt;
        pt.x = x; pt.y = y; pt.z = z;
        points_3d_msg.push_back(pt);

        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
      } catch (const tf2::TransformException &) {
        continue;
      }
    }

    if (points_2d.size() < 2) return;

    // 1. OpenCV 计算最小外接矩形
    cv::RotatedRect min_rect = cv::minAreaRect(points_2d);

    // 计算实际尺寸（增加 2 * 9cm = 18cm 膨胀量）
    double size_x = std::max(0.05f, min_rect.size.width) + 2.0 * margin_;
    double size_y = std::max(0.05f, min_rect.size.height) + 2.0 * margin_;
    double size_z = std::max(0.05, max_z - min_z) + 2.0 * margin_;

    // 2. 检测尺寸是否发生变化 (变化量 > 0.5毫米 时触发打印)
    const double threshold = 0.0005; // 0.5mm
    if (std::abs(size_x - last_size_x_) > threshold ||
        std::abs(size_y - last_size_y_) > threshold ||
        std::abs(size_z - last_size_z_) > threshold) 
    {
      RCLCPP_INFO(this->get_logger(), 
                  "外接矩形尺寸更新 -> 长(X): %.3f m | 宽(Y): %.3f m | 高(Z): %.3f m", 
                  size_x, size_y, size_z);

      last_size_x_ = size_x;
      last_size_y_ = size_y;
      last_size_z_ = size_z;
    }

    // 3. 构建 Marker 消息
    visualization_msgs::msg::Marker box_marker;
    box_marker.header.frame_id = target_frame_;
    box_marker.header.stamp = this->now();
    box_marker.ns = "bounding_box";
    box_marker.id = 0;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.action = visualization_msgs::msg::Marker::ADD;

    box_marker.pose.position.x = min_rect.center.x;
    box_marker.pose.position.y = min_rect.center.y;
    box_marker.pose.position.z = (min_z == std::numeric_limits<double>::max()) ? 0.5 : (min_z + max_z) / 2.0;

    double angle_rad = min_rect.angle * M_PI / 180.0;
    box_marker.pose.orientation.x = 0.0;
    box_marker.pose.orientation.y = 0.0;
    box_marker.pose.orientation.z = sin(angle_rad / 2.0);
    box_marker.pose.orientation.w = cos(angle_rad / 2.0);

    box_marker.scale.x = size_x;
    box_marker.scale.y = size_y;
    box_marker.scale.z = size_z;

    box_marker.color.r = 0.0f;
    box_marker.color.g = 0.8f;
    box_marker.color.b = 1.0f;
    box_marker.color.a = 0.35f;

    marker_pub_->publish(box_marker);

    // 4. 构建点阵 Marker
    visualization_msgs::msg::Marker pts_marker;
    pts_marker.header = box_marker.header;
    pts_marker.ns = "link_points";
    pts_marker.id = 1;
    pts_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    pts_marker.action = visualization_msgs::msg::Marker::ADD;
    pts_marker.scale.x = 0.04;
    pts_marker.scale.y = 0.04;
    pts_marker.scale.z = 0.04;
    pts_marker.color.r = 1.0f;
    pts_marker.color.g = 0.0f;
    pts_marker.color.b = 0.0f;
    pts_marker.color.a = 0.8f;
    pts_marker.points = points_3d_msg;

    points_pub_->publish(pts_marker);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr points_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string target_frame_;
  double margin_{0.09}; // 9cm

  // 用于记录上一次的尺寸，防刷屏
  double last_size_x_{0.0};
  double last_size_y_{0.0};
  double last_size_z_{0.0};

  std::unordered_set<std::string> known_frames_;
  std::mutex frames_mutex_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmBoundingBoxNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}