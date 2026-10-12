#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class FootprintPublisher : public rclcpp::Node
{
public:
    FootprintPublisher() : Node("base_footprint_publisher")
    {
        // 初始化 TF Buffer, Listener 和 Broadcaster
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 创建定时器，以 50Hz (20ms) 频率周期发布 TF
        timer_ = this->create_wall_timer(
            20ms, std::bind(&FootprintPublisher::publish_footprint_tf, this));
    }

private:
    void publish_footprint_tf()
    {
        try
        {
            // 获取 lidar -> body 的 3D 变换
            geometry_msgs::msg::TransformStamped trans = tf_buffer_->lookupTransform(
                "lidar", "base_link", tf2::TimePointZero);

            // 1. 提取位置
            double x = trans.transform.translation.x;
            double y = trans.transform.translation.y;

            // 2. 提取姿态并计算 Yaw，清零 Roll 和 Pitch
            tf2::Quaternion q_3d(
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w);

            double roll, pitch, yaw;
            tf2::Matrix3x3(q_3d).getRPY(roll, pitch, yaw);

            // 仅根据 Yaw 重新构造 2D 四元数
            tf2::Quaternion q_2d;
            q_2d.setRPY(0.0, 0.0, yaw);

            // 3. 构建 odom -> base_footprint 变换
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = "lidar";
            t.child_frame_id = "base_footprint";

            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = 0.0; // 强制置零，贴合地面平面

            t.transform.rotation.x = q_2d.x();
            t.transform.rotation.y = q_2d.y();
            t.transform.rotation.z = q_2d.z();
            t.transform.rotation.w = q_2d.w();

            // 4. 广播 TF
            tf_broadcaster_->sendTransform(t);
        }
        catch (const tf2::TransformException &ex)
        {
            // 在 TF 树尚未准备好时忽略异常
            RCLCPP_DEBUG(this->get_logger(), "Could not transform lidar to body: %s", ex.what());
        }
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FootprintPublisher>());
    rclcpp::shutdown();
    return 0;
}