#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

using namespace std::chrono_literals;

namespace koma
{
class HandPose : public rclcpp::Node
{
public:
  HandPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::TimerBase::SharedPtr hand_pose_timer_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hand_pose_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string target_frame_;
  std::string source_frame_;
  void handPoseCallback();
};
}  // namespace koma