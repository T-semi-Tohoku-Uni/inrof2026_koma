#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

using namespace std::chrono_literals;

namespace koma
{
class JointManager : public rclcpp::Node
{
public:
  JointManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr hand_pose_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_joint_state_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::TimerBase::SharedPtr hand_pose_timer_;
  std::string target_frame_;
  std::string source_frame_;

  void gazeboJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void handPoseCallback();
};
}  // namespace koma
