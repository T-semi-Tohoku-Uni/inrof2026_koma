#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

namespace koma
{
class JointManager : public rclcpp::Node
{
public:
  JointManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Connect between ros node
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_sub_;

  // Connect between gazebo
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gazebo_rev_12_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gazebo_rev_11_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gazebo_rev_7_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gazebo_rev_8_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gazebo_rev_9_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_joint_state_sub_;

  void gazeboJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void targetJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};
}  // namespace koma
