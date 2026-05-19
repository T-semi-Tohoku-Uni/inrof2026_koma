#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace koma
{
class JointManager : public rclcpp::Node
{
public:
  JointManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_joint_state_sub_;

  void gazeboJointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
};
}  // namespace koma
