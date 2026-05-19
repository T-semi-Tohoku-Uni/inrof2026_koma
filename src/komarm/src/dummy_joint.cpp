#include <komarm/joint_manager.hpp>

koma::JointManager::JointManager(const rclcpp::NodeOptions & options) : Node("joint_manager")
{
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  gazebo_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/komarm/gazebo_joint_states", 10,
    std::bind(&koma::JointManager::gazeboJointStatesCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Success initialize JointManager");
}

void koma::JointManager::gazeboJointStatesCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  this->joint_state_pub_->publish(*msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<koma::JointManager> node = std::make_shared<koma::JointManager>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}