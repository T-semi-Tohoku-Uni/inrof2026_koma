#include <komarm/joint_manager.hpp>

koma::JointManager::JointManager(const rclcpp::NodeOptions & options) : Node("joint_manager")
{
  // Connect between ros node
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  target_joint_state_sub_ = 
    this->create_subscription<sensor_msgs::msg::JointState>(
      "target_joint_states", rclcpp::SensorDataQoS(),
      std::bind(&koma::JointManager::targetJointStatesCallback, this, std::placeholders::_1)
    );
  
  // Connect between gazebo
  gazebo_rev_12_pub_ = this->create_publisher<std_msgs::msg::Float64>("/komarm/revolute_12/cmd_pos", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_11_pub_ = this->create_publisher<std_msgs::msg::Float64>("/komarm/revolute_11/cmd_pos", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_7_pub_ = this->create_publisher<std_msgs::msg::Float64>("/komarm/revolute_7/cmd_pos", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_8_pub_ = this->create_publisher<std_msgs::msg::Float64>("/komarm/revolute_8/cmd_pos", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_9_pub_ = this->create_publisher<std_msgs::msg::Float64>("/komarm/revolute_9/cmd_pos", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/komarm/gazebo_joint_states", 10,
    std::bind(&koma::JointManager::gazeboJointStatesCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Success initialize JointManager");
}

void koma::JointManager::targetJointStatesCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg
) {
  std_msgs::msg::Float64 revolute_12;
  revolute_12.data = msg->position[0];
  this->gazebo_rev_12_pub_->publish(revolute_12);

  std_msgs::msg::Float64 revolute_11;
  revolute_11.data = msg->position[1];
  this->gazebo_rev_11_pub_->publish(revolute_11);

  std_msgs::msg::Float64 revolute_7;
  revolute_7.data = msg->position[2];
  this->gazebo_rev_7_pub_->publish(revolute_7);

  std_msgs::msg::Float64 revolute_8;
  revolute_8.data = msg->position[3];
  this->gazebo_rev_8_pub_->publish(revolute_8);

  std_msgs::msg::Float64 revolute_9;
  revolute_9.data = msg->position[4];
  this->gazebo_rev_9_pub_->publish(revolute_9);
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