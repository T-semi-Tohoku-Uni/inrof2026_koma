#include <komarm/joint_manager.hpp>

koma::JointManager::JointManager(const rclcpp::NodeOptions & options) : Node("joint_manager")
{
  // Connect between ros node
  joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  target_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_command", rclcpp::SensorDataQoS(),
    std::bind(&koma::JointManager::targetJointStatesCallback, this, std::placeholders::_1));

  // Connect between gazebo
  gazebo_rev_1_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/komarm/revolute_1/cmd_pos",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_2_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/komarm/revolute_2/cmd_pos",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_3_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/komarm/revolute_3/cmd_pos",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_4_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/komarm/revolute_4/cmd_pos",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_5_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/komarm/revolute_5/cmd_pos",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_rev_6_pub_ = this->create_publisher<std_msgs::msg::Float64>(
    "/komarm/revolute_6/cmd_pos",
    rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  gazebo_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&koma::JointManager::gazeboJointStatesCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Success initialize JointManager");
}

void koma::JointManager::targetJointStatesCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std_msgs::msg::Float64 revolute_1;
  revolute_1.data = msg->position[0];
  this->gazebo_rev_1_pub_->publish(revolute_1);

  std_msgs::msg::Float64 revolute_2;
  revolute_2.data = msg->position[1];
  this->gazebo_rev_2_pub_->publish(revolute_2);

  std_msgs::msg::Float64 revolute_3;
  revolute_3.data = msg->position[2];
  this->gazebo_rev_3_pub_->publish(revolute_3);

  std_msgs::msg::Float64 revolute_4;
  revolute_4.data = msg->position[3];
  this->gazebo_rev_4_pub_->publish(revolute_4);

  std_msgs::msg::Float64 revolute_5;
  revolute_5.data = msg->position[4];
  this->gazebo_rev_5_pub_->publish(revolute_5);

  std_msgs::msg::Float64 revolute_6;
  revolute_6.data = msg->position[5];
  this->gazebo_rev_6_pub_->publish(revolute_6);
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