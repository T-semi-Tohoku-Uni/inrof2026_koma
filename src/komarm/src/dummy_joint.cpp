#include <komarm/joint_manager.hpp>

koma::JointManager::JointManager(const rclcpp::NodeOptions & options) : Node("joint_manager")
{
  // declare parameter
  target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
  source_frame_ = this->declare_parameter<std::string>("source_frame", "hand_v17_1");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  hand_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("hand_pose", 10);
  gazebo_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/komarm/gazebo_joint_states", 10,
    std::bind(&koma::JointManager::gazeboJointStatesCallback, this, std::placeholders::_1));

  hand_pose_timer_ =
    this->create_wall_timer(50ms, std::bind(&koma::JointManager::handPoseCallback, this));

  RCLCPP_INFO(this->get_logger(), "Success initialize JointManager");
}

void koma::JointManager::handPoseCallback()
{
  try {
    geometry_msgs::msg::TransformStamped transform =
      tf_buffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);

    geometry_msgs::msg::PoseStamped hand_pose;
    hand_pose.header.stamp = this->get_clock()->now();
    hand_pose.header.frame_id = target_frame_;
    hand_pose.pose.position.x = transform.transform.translation.x;
    hand_pose.pose.position.y = transform.transform.translation.y;
    hand_pose.pose.position.z = transform.transform.translation.z;
    hand_pose.pose.orientation = transform.transform.rotation;
    hand_pose_pub_->publish(hand_pose);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform %s to %s: %s", target_frame_.c_str(),
      source_frame_.c_str(), ex.what());
  }
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