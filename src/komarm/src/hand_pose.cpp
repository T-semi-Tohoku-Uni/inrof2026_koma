#include <komarm/hand_pose.hpp>

koma::HandPose::HandPose(const rclcpp::NodeOptions & options) : Node("hand_pose")
{
  // declare parameter
  target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");
  source_frame_ = this->declare_parameter<std::string>("source_frame", "hand_v17_1");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  hand_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("hand_pose", rclcpp::SensorDataQoS());
  hand_pose_timer_ =
    this->create_wall_timer(50ms, std::bind(&koma::HandPose::handPoseCallback, this));
}

void koma::HandPose::handPoseCallback()
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<koma::HandPose> node = std::make_shared<koma::HandPose>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}