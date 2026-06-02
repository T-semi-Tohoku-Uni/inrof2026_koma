#include "localization/broadcaster.hpp"
#include <functional>

#include "geometry_msgs/msg/transform_stamped.hpp"

OdomTfBroadcasterNode::OdomTfBroadcasterNode()
: Node("odom_tf_broadcaster")
{
  this->declare_parameter<std::string>("parent_frame", "odom");
  this->declare_parameter<std::string>("child_frame", "base_footprint");

  parent_frame_ = this->get_parameter("parent_frame").as_string();
  child_frame_ = this->get_parameter("child_frame").as_string();

  tf_broadcaster_ =
    std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "pose",
    rclcpp::QoS(10),
    std::bind(&OdomTfBroadcasterNode::position_callback, this, std::placeholders::_1)
  );

  RCLCPP_INFO(this->get_logger(), "OdomTfBroadcasterNode has been started.");
}

void OdomTfBroadcasterNode::position_callback(
  const geometry_msgs::msg::Pose::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped tf_msg;

  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = parent_frame_;
  tf_msg.child_frame_id = child_frame_;

  tf_msg.transform.translation.x = msg->position.x;
  tf_msg.transform.translation.y = msg->position.y;
  tf_msg.transform.translation.z = msg->position.z;
  tf_msg.transform.rotation = msg->orientation;

  tf_broadcaster_->sendTransform(tf_msg);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTfBroadcasterNode>());
  rclcpp::shutdown();
  return 0;
}