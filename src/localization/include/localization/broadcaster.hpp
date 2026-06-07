#pragma once
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OdomTfBroadcasterNode : public rclcpp::Node
{
public:
  OdomTfBroadcasterNode();

private:
  void position_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  std::string parent_frame_;
  std::string child_frame_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};