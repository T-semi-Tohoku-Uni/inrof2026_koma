#pragma once
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <inrof2026_koma_type/srv/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace koma
{
class BallPathNode : public rclcpp::Node
{
public:
  BallPathNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subPose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  geometry_msgs::msg::Pose::SharedPtr robot_pose_;
  void genBallPath(
    const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Request> request,
    const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Response> response);

  int num_points_;
  double shorten_;
  double theta_offset_;
  rclcpp::Service<inrof2026_koma_type::srv::PoseStamped>::SharedPtr srv_gen_route_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_arrow_pub_;
};
}  // namespace koma