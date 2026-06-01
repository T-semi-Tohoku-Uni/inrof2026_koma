#pragma once
#include <inrof2026_koma_type/action/pursuit.hpp>
#include <inrof2026_koma_type/srv/ball_position.hpp>
#include <inrof2026_koma_type/srv/pose_stamped.hpp>
#include <inrof2026_koma_type/action/pursuit.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace koma
{
class BTNode : public rclcpp::Node
{
public:
  BTNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool is_pursuit_runing() const;

  // service setter
  void path_waypoint_position(double x, double y);
  void path_goal_position(double x, double y, double theta);

  void target_ball_position(double x, double y);

  // service getter
  inrof2026_koma_type::srv::PoseStamped::Response current_robot_position();
  inrof2026_koma_type::srv::BallPosition::Response closest_ball_position();
  // TODO: ball color

  // action
  void start_path_pursuit();
  void pursuit_goal_response_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle);
  void pursuit_feedback_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle,
    const std::shared_ptr<const inrof2026_koma_type::action::Pursuit::Feedback> feedback);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::WrappedResult
      result);

private:
  std::atomic_bool is_pursuit_running_{false};

  // service
  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::SharedPtr path_waypoint_position_srv_;
  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::SharedPtr path_goal_position_srv_;

  // action
  rclcpp_action::Client<inrof2026_koma_type::action::Pursuit>::SharedPtr pursuit_act_;
};
}  // namespace koma