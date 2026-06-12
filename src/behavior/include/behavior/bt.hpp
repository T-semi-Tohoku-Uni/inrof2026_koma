#pragma once
#include <inrof2026_koma_type/action/arm_control.hpp>
#include <inrof2026_koma_type/action/pursuit.hpp>
#include <inrof2026_koma_type/srv/ball_position.hpp>
#include <inrof2026_koma_type/srv/color.hpp>
#include <inrof2026_koma_type/srv/pose_stamped.hpp>
#include <inrof2026_koma_type/srv/set_float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace koma
{
class BTNode : public rclcpp::Node
{
public:
  BTNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  bool is_pursuit_running() const;
  bool is_arm_control_running() const;

  // service setter
  void path_waypoint_position(double x, double y);
  void path_goal_position(double x, double y, double theta);
  void path_ball_position(double x, double y);
  void arm_ee_open();
  void arm_ee_close();
  void arm_default_pose();
  void arm_pursuit_pose();
  void arm_root_pose(double theta);

  std::optional<inrof2026_koma_type::srv::BallPosition::Response> target_ball_position();

  // service getter
  inrof2026_koma_type::srv::PoseStamped::Response current_robot_position();
  inrof2026_koma_type::srv::BallPosition::Response closest_ball_position();
  inrof2026_koma_type::srv::Color::Response color();
  // TODO: ball color

  // action
  // pursuit
  void start_path_pursuit();
  void pursuit_goal_response_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle);
  void pursuit_feedback_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle,
    const std::shared_ptr<const inrof2026_koma_type::action::Pursuit::Feedback> feedback);
  void result_callback(
    const rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::WrappedResult
      result);
  // arm control
  void start_arm_control(double x, double y, double z);
  void arm_goal_response_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::ArmControl>::SharedPtr
      goal_handle);
  void arm_feedback_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::ArmControl>::SharedPtr goal_handle,
    const std::shared_ptr<const inrof2026_koma_type::action::ArmControl::Feedback> feedback);
  void arm_result_callback(
    const rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::ArmControl>::WrappedResult
      result);

private:
  std::atomic_bool is_pursuit_goal_pending_{false};
  std::atomic_bool is_arm_control_runing_{false};

  // service
  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::SharedPtr path_waypoint_position_srv_;
  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::SharedPtr path_goal_position_srv_;
  rclcpp::Client<inrof2026_koma_type::srv::BallPosition>::SharedPtr target_ball_position_srv_;
  rclcpp::Client<inrof2026_koma_type::srv::Color>::SharedPtr color_srv_;
  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::SharedPtr path_ball_position_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_ee_open_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_ee_close_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_default_pose_srv_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_pursuit_pose_srv_;
  rclcpp::Client<inrof2026_koma_type::srv::SetFloat64>::SharedPtr arm_root_pose_srv_;

  // action
  rclcpp_action::Client<inrof2026_koma_type::action::Pursuit>::SharedPtr pursuit_act_;
  rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr
    pursuit_goal_handle_;
  rclcpp_action::Client<inrof2026_koma_type::action::ArmControl>::SharedPtr arm_control_act_;
};
}  // namespace koma