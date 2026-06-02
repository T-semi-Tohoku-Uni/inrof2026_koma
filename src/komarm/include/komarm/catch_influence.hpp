#pragma once
#include <torch/script.h>
#include <torch/torch.h>

#include <chrono>
#include <inrof2026_koma_type/srv/pose_stamped.hpp>
#include <inrof2026_koma_type/action/arm_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace koma
{
class CatchInfluence : public rclcpp::Node
{
public:
  CatchInfluence(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  //variables
  torch::jit::script::Module module_;
  std::string model_path_;
  sensor_msgs::msg::JointState cur_joint_state_;
  bool has_cur_joint_state_ = false;
  sensor_msgs::msg::JointState pre_action_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // current arm states
  geometry_msgs::msg::Pose cur_gripper_position_;
  
  // target
  geometry_msgs::msg::Pose target_ball_position_;
  sensor_msgs::msg::JointState target_joint_state_;

  // arm settings
  std::vector<double> default_position_;
  std::vector<std::string> joint_names_;
  std::string end_effector_link_;
  std::string base_link_;
  geometry_msgs::msg::PointStamped offset_point_;
  double reach_th_;

  // for transform
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //functions
  torch::jit::script::Module load_model(const std::string & model_path);
  torch::Tensor inference(const torch::Tensor & obs);
  void control_loop();

  //publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_joint_pub_;

  //subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr joint_command_send_timer_;
  void joint_command_send_callback();

  //action server
  rclcpp_action::Server<inrof2026_koma_type::action::ArmControl>::SharedPtr arm_control_act_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const inrof2026_koma_type::action::ArmControl::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::ArmControl>>
      goal_handle);
  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::ArmControl>>
      goal_handle);
  std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::ArmControl>>
    goal_handle_;

  //callback function
  void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace koma