#include <tf2/utils.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <inrof2026_koma_type/action/pursuit.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace koma
{

typedef struct MotorVel
{
  float v1;
  float v2;
  float v3;
} MotorVel;

class PIDController
{
public:
  PIDController() = default;
  PIDController(
    double Kp, double Ki, double Kd, double dt,
    std::function<double(double)> normalize_func = nullptr);
  double compute(double setpoint, double measured_value);

private:
  double Kp_;
  double Ki_;
  double Kd_;
  double dt_;
  double prev_error_;
  double integral_;
  std::function<double(double)> normalize_func_;
};

class Pursuit : public rclcpp::Node
{
public:
  Pursuit(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // robot parameter
  double r_;  // distance from center to wheel

  //PID control
  PIDController linear_PID_tan_, linear_PID_norm_, omega_PID_;

  // control parameters
  geometry_msgs::msg::Pose robot_pose_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  double lookahead_distance_;
  double max_linear_speed_;
  double max_theta_speed_;
  double max_linear_tolerance_;
  double max_reaching_distance_;
  double max_theta_tolerance_;
  double max_reaching_theta_;
  int current_waypoint_index_;
  void control_loop();
  void publish_zero_velocity();
  koma::MotorVel forwardKinematics(float vx, float vy, float vtheta);
  geometry_msgs::msg::Twist inverseKinematics(float v1, float v2, float v3);
  geometry_msgs::msg::Twist clip(geometry_msgs::msg::Twist cmd);

  // subscriber
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_sub_;
  void path_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void robot_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;  // --- IGNORE ---

  // action server
  rclcpp_action::Server<inrof2026_koma_type::action::Pursuit>::SharedPtr pursuit_action_server_;
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const inrof2026_koma_type::action::Pursuit::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::Pursuit>>
      goal_handle);
  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::Pursuit>>
      goal_handle);
  std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::Pursuit>>
    goal_handle_;
};
}  // namespace koma