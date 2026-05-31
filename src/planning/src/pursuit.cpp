#include <planning/pursuit.hpp>

koma::PIDController::PIDController(
  double Kp, double Ki, double Kd, double dt, std::function<double(double)> normalize_func)
: Kp_(Kp),
  Ki_(Ki),
  Kd_(Kd),
  prev_error_(0.0),
  integral_(0.0),
  dt_(dt),
  normalize_func_(normalize_func)
{
}

double koma::PIDController::compute(double setpoint, double measured_value)
{
  double error = setpoint - measured_value;
  if (normalize_func_) {
    error = normalize_func_(error);
  }
  integral_ += error * dt_;
  double derivative = (error - prev_error_) / dt_;
  prev_error_ = error;
  return Kp_ * error + Ki_ * integral_ + Kd_ * derivative;
}

koma::Pursuit::Pursuit(const rclcpp::NodeOptions & options) : Node("pursuit", options)
{
  // declare parameters
  double dt = 0.1;  // TODO: change to parameter

  // robot parameter
  r_ = this->declare_parameter<double>("r", 0.125);

  // tangent PID parameters
  double Kp_tan = this->declare_parameter<double>("Kp_tan", 0.80);
  double Ki_tan = this->declare_parameter<double>("Ki_tan", 0.00);
  double Kd_tan = this->declare_parameter<double>("Kd_tan", 0.00);

  // normal PID parameters
  double Kp_norm = this->declare_parameter<double>("Kp_norm", 0.80);
  double Ki_norm = this->declare_parameter<double>("Ki_norm", 0.00);
  double Kd_norm = this->declare_parameter<double>("Kd_norm", 0.00);

  // angular PID parameters
  double Kp_theta = this->declare_parameter<double>("Kp_theta", 0.40);
  double Ki_theta = this->declare_parameter<double>("Ki_theta", 0.00);
  double Kd_theta = this->declare_parameter<double>("Kd_theta", 0.00);

  // control parameters
  lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 0.05);
  max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 0.2);
  max_theta_speed_ = this->declare_parameter<double>("max_theta_speed", 2.0);
  max_linear_tolerance_ = this->declare_parameter<double>("max_linear_tolerance", 0.08);
  max_reaching_distance_ = this->declare_parameter<double>("max_reaching_distance", 0.02);
  max_theta_tolerance_ = this->declare_parameter<double>("max_theta_tolerance", 0.3);
  max_reaching_theta_ = this->declare_parameter<double>("max_reaching_theta", 0.1);

  // initialize PID controllers
  linear_PID_tan_ = koma::PIDController(Kp_tan, Ki_tan, Kd_tan, dt);
  linear_PID_norm_ = koma::PIDController(Kp_norm, Ki_norm, Kd_norm, dt);
  omega_PID_ = koma::PIDController(Kp_theta, Ki_theta, Kd_theta, dt, [](double e) {
    // normalize angle error to [-pi, pi]
    while (e > M_PI) e -= 2 * M_PI;
    while (e < -M_PI) e += 2 * M_PI;
    return e;
  });

  // subscriber
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&koma::Pursuit::path_callback, this, std::placeholders::_1));
  robot_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&koma::Pursuit::robot_pose_callback, this, std::placeholders::_1));

  // publisher
  twist_command_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "twist_command", rclcpp::QoS(rclcpp::KeepLast(10)));

  // action server
  pursuit_action_server_ = rclcpp_action::create_server<inrof2026_koma_type::action::Pursuit>(
    this, "pursuit_command",
    std::bind(&koma::Pursuit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&koma::Pursuit::handle_cancel, this, std::placeholders::_1),
    std::bind(&koma::Pursuit::handle_accepted, this, std::placeholders::_1));

  control_timer_ = this->create_wall_timer(100ms, std::bind(&koma::Pursuit::control_loop, this));
}

rclcpp_action::GoalResponse koma::Pursuit::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const inrof2026_koma_type::action::Pursuit::Goal> goal)
{
  if (goal_handle_) {
    RCLCPP_WARN(
      this->get_logger(),
      "Received a new goal request, but there is already an active goal. Rejecting.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Received a new goal request. Accepting.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse koma::Pursuit::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::Pursuit>>
    goal_handle)
{
  goal_handle_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void koma::Pursuit::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::Pursuit>>
    goal_handle)
{
  goal_handle_ = goal_handle;
}

void koma::Pursuit::path_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  path_ = msg->poses;
  current_waypoint_index_ = 0;
}

void koma::Pursuit::robot_pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_pose_ = msg->pose.pose;
}

void koma::Pursuit::control_loop()
{
  if (!goal_handle_ || goal_handle_->is_canceling()) {
    publish_zero_velocity();
    return;
  }
  if (path_.empty()) {
    publish_zero_velocity();
    return;
  }

  // publish goal position
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = path_[current_waypoint_index_].pose.position.x;
  target_pose.position.y = path_[current_waypoint_index_].pose.position.y;

  // target_pub_ ->publish(target_pose);

  //error calculation linear
  double dx = path_[current_waypoint_index_].pose.position.x - robot_pose_.position.x;
  double dy = path_[current_waypoint_index_].pose.position.y - robot_pose_.position.y;
  double tx = path_[current_waypoint_index_ + 1].pose.position.x -
              path_[current_waypoint_index_].pose.position.x;
  double ty = path_[current_waypoint_index_ + 1].pose.position.y -
              path_[current_waypoint_index_].pose.position.y;
  double norm = std::hypot(tx, ty);
  if (norm > 0) {
    tx /= norm;
    ty /= norm;
  }
  double nx = -ty;
  double ny = tx;
  double error_tan = dx * tx + dy * ty;
  double error_norm = dx * nx + dy * ny;
  double linear_error = std::hypot(dx, dy);
  double linear_goal_x = path_[path_.size() - 1].pose.position.x - robot_pose_.position.x;
  double linear_goal_y = path_[path_.size() - 1].pose.position.y - robot_pose_.position.y;
  double linear_goal_distance = std::hypot(linear_goal_x, linear_goal_y);

  //quoternion to yaw
  tf2::Quaternion q(
    path_[current_waypoint_index_].pose.orientation.x,
    path_[current_waypoint_index_].pose.orientation.y,
    path_[current_waypoint_index_].pose.orientation.z,
    path_[current_waypoint_index_].pose.orientation.w);

  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  double theta_error = yaw - tf2::getYaw(robot_pose_.orientation);
  while (theta_error > M_PI) theta_error -= 2 * M_PI;
  while (theta_error < -M_PI) theta_error += 2 * M_PI;

  tf2::Quaternion q_goal(
    path_[path_.size() - 1].pose.orientation.x, path_[path_.size() - 1].pose.orientation.y,
    path_[path_.size() - 1].pose.orientation.z, path_[path_.size() - 1].pose.orientation.w);

  double roll_goal, pitch_goal, yaw_goal;
  tf2::Matrix3x3(q_goal).getRPY(roll_goal, pitch_goal, yaw_goal);

  double theta_goal = yaw_goal - tf2::getYaw(robot_pose_.orientation);
  while (theta_goal >= M_PI) theta_goal -= M_PI;
  while (theta_goal < -M_PI) theta_goal += M_PI;

  //error calculation theta
  double target_theta = yaw;
  // printWayPointArrow(path_[current_waypoint_index_].pose, path_[path_.size()-1].pose);

  // // --- ★ここから修正版：RVizに表示するためのMarker publish ---
  // visualization_msgs::msg::Marker marker;
  // marker.header.frame_id = "map"; // TFに合わせる
  // marker.header.stamp = this->get_clock()->now();
  // marker.ns = "waypoint_marker";
  // marker.id = 0; // ← 常に同じIDを使うことで「上書き表示」できる！
  // marker.type = visualization_msgs::msg::Marker::SPHERE;
  // marker.action = visualization_msgs::msg::Marker::ADD;

  // marker.pose.position.x = path_[current_waypoint_index_].pose.position.x;
  // marker.pose.position.y = path_[current_waypoint_index_].pose.position.y;
  // marker.pose.position.z = 0.0;
  // marker.pose.orientation.w = 1.0;

  // // 点の大きさ
  // marker.scale.x = 0.15;
  // marker.scale.y = 0.15;
  // marker.scale.z = 0.15;

  // // 色：赤
  // marker.color.r = 1.0;
  // marker.color.g = 0.0;
  // marker.color.b = 0.0;
  // marker.color.a = 1.0;

  // // lifetimeを少し短くして上書き更新を確実にする
  // marker.lifetime = rclcpp::Duration::from_seconds(0.2);

  // marker_pub_->publish(marker);
  // // --- ★ここまで修正版 ---

  while (max_linear_tolerance_ > linear_error) {
    if (current_waypoint_index_ + 1 >= static_cast<int>(path_.size())) break;

    current_waypoint_index_++;
    linear_error = std::hypot(
      path_[current_waypoint_index_].pose.position.x - robot_pose_.position.x,
      path_[current_waypoint_index_].pose.position.y - robot_pose_.position.y);
  }

  if ((linear_goal_distance < max_reaching_distance_)) {  //&& theta_goal < max_reaching_theta) {
    //goal reached
    RCLCPP_INFO(this->get_logger(), "Goal reached.");
    publish_zero_velocity();
    auto result_msg = std::make_shared<inrof2026_koma_type::action::Pursuit::Result>();
    result_msg->success = true;
    goal_handle_->succeed(result_msg);
    goal_handle_.reset();
    return;
  }

  //PID control for linear speed
  double linear_cmd_tan = linear_PID_tan_.compute(error_tan, 0.0);
  double linear_cmd_norm = linear_PID_norm_.compute(error_norm, 0.0);

  //PID control for theta speed
  double theta_speed_cmd = omega_PID_.compute(target_theta, tf2::getYaw(robot_pose_.orientation));

  //convert to x,y speed
  double linear_speed_cmd_x = linear_cmd_tan * tx + linear_cmd_norm * nx;
  double linear_speed_cmd_y = linear_cmd_tan * ty + linear_cmd_norm * ny;

  geometry_msgs::msg::Twist linear_speed;
  double pose_theta = tf2::getYaw(robot_pose_.orientation);
  linear_speed.linear.x =
    cos(pose_theta) * linear_speed_cmd_x + sin(pose_theta) * linear_speed_cmd_y;
  linear_speed.linear.y =
    -sin(pose_theta) * linear_speed_cmd_x + cos(pose_theta) * linear_speed_cmd_y;
  linear_speed.angular.z = theta_speed_cmd;

  //apply speed limits
  geometry_msgs::msg::Twist clipped_v = clip(linear_speed);
  twist_command_pub_->publish(clipped_v);

  double clipped_v_x_r = clipped_v.linear.x;
  double clipped_v_y_r = clipped_v.linear.y;

  double clipped_v_x_f = cos(pose_theta) * clipped_v_x_r - sin(pose_theta) * clipped_v_y_r;
  double clipped_v_y_f = sin(pose_theta) * clipped_v_x_r + cos(pose_theta) * clipped_v_y_r;

  // printCmdVelArrow(linear_speed_cmd_x, linear_speed_cmd_y, clipped_v_x_f, clipped_v_y_f);

  //publish feedback
  auto feedback_msg = std::make_shared<inrof2026_koma_type::action::Pursuit::Feedback>();
  feedback_msg->pose_stamped.pose.position = robot_pose_.position;
  feedback_msg->pose_stamped.pose.orientation = robot_pose_.orientation;
  feedback_msg->pose_stamped.header.stamp = this->get_clock()->now();
  feedback_msg->pose_stamped.header.frame_id = "map";
  goal_handle_->publish_feedback(feedback_msg);
}

koma::MotorVel koma::Pursuit::forwardKinematics(float vx, float vy, float vtheta)
{
  MotorVel motor_vel;
  // motor_vel.v1 = vx + r_*vtheta;
  // motor_vel.v2 = 0.5 * vx + std::sqrt(3)/2*vy - r_*vtheta;
  // motor_vel.v3 = -0.5 * vx + std::sqrt(3)/2*vy + r_*vtheta;
  motor_vel.v1 = (-vy) + r_ * vtheta;
  motor_vel.v2 = 0.5 * (-vy) + std::sqrt(3) / 2 * vx - r_ * vtheta;
  motor_vel.v3 = -0.5 * (-vy) + std::sqrt(3) / 2 * vx + r_ * vtheta;
  return motor_vel;
}

geometry_msgs::msg::Twist koma::Pursuit::inverseKinematics(float v1, float v2, float v3)
{
  geometry_msgs::msg::Twist twist;
  // twist.linear.y = -((2.0/3.0)*v1 + (1.0/3.0)*v2 - (1.0/3.0)*v3);
  // twist.linear.x = (1.0/std::sqrt(3))*v2 + (1.0/std::sqrt(3))*v3;
  // twist.angular.z = (1.0/3.0/r_)*v1 - (1.0/3.0/r_)*v2 + (1.0/3.0/r_)*v3;
  twist.linear.y = -((2.0 / 3.0) * v1 + (1.0 / 3.0) * v2 - (1.0 / 3.0) * v3);
  twist.linear.x = (1.0 / std::sqrt(3)) * v2 + (1.0 / std::sqrt(3)) * v3;
  twist.angular.z = (1.0 / 3.0 / r_) * v1 - (1.0 / 3.0 / r_) * v2 + (1.0 / 3.0 / r_) * v3;
  return twist;
}

geometry_msgs::msg::Twist koma::Pursuit::clip(geometry_msgs::msg::Twist cmd)
{
  double linear_speed_cmd_x;
  double linear_speed_cmd_y;
  double theta_speed_cmd;
  linear_speed_cmd_x = cmd.linear.x;
  linear_speed_cmd_y = cmd.linear.y;
  theta_speed_cmd = cmd.angular.z;

  MotorVel v_motor = forwardKinematics(linear_speed_cmd_x, linear_speed_cmd_y, theta_speed_cmd);
  double v1 = v_motor.v1;
  double v2 = v_motor.v2;
  double v3 = v_motor.v3;

  double v_max = std::max({std::abs(v1), std::abs(v2), std::abs(v3)});

  if (v_max > max_linear_speed_) {
    double scale = max_linear_speed_ / v_max;
    v1 *= scale;
    v2 *= scale;
    v3 *= scale;
  }

  return inverseKinematics(v1, v2, v3);
}

void koma::Pursuit::publish_zero_velocity()
{
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.linear.y = 0.0;
  cmd.angular.z = 0.0;
  twist_command_pub_->publish(cmd);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<koma::Pursuit>());
  rclcpp::shutdown();
  return 0;
}