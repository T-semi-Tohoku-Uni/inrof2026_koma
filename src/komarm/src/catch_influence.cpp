#include "komarm/catch_influence.hpp"

#include <cmath>

namespace koma
{

//define the constructor of the node
CatchInfluence::CatchInfluence(const rclcpp::NodeOptions & options)
: Node("catch_influence", options)
{
  RCLCPP_INFO(this->get_logger(), "CatchInfluence node has been started.");

  // default position
  default_position_ = this->declare_parameter<std::vector<double>>(
    "default_position", std::vector<double>{0.0, -1.3, 0.0, 1.57, 0.0, 0.0});

  // joint names
  joint_names_ = this->declare_parameter<std::vector<std::string>>(
    "joint_names",
    std::vector<std::string>{
      "Revolute_1", "Revolute_2", "Revolute_3", "Revolute_4", "Revolute_5", "Revolute_6"

    });

  end_effector_link_ = this->declare_parameter<std::string>("end_effector_link", "end_effector");

  base_link_ = this->declare_parameter<std::string>("base_link", "base_link");

  x_reach_th_ = this->declare_parameter<double>("x_reach_th", 0.01);
  y_reach_th_ = this->declare_parameter<double>("y_reach_th", 0.01);
  z_reach_th_ = this->declare_parameter<double>("z_reach_th", 0.05);

  open_command_ = this->declare_parameter<double>("open_command", -0.4);

  close_command_ = this->declare_parameter<double>("close_command", 1.0);

  // initialize tf buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize publishers and subscribers
  target_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "joint_command", rclcpp::QoS(10).reliable());

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    std::bind(&CatchInfluence::joint_callback, this, std::placeholders::_1));

  // Initialize service
  arm_ee_open_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "arm_ee_open",
    std::bind(
      &CatchInfluence::arm_ee_open_callback, this, std::placeholders::_1, std::placeholders::_2));
  arm_ee_close_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "arm_ee_close",
    std::bind(
      &CatchInfluence::arm_ee_close_callback, this, std::placeholders::_1, std::placeholders::_2));

  arm_default_pose_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "arm_default_pose", std::bind(
                          &CatchInfluence::arm_default_pose_callback, this, std::placeholders::_1,
                          std::placeholders::_2));
  
  arm_action_timeout_sec_ = this->declare_parameter<double>("arm_action_timeout_sec", 5.0);

  // Initialize action server
  arm_control_act_ = rclcpp_action::create_server<inrof2026_koma_type::action::ArmControl>(
    this, "arm_command",
    std::bind(
      &koma::CatchInfluence::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&koma::CatchInfluence::handle_cancel, this, std::placeholders::_1),
    std::bind(&koma::CatchInfluence::handle_accepted, this, std::placeholders::_1));

  // Initialize joint states and pre action
  target_joint_state_ = sensor_msgs::msg::JointState();
  target_joint_state_.name = joint_names_;
  target_joint_state_.position = default_position_;

  pre_action_ = sensor_msgs::msg::JointState();
  pre_action_.name = joint_names_;
  pre_action_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Load the model
  const std::string model_path_ = this->declare_parameter<std::string>(
    "model_path", "/home/keigo/inrof2026_koma/src/komarm/weight/policy.pt");

  module_ = load_model(model_path_);
  //inference mode
  module_.eval();

  // timer setting
  // create a timer for the control loop
  control_timer_ = this->create_wall_timer(20ms, std::bind(&CatchInfluence::control_loop, this));
  // create a timer for joint commnad send
  joint_command_send_timer_ = this->create_wall_timer(
    10ms, std::bind(&koma::CatchInfluence::joint_command_send_callback, this));

  RCLCPP_INFO(this->get_logger(), "Model loaded successfully.");
}

void koma::CatchInfluence::arm_ee_open_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  target_joint_state_.position[5] = open_command_;
  response->success = true;
}

void koma::CatchInfluence::arm_ee_close_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  target_joint_state_.position[5] = close_command_;
  response->success = true;
}

void koma::CatchInfluence::arm_default_pose_callback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  target_joint_state_.position[0] = default_position_[0];
  target_joint_state_.position[1] = default_position_[1];
  target_joint_state_.position[2] = default_position_[2];
  target_joint_state_.position[3] = default_position_[3];
  target_joint_state_.position[4] = default_position_[4];
  response->success = true;
}

void koma::CatchInfluence::joint_command_send_callback()
{
  target_joint_pub_->publish(target_joint_state_);
}

void CatchInfluence::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  cur_joint_state_ = *msg;
  has_cur_joint_state_ = true;
}

rclcpp_action::GoalResponse koma::CatchInfluence::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const inrof2026_koma_type::action::ArmControl::Goal> goal)
{
  (void)uuid;
  if (goal_handle_) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "Received a new goal request. Accepting.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse koma::CatchInfluence::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::ArmControl>>
    goal_handle)
{
  goal_handle_.reset();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void koma::CatchInfluence::handle_accepted(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<inrof2026_koma_type::action::ArmControl>>
    goal_handle)
{
  goal_handle_ = goal_handle;
  arm_goal_start_time_ = this->get_clock()->now();
  pre_action_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // target ball position based odom frame. convert position based to base_link
  geometry_msgs::msg::PoseStamped target_ball_pose_odom_frame;
  target_ball_pose_odom_frame.header.frame_id = "odom";
  target_ball_pose_odom_frame.header.stamp = this->get_clock()->now();
  target_ball_pose_odom_frame.pose = goal_handle->get_goal()->target_hand_position.pose;

  geometry_msgs::msg::PoseStamped pose_base_link;
  try {
    pose_base_link =
      tf_buffer_->transform(target_ball_pose_odom_frame, "base_link", tf2::durationFromSec(0.1));
    target_ball_position_ = pose_base_link.pose;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform odom to base_link failed: %s", ex.what());
    return;
  }
}

//define the function that loads the parameters of the model
torch::jit::script::Module CatchInfluence::load_model(const std::string & model_path)
{
  torch::jit::script::Module module;

  try {
    module = torch::jit::load(model_path);
  } catch (const c10::Error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("CatchInfluence"), "Error loading the model: %s", e.what());
    throw;
  }

  return module;
}

//define the function that inferes the action from the observation
torch::Tensor CatchInfluence::inference(const torch::Tensor & obs)
{
  // Disable gradient calculation for inference
  torch::NoGradGuard no_grad;

  // forward the observation
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(obs);
  torch::Tensor action = module_.forward(inputs).toTensor();

  return action;
}

//main control loop
void CatchInfluence::control_loop()
{
  //create states
  //influence
  //publish action
  //pre action = action

  if (!goal_handle_) return;

  rclcpp::Duration elapsed = this->get_clock()->now() - arm_goal_start_time_;
  if (elapsed.seconds() >= arm_action_timeout_sec_) {
    RCLCPP_WARN(this->get_logger(), "Arm action timeout. Forcing success.");

    std::shared_ptr<inrof2026_koma_type::action::ArmControl::Result> result_msg = std::make_shared<inrof2026_koma_type::action::ArmControl::Result>();
    result_msg->success = true;

    goal_handle_->succeed(result_msg);
    goal_handle_.reset();
    return;
  }

  if (!has_cur_joint_state_) {
    RCLCPP_WARN(this->get_logger(), "cur_joint_state is empty");
    return;
  }

  // transform hand position from joint states and feedback
  try {
    geometry_msgs::msg::TransformStamped tf =
      tf_buffer_->lookupTransform("base_link", end_effector_link_, tf2::TimePointZero);

    cur_gripper_position_.position.x = tf.transform.translation.x;
    cur_gripper_position_.position.y = tf.transform.translation.y;
    cur_gripper_position_.position.z = tf.transform.translation.z;
    cur_gripper_position_.orientation = tf.transform.rotation;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  // judge hand position
  if (
    std::abs(cur_gripper_position_.position.x - target_ball_position_.position.x) < x_reach_th_ &&
    std::abs(cur_gripper_position_.position.y - target_ball_position_.position.y) < y_reach_th_ &&
    std::abs(cur_gripper_position_.position.z - target_ball_position_.position.z) < z_reach_th_) {
    auto result_msg = std::make_shared<inrof2026_koma_type::action::ArmControl::Result>();
    result_msg->success = true;
    goal_handle_->succeed(result_msg);
    goal_handle_.reset();
    return;
  }

  // publish feedback
  std::shared_ptr<inrof2026_koma_type::action::ArmControl_Feedback> feed_back =
    std::make_shared<inrof2026_koma_type::action::ArmControl::Feedback>();
  feed_back->current_hand_position.pose = cur_gripper_position_;
  feed_back->current_hand_position.header.frame_id = base_link_;
  feed_back->current_hand_position.header.stamp = this->get_clock()->now();
  goal_handle_->publish_feedback(feed_back);

  RCLCPP_INFO(
    this->get_logger(), "%lf %lf %lf %lf %lf %lf", target_ball_position_.position.x,
    target_ball_position_.position.y, target_ball_position_.position.z,
    cur_gripper_position_.position.x, cur_gripper_position_.position.y,
    cur_gripper_position_.position.z);

  //create states
  torch::Tensor obs = torch::tensor(
                        {
                          /* joint position */
                          cur_joint_state_.position[0] - default_position_[0],
                          cur_joint_state_.position[1] - default_position_[1],
                          cur_joint_state_.position[2] - default_position_[2],
                          cur_joint_state_.position[3] - default_position_[3],
                          cur_joint_state_.position[4] - default_position_[4],
                          cur_joint_state_.position[5] - default_position_[5],
                          // cur_joint_state_.position[5] - default_position_[5],

                          // /* joint vel */
                          cur_joint_state_.velocity[0],
                          cur_joint_state_.velocity[1],
                          cur_joint_state_.velocity[2],
                          cur_joint_state_.velocity[3],
                          cur_joint_state_.velocity[4],
                          cur_joint_state_.velocity[5],

                          // /* target arm position */
                          target_ball_position_.position.x,
                          target_ball_position_.position.y,

                          /* pre action */
                          pre_action_.position[0],
                          pre_action_.position[1],
                          pre_action_.position[2],
                          pre_action_.position[3],
                          pre_action_.position[4],
                          pre_action_.position[5],
                        },
                        torch::TensorOptions().dtype(torch::kFloat32))
                        .unsqueeze(0);

  //influence
  torch::Tensor action = inference(obs).squeeze(0);

  // post compute
  target_joint_state_.header.stamp = this->get_clock()->now();
  // TODO
  for (size_t i = 0; i < target_joint_state_.name.size(); i++) {
    double raw;
    raw = action[i].item<double>();

    if (i == 5) {
      // TODO: hard code for gripper open/close
      target_joint_state_.position[i] = open_command_;
    } else {
      target_joint_state_.position[i] = 0.5 * raw + default_position_[i];
    }
    pre_action_.position[i] = raw;
  }
}
}  // namespace koma

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<koma::CatchInfluence>());
  rclcpp::shutdown();
  return 0;
}
