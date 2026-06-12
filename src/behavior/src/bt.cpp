#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behavior/arm_control.hpp>
#include <behavior/arm_default_pose.hpp>
#include <behavior/arm_ee_close.hpp>
#include <behavior/arm_ee_open.hpp>
#include <behavior/arm_root_pose.hpp>
#include <behavior/arm_pursuit_pose.hpp>
#include <behavior/bt.hpp>
#include <behavior/path_ball_position.hpp>
#include <behavior/path_goal_position.hpp>
#include <behavior/path_waypoint_position.hpp>
#include <behavior/pursuit.hpp>
#include <behavior/target_ball_position.hpp>
#include <behavior/while_do_else_break.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

koma::BTNode::BTNode(const rclcpp::NodeOptions & options) : Node("bt_node", options)
{
  // service
  // server: localization/path_plan
  path_waypoint_position_srv_ =
    this->create_client<inrof2026_koma_type::srv::PoseStamped>("waypoint");

  // server: localization/path_plan
  path_goal_position_srv_ = this->create_client<inrof2026_koma_type::srv::PoseStamped>("goal_pose");

  // server: ball_detection/dbscan
  target_ball_position_srv_ =
    this->create_client<inrof2026_koma_type::srv::BallPosition>("target_ball_pose");

  path_ball_position_srv_ = this->create_client<inrof2026_koma_type::srv::PoseStamped>("path_ball");

  // server: komarm/catch_influence
  arm_ee_open_srv_ = this->create_client<std_srvs::srv::Trigger>("arm_ee_open");
  arm_ee_close_srv_ = this->create_client<std_srvs::srv::Trigger>("arm_ee_close");
  arm_default_pose_srv_ = this->create_client<std_srvs::srv::Trigger>("arm_default_pose");
  arm_pursuit_pose_srv_ = this->create_client<std_srvs::srv::Trigger>("arm_pursuit_pose");
  arm_root_pose_srv_ = this->create_client<inrof2026_koma_type::srv::SetFloat64>("arm_root_pose");

  // action
  // server: localization/pursuit
  pursuit_act_ =
    rclcpp_action::create_client<inrof2026_koma_type::action::Pursuit>(this, "pursuit_command");
  // server: komarm/catch_influence
  arm_control_act_ =
    rclcpp_action::create_client<inrof2026_koma_type::action::ArmControl>(this, "arm_command");
}

void koma::BTNode::path_waypoint_position(double x, double y)
{
  while (!path_waypoint_position_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "waypoint is not available. localization/path_plan");
  }

  inrof2026_koma_type::srv::PoseStamped_Request::SharedPtr request =
    std::make_shared<inrof2026_koma_type::srv::PoseStamped::Request>();
  request->pose_stamped.pose.position.x = x;
  request->pose_stamped.pose.position.y = y;

  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::FutureAndRequestId result_future =
    path_waypoint_position_srv_->async_send_request(request);

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

void koma::BTNode::path_goal_position(double x, double y, double theta)
{
  while (!path_goal_position_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "goal_pose is not available. localization/path_plan");
  }

  inrof2026_koma_type::srv::PoseStamped_Request::SharedPtr request =
    std::make_shared<inrof2026_koma_type::srv::PoseStamped::Request>();
  request->pose_stamped.pose.position.x = x;
  request->pose_stamped.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  request->pose_stamped.pose.orientation = tf2::toMsg(q);

  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::FutureAndRequestId result_future =
    path_goal_position_srv_->async_send_request(request);

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

// action server
/*
  arm control
*/
void koma::BTNode::start_arm_control(double x, double y, double z)
{
  while (!arm_control_act_->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) return;
    RCLCPP_WARN(this->get_logger(), "arm control not available");
  }

  auto goal_msg = inrof2026_koma_type::action::ArmControl::Goal();
  auto send_goal_options =
    rclcpp_action::Client<inrof2026_koma_type::action::ArmControl>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&BTNode::arm_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback =
    std::bind(&BTNode::arm_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&BTNode::arm_result_callback, this, std::placeholders::_1);

  goal_msg.target_hand_position.pose.position.x = x;
  goal_msg.target_hand_position.pose.position.y = y;
  goal_msg.target_hand_position.pose.position.z = z;

  arm_control_act_->async_send_goal(goal_msg, send_goal_options);
  is_arm_control_runing_.store(true);
}
void koma::BTNode::arm_goal_response_callback(
  rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::ArmControl>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    is_arm_control_runing_.store(false);
    RCLCPP_ERROR(this->get_logger(), "arm control goal was rejected by action server.");
    return;
  }

  is_arm_control_runing_.store(true);
  RCLCPP_INFO(this->get_logger(), "Start arm control.");
}
void koma::BTNode::arm_feedback_callback(
  rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::ArmControl>::SharedPtr goal_handle,
  const std::shared_ptr<const inrof2026_koma_type::action::ArmControl::Feedback> feedback)
{
  (void)goal_handle;
  (void)feedback;
}
void koma::BTNode::arm_result_callback(
  const rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::ArmControl>::WrappedResult
    result)
{
  is_arm_control_runing_.store(false);

  if (!result.result) {
    RCLCPP_WARN(this->get_logger(), "arm control action returned null result.");
  };
  RCLCPP_INFO(this->get_logger(), "arm control complete.");
}
bool koma::BTNode::is_arm_control_running() const { return is_arm_control_runing_.load(); }

/*
  pursuit
*/

void koma::BTNode::start_path_pursuit()
{
  while (!pursuit_act_->wait_for_action_server(1s)) {
    if (!rclcpp::ok()) return;
    RCLCPP_WARN(this->get_logger(), "pursuit not available");
  }

  // when is_pursuit_running_ is true, cancel the current pursuit goal
  if (pursuit_goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "Canceling current pursuit goal");
    pursuit_act_->async_cancel_goal(pursuit_goal_handle_);
  }

  is_pursuit_goal_pending_.store(true);

  auto goal_msg = inrof2026_koma_type::action::Pursuit::Goal();
  auto send_goal_options =
    rclcpp_action::Client<inrof2026_koma_type::action::Pursuit>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&BTNode::pursuit_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(
    &BTNode::pursuit_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback =
    std::bind(&BTNode::result_callback, this, std::placeholders::_1);

  pursuit_act_->async_send_goal(goal_msg, send_goal_options);
}

void koma::BTNode::pursuit_goal_response_callback(
  rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Pursuit goal was rejected by action server.");
    return;
  }
  pursuit_goal_handle_ = goal_handle;
  is_pursuit_goal_pending_.store(false);
  RCLCPP_INFO(this->get_logger(), "Received pursuit goal response. Goal accepted.");
}

void koma::BTNode::pursuit_feedback_callback(
  rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle,
  const std::shared_ptr<const inrof2026_koma_type::action::Pursuit::Feedback> feedback)
{
  (void)goal_handle;
  (void)feedback;
}

void koma::BTNode::result_callback(
  const rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::WrappedResult result)
{
  if (pursuit_goal_handle_ && pursuit_goal_handle_->get_goal_id() == result.goal_id) {
    pursuit_goal_handle_.reset();
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Received result for an old pursuit goal. Ignoring handle reset.");
  }

  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "Pursuit succeeded.");
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(this->get_logger(), "Pursuit canceled.");
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_WARN(this->get_logger(), "Pursuit aborted.");
  }

  if (!result.result) {
    RCLCPP_WARN(this->get_logger(), "Pursuit action returned null result.");
  }
}

bool koma::BTNode::is_pursuit_running() const
{
  return pursuit_goal_handle_ != nullptr || is_pursuit_goal_pending_.load();
}

std::optional<inrof2026_koma_type::srv::BallPosition::Response> koma::BTNode::target_ball_position()
{
  while (!target_ball_position_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "target_ball_pose is not available");
  }

  std::shared_ptr<inrof2026_koma_type::srv::BallPosition_Request> request =
    std::make_shared<inrof2026_koma_type::srv::BallPosition::Request>();
  rclcpp::Client<inrof2026_koma_type::srv::BallPosition>::FutureAndRequestId result_future =
    target_ball_position_srv_->async_send_request(request);

  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
    std::shared_ptr<inrof2026_koma_type::srv::BallPosition_Response> response = result_future.get();
    return *response;
  } else {
    RCLCPP_WARN(this->get_logger(), "target_ball_position service call failed or timed out");
    return std::nullopt;
  }
}

void koma::BTNode::path_ball_position(double x, double y)
{
  // check action server available
  while (!this->path_ball_position_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "path_ball_position_srv_ is not available");
  }

  std::shared_ptr<inrof2026_koma_type::srv::PoseStamped_Request> request =
    std::make_shared<inrof2026_koma_type::srv::PoseStamped::Request>();
  request->pose_stamped.pose.position.x = x;
  request->pose_stamped.pose.position.y = y;

  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::FutureAndRequestId result_future =
    path_ball_position_srv_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

void koma::BTNode::arm_ee_open()
{
  while (!this->arm_ee_open_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "arm_ee_open_srv_ is not available");
  }

  std::shared_ptr<std_srvs::srv::Trigger_Request> request =
    std::make_shared<std_srvs::srv::Trigger::Request>();

  rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId result_future =
    arm_ee_open_srv_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

void koma::BTNode::arm_ee_close()
{
  while (!this->arm_ee_close_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "arm_ee_close_srv_ is not available");
  }

  std::shared_ptr<std_srvs::srv::Trigger_Request> request =
    std::make_shared<std_srvs::srv::Trigger::Request>();

  rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId result_future =
    arm_ee_close_srv_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

void koma::BTNode::arm_default_pose()
{
  while (!this->arm_default_pose_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "arm_default_pose_srv_ is not available");
  }

  std::shared_ptr<std_srvs::srv::Trigger_Request> request =
    std::make_shared<std_srvs::srv::Trigger::Request>();

  rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId result_future =
    arm_default_pose_srv_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

void koma::BTNode::arm_pursuit_pose()
{
  while (!this->arm_pursuit_pose_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "arm_pursuit_pose_srv_ is not available");
  }

  std::shared_ptr<std_srvs::srv::Trigger_Request> request =
    std::make_shared<std_srvs::srv::Trigger::Request>();

  rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId result_future =
    arm_pursuit_pose_srv_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

void koma::BTNode::arm_root_pose(double theta)
{
  while (!this->arm_root_pose_srv_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "arm_root_pose_srv_ is not available");
  }

  std::shared_ptr<inrof2026_koma_type::srv::SetFloat64_Request> request =
    std::make_shared<inrof2026_koma_type::srv::SetFloat64::Request>();
  request->data = theta;

  rclcpp::Client<inrof2026_koma_type::srv::SetFloat64>::FutureAndRequestId result_future =
    arm_root_pose_srv_->async_send_request(request);
  if (
    rclcpp::spin_until_future_complete(
      this->get_node_base_interface(), result_future, std::chrono::seconds(1)) ==
    rclcpp::FutureReturnCode::SUCCESS) {
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<koma::BTNode> ros_node = std::make_shared<koma::BTNode>();
  BT::BehaviorTreeFactory factory;

  BT::NodeBuilder builder_path_waypoint_position =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::PathWaypointPosition>(name, config, ros_node);
    };
  factory.registerBuilder<koma::PathWaypointPosition>("waypoint", builder_path_waypoint_position);

  BT::NodeBuilder builder_path_goal_position =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::PathGoalPosition>(name, config, ros_node);
    };
  factory.registerBuilder<koma::PathGoalPosition>("goal_position", builder_path_goal_position);

  BT::NodeBuilder builder_pursuit =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::BTPursuit>(name, config, ros_node);
    };
  factory.registerBuilder<koma::BTPursuit>("pursuit", builder_pursuit);

  BT::NodeBuilder builder_target_ball_position =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::TargetBallPosition>(name, config, ros_node);
    };
  factory.registerBuilder<koma::TargetBallPosition>(
    "target_ball_position", builder_target_ball_position);

  BT::NodeBuilder builder_target_arm_ee_open =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::ArmEEOpen>(name, config, ros_node);
    };
  factory.registerBuilder<koma::ArmEEOpen>("arm_ee_open", builder_target_arm_ee_open);

  BT::NodeBuilder builder_target_arm_ee_close =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::ArmEEClose>(name, config, ros_node);
    };
  factory.registerBuilder<koma::ArmEEClose>("arm_ee_close", builder_target_arm_ee_close);

  BT::NodeBuilder builder_path_ball_position =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::PathBallPosition>(name, config, ros_node);
    };
  factory.registerBuilder<koma::PathBallPosition>("path_ball", builder_path_ball_position);

  BT::NodeBuilder builder_arm_control =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::ArmControl>(name, config, ros_node);
    };
  factory.registerBuilder<koma::ArmControl>("arm_control", builder_arm_control);

  BT::NodeBuilder builder_arm_default_pose =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::ArmDefaultPose>(name, config, ros_node);
    };
  factory.registerBuilder<koma::ArmDefaultPose>("arm_default_pose", builder_arm_default_pose);

  BT::NodeBuilder builder_arm_root_pose =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::ArmRootPose>(name, config, ros_node);
    };
  factory.registerBuilder<koma::ArmRootPose>("arm_root_pose", builder_arm_root_pose);

  BT::NodeBuilder builder_arm_pursuit_pose =
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::ArmPursuitPose>(name, config, ros_node);
    };
  factory.registerBuilder<koma::ArmPursuitPose>("arm_pursuit_pose", builder_arm_pursuit_pose);

  factory.registerNodeType<koma::WhileDoElseBreakNode>("WhileDoElseBreak");

  ros_node->declare_parameter<std::string>("config_path", "");

  std::string config_path;
  ros_node->get_parameter("config_path", config_path);

  if (config_path.empty()) {
    RCLCPP_ERROR(
      ros_node->get_logger(),
      "Parameter 'config_path' is empty. Please set config_path from launch file.");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(ros_node->get_logger(), "Loading behavior tree config: %s", config_path.c_str());

  factory.registerBehaviorTreeFromFile(config_path);
  BT::Tree tree = factory.createTree("main");

  BT::Groot2Publisher groot2_publisher(tree);
  printTreeRecursively(tree.rootNode());

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
    rclcpp::spin_some(ros_node);
    status = tree.tickOnce();
  }

  tree.haltTree();
  rclcpp::shutdown();
}