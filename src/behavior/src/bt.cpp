#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behavior/bt.hpp>
#include <behavior/path_goal_position.hpp>
#include <behavior/path_waypoint_position.hpp>
#include <behavior/pursuit.hpp>
#include <behavior/target_ball_position.hpp>
#include <behavior/while_do_else_break.hpp>
#include <behavior/path_ball_position.hpp>

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
  target_ball_position_srv_ = this->create_client<inrof2026_koma_type::srv::BallPosition>("target_ball_pose");

  path_ball_position_srv_ = this->create_client<inrof2026_koma_type::srv::PoseStamped>("path_ball");

  // action
  // server: localization/pursuit
  pursuit_act_ = rclcpp_action::create_client<inrof2026_koma_type::action::Pursuit> (this, "pursuit_command");
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

void koma::BTNode::start_path_pursuit() {
    while (!pursuit_act_->wait_for_action_server(1s)){
        if (!rclcpp::ok()) return;
        RCLCPP_WARN(this->get_logger(), "pursuit not available");
    }

    auto goal_msg = inrof2026_koma_type::action::Pursuit::Goal();
    auto send_goal_options = rclcpp_action::Client<inrof2026_koma_type::action::Pursuit>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&BTNode::pursuit_goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&BTNode::pursuit_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&BTNode::result_callback, this, std::placeholders::_1);

    pursuit_act_->async_send_goal(goal_msg, send_goal_options);
    is_pursuit_running_.store(true);
}

void koma::BTNode::pursuit_goal_response_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle
) {
     if (!goal_handle) {
        is_pursuit_running_.store(false);
        RCLCPP_ERROR(this->get_logger(), "Pursuit goal was rejected by action server.");
        return;
    }

    is_pursuit_running_.store(true);
    RCLCPP_INFO(this->get_logger(), "Start pursuit.");
}

void koma::BTNode::pursuit_feedback_callback(
    rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::SharedPtr goal_handle, 
    const std::shared_ptr<const inrof2026_koma_type::action::Pursuit::Feedback> feedback
) {
    (void)goal_handle;
    (void)feedback;
}

void koma::BTNode::result_callback(
    const rclcpp_action::ClientGoalHandle<inrof2026_koma_type::action::Pursuit>::WrappedResult result
){
    is_pursuit_running_.store(false);

    if (!result.result)  {
        RCLCPP_WARN(this->get_logger(), "Pursuit action returned null result.");
    };
    RCLCPP_INFO(this->get_logger(), "Pursuit complete.");
}


bool koma::BTNode::is_pursuit_runing() const
{
    return is_pursuit_running_.load();
}

std::optional<inrof2026_koma_type::srv::BallPosition::Response> koma::BTNode::target_ball_position() {
    while(!target_ball_position_srv_->wait_for_service(1s)) {
        if (!rclcpp::ok()) break;
        RCLCPP_WARN(this->get_logger(), "target_ball_pose is not available");
    }

    std::shared_ptr<inrof2026_koma_type::srv::BallPosition_Request> request = std::make_shared<inrof2026_koma_type::srv::BallPosition::Request>();
    rclcpp::Client<inrof2026_koma_type::srv::BallPosition>::FutureAndRequestId result_future = target_ball_position_srv_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            result_future,
            std::chrono::seconds(1))
        == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::shared_ptr<inrof2026_koma_type::srv::BallPosition_Response> response = result_future.get();
        return *response;
    } else {
        RCLCPP_WARN(this->get_logger(), "target_ball_position service call failed or timed out");
        return std::nullopt;
    }
}

void koma::BTNode::path_ball_position(double x, double y) {
    // check action server available
    while (!this->path_ball_position_srv_->wait_for_service(1s))
    {
        if (!rclcpp::ok()) break;
        RCLCPP_WARN(this->get_logger(), "path_ball_position_srv_ is not available");
    }

    std::shared_ptr<inrof2026_koma_type::srv::PoseStamped_Request> request = std::make_shared<inrof2026_koma_type::srv::PoseStamped::Request>();
    request->pose_stamped.pose.position.x = x;
    request->pose_stamped.pose.position.y = y;

    rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::FutureAndRequestId result_future = path_ball_position_srv_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            result_future,
            std::chrono::seconds(1))
        == rclcpp::FutureReturnCode::SUCCESS){}
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
  factory.registerBuilder<koma::PathGoalPosition>("goal_position",   builder_path_goal_position);

  BT::NodeBuilder builder_pursuit = 
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::BTPursuit>(name, config, ros_node);
    };
  factory.registerBuilder<koma::BTPursuit>("pursuit", builder_pursuit);

  BT::NodeBuilder builder_target_ball_position = 
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::TargetBallPosition>(name, config, ros_node);
    };
  factory.registerBuilder<koma::TargetBallPosition>("target_ball_position", builder_target_ball_position);

  BT::NodeBuilder builder_path_ball_position = 
    [ros_node](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<koma::PathBallPosition>(name, config, ros_node);
    };
  factory.registerBuilder<koma::PathBallPosition>("path_ball", builder_path_ball_position);

  factory.registerNodeType<koma::WhileDoElseBreakNode>("WhileDoElseBreak");

  std::string package_path = ament_index_cpp::get_package_share_directory("inrof2026_koma");
  factory.registerBehaviorTreeFromFile(package_path + "/config/koma_bt.xml");
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