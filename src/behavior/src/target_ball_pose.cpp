#include <behavior/bt.hpp>
#include <behavior/target_ball_position.hpp>

koma::TargetBallPosition::TargetBallPosition(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<koma::BTNode> ros_node)
: SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList koma::TargetBallPosition::providedPorts()
{
  return {BT::OutputPort<double>("x"), BT::OutputPort<double>("y")};
}

BT::NodeStatus koma::TargetBallPosition::tick()
{
  std::optional<inrof2026_koma_type::srv::BallPosition::Response> target_ball_position =
    this->ros_node_->target_ball_position();
  if (!target_ball_position) return BT::NodeStatus::FAILURE;
  if (!target_ball_position->detect) return BT::NodeStatus::FAILURE;
  setOutput("x", target_ball_position->pose_stamped.pose.position.x);
  setOutput("y", target_ball_position->pose_stamped.pose.position.y);
  RCLCPP_INFO(
    ros_node_->get_logger(), "Ball found (x, y) = (%lf, %lf)",
    target_ball_position->pose_stamped.pose.position.x,
    target_ball_position->pose_stamped.pose.position.y);
  return BT::NodeStatus::SUCCESS;
}

koma::TargetBallPosition::~TargetBallPosition() { this->ros_node_.reset(); }