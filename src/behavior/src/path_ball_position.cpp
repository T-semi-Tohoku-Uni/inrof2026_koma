#include <behavior/bt.hpp>
#include <behavior/path_ball_position.hpp>
#include <rclcpp/rclcpp.hpp>

koma::PathBallPosition::PathBallPosition(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList koma::PathBallPosition::providedPorts()
{
  return {
    BT::InputPort<double>("x"),
    BT::InputPort<double>("y"),
  };
}

BT::NodeStatus koma::PathBallPosition::tick()
{
  RCLCPP_INFO(this->ros_node_->get_logger(), "Start BallPath");

  BT::Expected<double> tmp_x = getInput<double>("x");
  BT::Expected<double> tmp_y = getInput<double>("y");
  if (!tmp_x) {
    throw BT::RuntimeError("missing required input x: ", tmp_x.error());
  }
  if (!tmp_y) {
    throw BT::RuntimeError("missing required input y: ", tmp_y.error());
  }

  double x = tmp_x.value();
  double y = tmp_y.value();

  if (this->ros_node_ == nullptr) RCLCPP_INFO(this->ros_node_->get_logger(), "null ptr");

  this->ros_node_->path_ball_position(x, y);

  return BT::NodeStatus::SUCCESS;
}