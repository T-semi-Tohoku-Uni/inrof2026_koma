#include <behavior/bt.hpp>
#include <behavior/path_waypoint_position.hpp>

koma::PathWaypointPosition::PathWaypointPosition(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<koma::BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node)
{
}

BT::PortsList koma::PathWaypointPosition::providedPorts()
{
  return {BT::InputPort<double>("x"), BT::InputPort<double>("y")};
}

BT::NodeStatus koma::PathWaypointPosition::tick()
{
  BT::Expected<double> tmp_x = getInput<double>("x");
  BT::Expected<double> tmp_y = getInput<double>("y");

  if (!tmp_x) {
    throw BT::RuntimeError("missing required input x: ", tmp_x.error());
  }
  if (!tmp_y) {
    throw BT::RuntimeError("missing required input x: ", tmp_y.error());
  }

  double x = tmp_x.value();
  double y = tmp_y.value();

  if (this->ros_node_ == nullptr) {
    throw BT::RuntimeError("ros_node is null ptr");
  }

  this->ros_node_->path_waypoint_position(x, y);

  return BT::NodeStatus::SUCCESS;
}