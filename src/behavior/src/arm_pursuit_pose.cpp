#include "behavior/arm_pursuit_pose.hpp"
#include "behavior/bt.hpp"
#include <rclcpp/rclcpp.hpp>

koma::ArmPursuitPose::ArmPursuitPose(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList koma::ArmPursuitPose::providedPorts() { return {}; }

BT::NodeStatus koma::ArmPursuitPose::tick()
{
  this->ros_node_->arm_pursuit_pose();

  return BT::NodeStatus::SUCCESS;
}