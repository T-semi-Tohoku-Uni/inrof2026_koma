#include <behavior/arm_default_pose.hpp>
#include <behavior/bt.hpp>
#include <rclcpp/rclcpp.hpp>

koma::ArmDefaultPose::ArmDefaultPose(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList koma::ArmDefaultPose::providedPorts() { return {}; }

BT::NodeStatus koma::ArmDefaultPose::tick()
{
  this->ros_node_->arm_default_pose();

  return BT::NodeStatus::SUCCESS;
}