#include <behavior/arm_ee_open.hpp>
#include <behavior/bt.hpp>
#include <rclcpp/rclcpp.hpp>

koma::ArmEEOpen::ArmEEOpen(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList koma::ArmEEOpen::providedPorts() { return {}; }

BT::NodeStatus koma::ArmEEOpen::tick()
{
  this->ros_node_->arm_ee_open();

  return BT::NodeStatus::SUCCESS;
}