#include <behavior/bt.hpp>
#include <behavior/arm_ee_close.hpp>
#include <rclcpp/rclcpp.hpp>

koma::ArmEEClose::ArmEEClose(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList koma::ArmEEClose::providedPorts() {
    return {};
}

BT::NodeStatus koma::ArmEEClose::tick() {
    this->ros_node_->arm_ee_close();

    return BT::NodeStatus::SUCCESS;
}