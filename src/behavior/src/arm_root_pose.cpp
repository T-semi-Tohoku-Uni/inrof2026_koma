#include <behavior/arm_root_pose.hpp>
#include <behavior/bt.hpp>
#include <rclcpp/rclcpp.hpp>

koma::ArmRootPose::ArmRootPose(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList koma::ArmRootPose::providedPorts() { return {BT::InputPort<double>("theta")}; }

BT::NodeStatus koma::ArmRootPose::tick()
{
  BT::Expected<double> tmp_theta = getInput<double>("theta");
  if (!tmp_theta) {
    throw BT::RuntimeError("missing required input theta: ", tmp_theta.error());
  }
  double theta = tmp_theta.value();

  this->ros_node_->arm_root_pose(theta);

  return BT::NodeStatus::SUCCESS;
}