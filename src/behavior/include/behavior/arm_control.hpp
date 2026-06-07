#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace koma
{
class BTNode;

class ArmControl : public BT::StatefulActionNode
{
public:
  ArmControl(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<BTNode> ros_node);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::shared_ptr<BTNode> ros_node_;
};
}  // namespace koma