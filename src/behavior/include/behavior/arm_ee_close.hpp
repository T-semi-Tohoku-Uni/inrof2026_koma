#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace koma
{
class BTNode;

class ArmEEClose : public BT::SyncActionNode
{
public:
  ArmEEClose(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BTNode> ros_node_;
};
}  // namespace koma