#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace koma
{
class BTNode;

class BTPursuit : public BT::StatefulActionNode
{
public:
  BTPursuit(
    const std::string & name, const BT::NodeConfiguration & config,
    std::shared_ptr<BTNode> ros_node);
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  ~BTPursuit();

private:
  std::shared_ptr<BTNode> ros_node_;
};
}  // namespace koma