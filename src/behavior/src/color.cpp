#include <behavior/bt.hpp>
#include <behavior/color.hpp>
#include <inrof2026_koma_type/srv/color.hpp>

namespace koma
{
Color::Color(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BTNode> ros_node)
: BT::SyncActionNode(name, config), ros_node_(ros_node){};

BT::PortsList Color::providedPorts() { return {BT::OutputPort<int>("color")}; }

BT::NodeStatus Color::tick()
{
  inrof2026_koma_type::srv::Color::Response color = this->ros_node_->color();

  setOutput("color", color.color);

  return BT::NodeStatus::SUCCESS;
}
}  // namespace koma