#include <behavior/bt.hpp>
#include <behavior/pursuit.hpp>

koma::BTPursuit::BTPursuit(
  const std::string & name, const BT::NodeConfiguration & config, std::shared_ptr<BTNode> ros_node)
: StatefulActionNode(name, config), ros_node_(ros_node)
{
}

BT::NodeStatus koma::BTPursuit::onStart()
{
  this->ros_node_->start_path_pursuit();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus koma::BTPursuit::onRunning()
{
  if (this->ros_node_->is_pursuit_runing()) {
    RCLCPP_DEBUG(this->ros_node_->get_logger(), "Pursuit is Runing");
    return BT::NodeStatus::RUNNING;
  } else {
    RCLCPP_DEBUG(this->ros_node_->get_logger(), "Pursuit is Success");
    return BT::NodeStatus::SUCCESS;
  }
}

void koma::BTPursuit::onHalted()
{
  RCLCPP_INFO(this->ros_node_->get_logger(), "interrupt Pursuit");
}

koma::BTPursuit::~BTPursuit() { this->ros_node_.reset(); }