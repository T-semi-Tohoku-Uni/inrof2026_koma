#include <behavior/arm_control.hpp>
#include <behavior/bt.hpp>

koma::ArmControl::ArmControl(
  const std::string & name, const BT::NodeConfiguration & config, std::shared_ptr<BTNode> ros_node)
: StatefulActionNode(name, config), ros_node_(ros_node)
{
}

BT::PortsList koma::ArmControl::providedPorts()
{
  return {BT::InputPort<double>("x"), BT::InputPort<double>("y"), BT::InputPort<double>("z")};
}

BT::NodeStatus koma::ArmControl::onStart()
{
  BT::Expected<double> tmp_x = getInput<double>("x");
  BT::Expected<double> tmp_y = getInput<double>("y");
  BT::Expected<double> tmp_z = getInput<double>("z");
  if (!tmp_x) {
    throw BT::RuntimeError("missing required input x: ", tmp_x.error());
  }
  if (!tmp_y) {
    throw BT::RuntimeError("missing required input y: ", tmp_y.error());
  }
  if (!tmp_z) {
    throw BT::RuntimeError("missing required input z: ", tmp_z.error());
  }

  double x = tmp_x.value();
  double y = tmp_y.value();
  double z = tmp_z.value();

  this->ros_node_->start_arm_control(x, y, z);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus koma::ArmControl::onRunning()
{
  if (this->ros_node_->is_arm_control_running()) {
    RCLCPP_DEBUG(this->ros_node_->get_logger(), "Arm control is Runing");
    return BT::NodeStatus::RUNNING;
  } else {
    RCLCPP_DEBUG(this->ros_node_->get_logger(), "Arm control is Success");
    return BT::NodeStatus::SUCCESS;
  }
}

void koma::ArmControl::onHalted()
{
  RCLCPP_INFO(this->ros_node_->get_logger(), "interrupt Arm control");
}