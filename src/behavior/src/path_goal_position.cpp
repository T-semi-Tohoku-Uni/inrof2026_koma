#include <behavior/path_goal_position.hpp>
#include <behavior/bt.hpp>

koma::PathGoalPosition::PathGoalPosition(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<koma::BTNode> ros_node):
    BT::SyncActionNode(name, config),
    ros_node_(ros_node) {}

BT::PortsList koma::PathGoalPosition::providedPorts() {
    return {
        BT::InputPort<double> ("x"),
        BT::InputPort<double> ("y"),
        BT::InputPort<double> ("theta")
    };
}

BT::NodeStatus koma::PathGoalPosition::tick() {
    BT::Expected<double> tmp_x = getInput<double>("x");
    BT::Expected<double> tmp_y = getInput<double>("y");
    BT::Expected<double> tmp_theta = getInput<double>("theta");

    if (!tmp_x) {
        throw BT::RuntimeError("missing required input x: ", tmp_x.error());
    } 
    if (!tmp_y) {
        throw BT::RuntimeError("missing required input y: ", tmp_y.error());
    }
    if (!tmp_theta) {
        throw BT::RuntimeError("missing required input theta: ", tmp_theta.error());
    }

    double x = tmp_x.value();
    double y = tmp_y.value();
    double theta = tmp_theta.value();

    if (this->ros_node_ == nullptr) {
        throw BT::RuntimeError("ros_node is null ptr");
    }

    this->ros_node_->path_goal_position(x, y, theta);

    return BT::NodeStatus::SUCCESS;
}