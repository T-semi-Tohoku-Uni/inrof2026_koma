#pragma once
#include <behaviortree_cpp/behavior_tree.h>

namespace koma {
    class BTNode;

    class TargetBallPosition: public BT::SyncActionNode {
        public:
            TargetBallPosition(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<koma::BTNode> ros_node);
            static BT::PortsList providedPorts(); 
            BT::NodeStatus tick() override;
            ~TargetBallPosition();
        private:
            std::shared_ptr<BTNode> ros_node_;
    };
}