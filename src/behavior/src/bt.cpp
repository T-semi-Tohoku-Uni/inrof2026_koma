#include <behavior/bt.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behavior/path_waypoint_position.hpp>
#include <behavior/path_goal_position.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std::chrono_literals;

koma::BTNode::BTNode(const rclcpp::NodeOptions & options): Node("bt_node", options) {
    // server: localization/path_plan
    path_waypoint_position_srv_ = 
        this->create_client<inrof2026_koma_type::srv::PoseStamped>("waypoint");
    
    // server: localization/path_plan
    path_goal_position_srv_ = 
        this->create_client<inrof2026_koma_type::srv::PoseStamped>("goal_pose");
}

void koma::BTNode::path_waypoint_position(double x, double y) {
    while(!path_waypoint_position_srv_->wait_for_service(1s)) {
        if (!rclcpp::ok()) break;
        RCLCPP_WARN(this->get_logger(), "waypoint is not available. localization/path_plan");
    }

    inrof2026_koma_type::srv::PoseStamped_Request::SharedPtr request
        = std::make_shared<inrof2026_koma_type::srv::PoseStamped::Request>();
    request->pose_stamped.pose.position.x = x;
    request->pose_stamped.pose.position.y = y;

    rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::FutureAndRequestId result_future
            = path_waypoint_position_srv_->async_send_request(request);

    if (
        rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            result_future,
            std::chrono::seconds(1)
        ) == rclcpp::FutureReturnCode::SUCCESS
    ) {}
}

void koma::BTNode::path_goal_position(double x, double y, double theta) {
    while(!path_goal_position_srv_->wait_for_service(1s)) {
        if (!rclcpp::ok()) break;
        RCLCPP_WARN(this->get_logger(), "goal_pose is not available. localization/path_plan");
    }

    inrof2026_koma_type::srv::PoseStamped_Request::SharedPtr request
        = std::make_shared<inrof2026_koma_type::srv::PoseStamped::Request>();
    request->pose_stamped.pose.position.x = x;
    request->pose_stamped.pose.position.y = y;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    request->pose_stamped.pose.orientation = tf2::toMsg(q);

    rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::FutureAndRequestId result_future
            = path_goal_position_srv_->async_send_request(request);

    if (
        rclcpp::spin_until_future_complete(
            this->get_node_base_interface(),
            result_future,
            std::chrono::seconds(1)
        ) == rclcpp::FutureReturnCode::SUCCESS
    ) {}
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); 

    std::shared_ptr<koma::BTNode> ros_node = std::make_shared<koma::BTNode>();
    BT::BehaviorTreeFactory factory;

    BT::NodeBuilder path_waypoint_position = 
        [ros_node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<koma::PathWaypointPosition>(name, config, ros_node);
        };
    factory.registerBuilder<koma::PathWaypointPosition>("waypoint", path_waypoint_position);

    BT::NodeBuilder path_goal_position = 
        [ros_node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<koma::PathGoalPosition>(name, config, ros_node);
        };
    factory.registerBuilder<koma::PathGoalPosition>("goal_position", path_goal_position);

    std::string package_path = ament_index_cpp::get_package_share_directory("inrof2026_koma");
    factory.registerBehaviorTreeFromFile(package_path + "/config/koma_bt.xml");
    BT::Tree tree = factory.createTree("main");
    
    BT::Groot2Publisher groot2_publisher(tree);
    printTreeRecursively(tree.rootNode());

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while(status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
        rclcpp::spin_some(ros_node);
        status = tree.tickOnce();
    }

    tree.haltTree();
    rclcpp::shutdown();
}