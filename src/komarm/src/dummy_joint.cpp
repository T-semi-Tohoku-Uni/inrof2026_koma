#include <komarm/joint_manager.hpp>

koma::JointManager::JointManager(
    const rclcpp::NodeOptions & options
): Node("joint_manager") {
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states",
        10
    );

    joint_names_ = {
        "Revolute_12",
        "Revolute_11",
        "Revolute_7",
        "Revolute_8",
        "Revolute_9",
    };
    joint_positions_ = {
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    };

    timer_ = this->create_wall_timer(
        50ms,
        std::bind(&koma::JointManager::timerCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "Success initialize JointManager");
}

void koma::JointManager::timerCallback() {
    sensor_msgs::msg::JointState msg;

    msg.header.stamp = this->get_clock()->now();
    msg.name = joint_names_;
    msg.position = joint_positions_;

    joint_state_pub_->publish(msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<koma::JointManager> node = std::make_shared<koma::JointManager>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}