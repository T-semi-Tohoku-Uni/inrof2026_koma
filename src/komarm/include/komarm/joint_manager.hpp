#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::chrono_literals;

namespace koma {
    class JointManager: public rclcpp::Node {
        public:
            JointManager(const rclcpp::NodeOptions & options=rclcpp::NodeOptions());
        private:
            rclcpp::TimerBase::SharedPtr timer_;

            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
            std::vector<std::string> joint_names_;
            std::vector<double> joint_positions_;

            void timerCallback();
    };
}
