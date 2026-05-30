#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace koma {
    class Joy2Vel: public rclcpp::Node {
        public:
            Joy2Vel(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        private:
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

            void callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    };
}