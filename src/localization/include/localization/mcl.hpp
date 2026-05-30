#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace koma {
    class MCL: public rclcpp::Node {
        public:
            MCL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        private:

            // publisher
            rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;

            // subscriber
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    };
}