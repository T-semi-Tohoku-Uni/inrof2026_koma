#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mcl_node");
  RCLCPP_INFO(node->get_logger(), "hello ros2");
  rclcpp::shutdown();
  return 0;
}