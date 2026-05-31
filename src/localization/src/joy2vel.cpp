#include <localization/joy2vel.hpp>

koma::Joy2Vel::Joy2Vel(const rclcpp::NodeOptions & options) : Node("joy2vel", options)
{
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile());
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
    std::bind(&koma::Joy2Vel::callback, this, std::placeholders::_1));
}

void koma::Joy2Vel::callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  std::float_t leftJoyx_ = msg->axes[0];
  std::float_t leftJoyy_ = msg->axes[1];
  // std::float_t rightJoyx_ = msg->axes[2];
  std::float_t rightJoyy_ = msg->axes[3];

  geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();
  twist.linear.set__x(leftJoyy_ * 0.1);
  twist.linear.set__y(leftJoyx_ * 0.1);
  twist.linear.set__z(0.0);
  twist.angular.set__x(0.0);
  twist.angular.set__y(0.0);
  twist.angular.set__z(rightJoyy_);
  cmd_pub_->publish(twist);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<koma::Joy2Vel>());
  rclcpp::shutdown();

  return 0;
}