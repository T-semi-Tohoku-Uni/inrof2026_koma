#include <uart_bridge/dummy_color.hpp>

#include <functional>

namespace koma
{

ColorMock::ColorMock(const rclcpp::NodeOptions & options)
: Node("color_node", options),
  gen_(std::random_device{}()),
  dist_(0, 2)
{
  color_srv_ = this->create_service<inrof2026_koma_type::srv::Color>(
    "color",
    std::bind(
      &ColorMock::color_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Color mock service started");
}

void ColorMock::color_callback(
  const std::shared_ptr<inrof2026_koma_type::srv::Color::Request> request,
  const std::shared_ptr<inrof2026_koma_type::srv::Color::Response> response)
{
  (void)request;

  response->color = dist_(gen_);

  RCLCPP_INFO(this->get_logger(), "mock color: %d", response->color);
}

}  // namespace koma

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<koma::ColorMock>());
  rclcpp::shutdown();
  return 0;
}