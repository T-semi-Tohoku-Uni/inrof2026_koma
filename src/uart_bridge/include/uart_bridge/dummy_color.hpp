#pragma once

#include <rclcpp/rclcpp.hpp>
#include <inrof2026_koma_type/srv/color.hpp>

#include <memory>
#include <random>

namespace koma
{

class ColorMock : public rclcpp::Node
{
public:
  explicit ColorMock(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void color_callback(
    const std::shared_ptr<inrof2026_koma_type::srv::Color::Request> request,
    const std::shared_ptr<inrof2026_koma_type::srv::Color::Response> response);

  rclcpp::Service<inrof2026_koma_type::srv::Color>::SharedPtr color_srv_;

  std::mt19937 gen_;
  std::uniform_int_distribution<int> dist_;
};

}  // namespace koma