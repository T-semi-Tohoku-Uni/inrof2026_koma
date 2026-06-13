#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>

class RealsenseRgb : public rclcpp::Node
{
public:
  RealsenseRgb();

private:
  void publish_frame();

  std::string camera_path_;
  int width_;
  int height_;
  double fps_;

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};