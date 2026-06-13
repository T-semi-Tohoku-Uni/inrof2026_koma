#include "uart_bridge/realsense_rgb.hpp"

#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>

RealsenseRgb::RealsenseRgb() : Node("realsense_rgb_publisher")
{
  this->declare_parameter<std::string>("camera_path", "/dev/video0");
  this->declare_parameter<int>("width", 320);
  this->declare_parameter<int>("height", 180);
  this->declare_parameter<double>("fps", 30.0);

  camera_path_ = this->get_parameter("camera_path").as_string();
  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();
  fps_ = this->get_parameter("fps").as_double();

  cap_.open(camera_path_, cv::CAP_V4L2);

  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open camera_path=%s", camera_path_.c_str());
    throw std::runtime_error("camera open failed");
  }

  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS, fps_);

  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);

  std::chrono::milliseconds period(static_cast<int>(1000.0 / fps_));

  timer_ = this->create_wall_timer(period, std::bind(&RealsenseRgb::publish_frame, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Started RealsenseRgb publisher: camera_path=%s, width=%d, height=%d, fps=%.2f",
    camera_path_.c_str(), width_, height_, fps_);
}

void RealsenseRgb::publish_frame()
{
  cv::Mat frame_bgr;
  cap_ >> frame_bgr;

  if (frame_bgr.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty frame");
    return;
  }

  cv_bridge::CvImage cv_image(std_msgs::msg::Header(), "bgr8", frame_bgr);

  cv_image.header.stamp = this->now();
  cv_image.header.frame_id = "camera_color_frame";

  sensor_msgs::msg::Image::SharedPtr image_msg = cv_image.toImageMsg();
  publisher_->publish(*image_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealsenseRgb>());
  rclcpp::shutdown();
  return 0;
}