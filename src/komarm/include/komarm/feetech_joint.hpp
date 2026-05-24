#include <error.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <feetech_servo/sts3125.hpp>
#include <functional>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

using namespace std::chrono_literals;

namespace koma
{
class FeetechJointManager : public rclcpp::Node
{
public:
  FeetechJointManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cur_joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_sub_;
  rclcpp::TimerBase::SharedPtr publish_cur_joint_state_timer_;
  std::unique_ptr<koma::FeetechSerial> serial_1_2_;
  std::unique_ptr<koma::FeetechSerial> serial_3_4_;
  std::unique_ptr<koma::FeetechSerial> serial_5_6_;

  std::vector<std::string> joint_names_;

  koma::FeetechState state_1;
  koma::FeetechState state_2;
  koma::FeetechState state_3;
  koma::FeetechState state_4;
  koma::FeetechState state_5;
  koma::FeetechState state_6;

  void publish_cur_joint_state();
  void target_joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);
  double tick_to_rad(int tick);
  uint16_t rad_to_tick(double rad);
  int decode_present_speed(uint16_t raw_speed);
  double speed_tick_to_rad_per_sec(uint16_t raw_speed);
  // void target_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};
}  // namespace koma