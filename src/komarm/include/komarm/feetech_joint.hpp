#include <error.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vector>

namespace koma
{
class FeetechJointState
{
public:
  FeetechJointState(int id);

private:
  int id_;
  double position_;
  double velocity_;
  double effort_;
};

class Serial
{
public:
  Serial(const char * device_name);

private:
  int serial_fd_;
  int open_serial(const char * device_name);
  // bool write();
  // FeetechJointState read();
};

class FeetechJointManager : public rclcpp::Node
{
public:
  FeetechJointManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cur_joint_state_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_joint_state_sub_;
  koma::Serial serial_port;

  void publish_cur_joint_state();
  // void target_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};
}  // namespace koma