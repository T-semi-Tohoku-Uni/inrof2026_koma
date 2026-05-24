#include <komarm/feetech_joint.hpp>

koma::FeetechJointManager::FeetechJointManager(const rclcpp::NodeOptions & options)
: Node("joint_manager", options),
  joint_names_{"Revolute 12", "Revolute 11", "Revolute 7", "Revolute 8", "Revolute 9"}
{
  const std::string port_1_2 = this->declare_parameter<std::string>(
    "port_1_2", "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.1:1.0");

  const std::string port_3_4 = this->declare_parameter<std::string>(
    "port_3_4", "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.2:1.0");

  const std::string port_5_6 = this->declare_parameter<std::string>(
    "port_5_6", "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.3:1.0");

  const double servo_1_min = this->declare_parameter<double> (
    "servo_1_min", -M_PI
  );
  const double servo_2_min = this->declare_parameter<double> (
    "servo_2_min", -M_PI
  );
  const double servo_3_min = this->declare_parameter<double> (
    "servo_3_min", -M_PI
  );
  const double servo_4_min = this->declare_parameter<double> (
    "servo_4_min", -M_PI
  );
  const double servo_5_min = this->declare_parameter<double> (
    "servo_5_min", -M_PI
  );
  // const double servo_1_min = this->declare_parameter<double> (
  //   "servo_6_min", -M_1_PI
  // );

  const double servo_1_max = this->declare_parameter<double> (
    "servo_1_max", M_PI
  );
  const double servo_2_max = this->declare_parameter<double> (
    "servo_2_max", M_PI
  );
  const double servo_3_max = this->declare_parameter<double> (
    "servo_3_max", M_PI
  );
  const double servo_4_max = this->declare_parameter<double> (
    "servo_4_max", M_PI
  );
  const double servo_5_max = this->declare_parameter<double> (
    "servo_5_max", M_PI
  );
  // const double servo_6_max = this->declare_parameter<double> (
  //   "servo_6_max", M_1_PI
  // );

  const bool is_servo_1_reverse = this->declare_parameter<bool> (
    "is_servo_1_reverse", false
  );
  const bool is_servo_2_reverse = this->declare_parameter<bool> (
    "is_servo_2_reverse", false
  );
  const bool is_servo_3_reverse = this->declare_parameter<bool> (
    "is_servo_3_reverse", false
  );
  const bool is_servo_4_reverse = this->declare_parameter<bool> (
    "is_servo_4_reverse", false
  );
  const bool is_servo_5_reverse = this->declare_parameter<bool> (
    "is_servo_5_reverse", false
  );

  serial_1_2_ = std::make_unique<koma::FeetechSerial>(port_1_2.c_str());
  serial_3_4_ = std::make_unique<koma::FeetechSerial>(port_3_4.c_str());
  serial_5_6_ = std::make_unique<koma::FeetechSerial>(port_5_6.c_str());


  if (!serial_1_2_->set_angle_limit(1, rad_to_tick(servo_1_min), rad_to_tick(servo_1_max))) {
    RCLCPP_WARN(this->get_logger(), "Failed to limit servo 1");
  }
  if (!serial_1_2_->set_angle_limit(2, rad_to_tick(servo_2_min), rad_to_tick(servo_2_max))) {
    RCLCPP_WARN(this->get_logger(), "Failed to limit servo 1");
  }
  if (!serial_3_4_->set_angle_limit(3, rad_to_tick(servo_3_min), rad_to_tick(servo_3_max))) {
    RCLCPP_WARN(this->get_logger(), "Failed to limit servo 1");
  }
  if (!serial_3_4_->set_angle_limit(4, rad_to_tick(servo_4_min), rad_to_tick(servo_4_max))) {
    RCLCPP_WARN(this->get_logger(), "Failed to limit servo 1");
  }
  if (!serial_5_6_->set_angle_limit(5, rad_to_tick(servo_5_min), rad_to_tick(servo_5_max))) {
    RCLCPP_WARN(this->get_logger(), "Failed to limit servo 1");
  }

  if (!serial_1_2_->set_reverse(1, is_servo_1_reverse)) {
    RCLCPP_WARN(this->get_logger(), "Failed to reverse reverse 1");
  }
  if (!serial_1_2_->set_reverse(2, is_servo_2_reverse)) {
    RCLCPP_WARN(this->get_logger(), "Failed to reverse reverse 2");
  }
  if (!serial_3_4_->set_reverse(3, is_servo_3_reverse)) {
    RCLCPP_WARN(this->get_logger(), "Failed to reverse reverse 3");
  }
  if (!serial_3_4_->set_reverse(4, is_servo_4_reverse)) {
    RCLCPP_WARN(this->get_logger(), "Failed to reverse reverse 4");
  }
  if (!serial_5_6_->set_reverse(5, is_servo_5_reverse)) {
    RCLCPP_WARN(this->get_logger(), "Failed to reverse reverse 5");
  }

  // check serial
  if (!serial_1_2_->read_all_state(1, state_1)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 1 is not connected");
  } else {
    state_1.print();
  }
  if (!serial_1_2_->read_all_state(2, state_2)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 2 is not connected");
  } else {
    state_2.print();
  }
  if (!serial_3_4_->read_all_state(3, state_3)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 3 is not connected");
  } else {
    state_3.print();
  }
  if (!serial_3_4_->read_all_state(4, state_4)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 4 is not connected");
  } else {
    state_4.print();
  }
  if (!serial_5_6_->read_all_state(5, state_5)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 5 is not connected");
  } else {
    state_5.print();
  }
  // if (!serial_5_6_->read_all_state(6, state_6)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 6 is not connected");
  // } else {
  //   state_6.print();
  // }

  cur_joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  publish_cur_joint_state_timer_ = this->create_wall_timer(
    20ms, std::bind(&koma::FeetechJointManager::publish_cur_joint_state, this));
  target_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "target_joint_states", rclcpp::SensorDataQoS(),
    std::bind(
      &koma::FeetechJointManager::target_joint_state_callback, this, std::placeholders::_1));
}

uint16_t koma::FeetechJointManager::rad_to_tick(double rad)
{
  while (rad > M_PI) {
    rad -= 2.0 * M_PI;
  }

  while (rad < -M_PI) {
    rad += 2.0 * M_PI;
  }

  double tick = rad * 4096.0 / (2.0 * M_PI) + 2048.0;

  if (tick < 0.0) {
    tick = 0.0;
  }

  if (tick > 4095.0) {
    tick = 4095.0;
  }

  return static_cast<uint16_t>(std::lround(tick));
}

double koma::FeetechJointManager::tick_to_rad(int tick)
{
  return (static_cast<double>(tick) / 4096.0) * 2.0 * M_PI - M_PI;
}

void koma::FeetechJointManager::target_joint_state_callback(
  sensor_msgs::msg::JointState::SharedPtr msg)
{
  serial_1_2_->send_write_state_command(1, rad_to_tick(msg->position[0]));
  serial_3_4_->send_write_state_command(3, rad_to_tick(msg->position[2]));
  serial_5_6_->send_write_state_command(5, rad_to_tick(msg->position[4]));
  if (!serial_1_2_->wait_write_ack(1, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 1 is not responed");
  }
  if (!serial_3_4_->wait_write_ack(3, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 3 is not responed");
  }
  if (!serial_5_6_->wait_write_ack(5, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 5 is not responed");
  }

  serial_1_2_->send_write_state_command(2, rad_to_tick(msg->position[1]));
  serial_3_4_->send_write_state_command(4, rad_to_tick(msg->position[3]));
  // serial_5_6_->send_write_state_command(6, rad_to_tick(msg->position[5]));
  if (!serial_1_2_->wait_write_ack(2, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 2 is not responed");
  }
  if (!serial_3_4_->wait_write_ack(4, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 4 is not responed");
  }
  // if (!serial_5_6_->wait_write_ack(6, 4ms)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 6 is not responed");
  // }
  serial_1_2_->send_action_command();
  serial_3_4_->send_action_command();
  serial_5_6_->send_action_command();
}

void koma::FeetechJointManager::publish_cur_joint_state()
{
  serial_1_2_->send_read_state_command(1);
  serial_3_4_->send_read_state_command(3);
  serial_5_6_->send_read_state_command(5);
  if (!serial_1_2_->wait_read_state_response(1, state_1, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 1 is not responed");
  }
  if (!serial_3_4_->wait_read_state_response(3, state_3, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 3 is not responed");
  }
  if (!serial_5_6_->wait_read_state_response(5, state_5, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 5 is not responed");
  }
  serial_1_2_->send_read_state_command(2);
  serial_3_4_->send_read_state_command(4);
  // serial_5_6_->send_read_state_command(6);
  if (!serial_1_2_->wait_read_state_response(2, state_2, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 2 is not responed");
  }
  if (!serial_3_4_->wait_read_state_response(4, state_4, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 4 is not responed");
  }
  // if (!serial_5_6_->wait_read_state_response(6, state_6, 4ms)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 6 is not responed");
  // }

  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names_;
  joint_state.position = {
    tick_to_rad(state_1.present_position), tick_to_rad(state_2.present_position),
    tick_to_rad(state_3.present_position), tick_to_rad(state_4.present_position),
    tick_to_rad(state_5.present_position), 
    // tick_to_rad(state_6.present_position)
  };

  joint_state.velocity = {
    speed_tick_to_rad_per_sec(state_1.present_speed),
    speed_tick_to_rad_per_sec(state_2.present_speed),
    speed_tick_to_rad_per_sec(state_3.present_speed),
    speed_tick_to_rad_per_sec(state_4.present_speed),
    speed_tick_to_rad_per_sec(state_5.present_speed),
    // speed_tick_to_rad_per_sec(state_6.present_speed),
  };

  joint_state.header.stamp = this->get_clock()->now();
  cur_joint_state_pub_->publish(joint_state);
}

int koma::FeetechJointManager::decode_present_speed(uint16_t raw_speed)
{
  if (raw_speed & 0x8000) {
    return -static_cast<int>(raw_speed & 0x7FFF);
  }

  return static_cast<int>(raw_speed);
}

double koma::FeetechJointManager::speed_tick_to_rad_per_sec(uint16_t raw_speed)
{
  const int speed_tick_per_sec = decode_present_speed(raw_speed);
  return static_cast<double>(speed_tick_per_sec) * 2.0 * M_PI / 4096.0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<koma::FeetechJointManager> node = std::make_shared<koma::FeetechJointManager>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}