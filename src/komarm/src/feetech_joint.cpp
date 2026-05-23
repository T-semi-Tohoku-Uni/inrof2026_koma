#include <komarm/feetech_joint.hpp>

koma::FeetechJointManager::FeetechJointManager(const rclcpp::NodeOptions & options)
: Node("joint_manager", options),
  joint_names_{"Revolute 12", "Revolute 11", "Revolute 7", "Revolute 8", "Revolute 9"}
{
  const std::string port_1_2 = this->declare_parameter<std::string>(
    "port_1_2",
    "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.1:1.0"
  );

  const std::string port_3_4 = this->declare_parameter<std::string>(
    "port_3_4",
    "/dev/serial/by-path/pci-0000:00:14.0-usb-0:5:1.0"
  );

  const std::string port_5 = this->declare_parameter<std::string>(
    "port_5",
    ""
  );

  const std::string port_6 = this->declare_parameter<std::string>(
    "port_6",
    ""
  );

  serial_1_2_ = std::make_unique<koma::FeetechSerial>(port_1_2.c_str());
  // serial_3_4_ = std::make_unique<koma::FeetechSerial>(port_3_4.c_str());
  // serial_5_ = std::make_unique<koma::FeetechSerial>(port_5.c_str());
  // serial_6_ = std::make_unique<koma::FeetechSerial>(port_6.c_str());

  // check serial
  if(!serial_1_2_->read_all_state(1, state_1)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 1 is not connected");
  } else {
    state_1.print();
  }
  if (!serial_1_2_->read_all_state(2, state_2)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 2 is not connected");
  } else {
    state_2.print();
  }
  // if (!serial_3_4_->read_all_state(3, state_3)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 3 is not connected");
  // }
  // if (!serial_3_4_->read_all_state(4, state_4)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 4 is not connected");
  // }
  // if (!serial_5_->read_all_state(5, state_5)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 5 is not connected");
  // }
  // if (!serial_6_->read_all_state(6, state_6)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 6 is not connected");
  // }

  cur_joint_state_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());
  publish_cur_joint_state_timer_ = this->create_wall_timer(
    20ms,
    std::bind(&koma::FeetechJointManager::publish_cur_joint_state, this)
  );
  target_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "target_joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&koma::FeetechJointManager::target_joint_state_callback, this, std::placeholders::_1)
  );
}

double koma::FeetechJointManager::rad_to_tick(double rad)
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

  return tick;
}

double koma::FeetechJointManager::tick_to_rad(int tick)
{
  return (static_cast<double>(tick) / 4096.0) * 2.0 * M_PI - M_PI;
}

void koma::FeetechJointManager::target_joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg) {
  serial_1_2_->send_write_state_command(1, rad_to_tick(msg->position[0]));
  if (serial_1_2_->wait_write_ack(1, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 1 is not responed");
  }
  serial_1_2_->send_write_state_command(2, rad_to_tick(msg->position[1]));
  if (serial_1_2_->wait_write_ack(2, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 2 is not responed");
  }
  serial_1_2_->send_action_command();
}

void koma::FeetechJointManager::publish_cur_joint_state() {
  if (!serial_1_2_->wait_read_state_response(1, state_1, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 1 is not responed");
  }
  if (!serial_1_2_->wait_read_state_response(2, state_2, 4ms)) {
    RCLCPP_WARN(this->get_logger(), "Servo id 2 is not responed");
  }
  // if (serial_3_4_->wait_read_state_response(3, state_3, 4ms)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 3 is not responed");
  // }
  // if (serial_3_4_->wait_read_state_response(4, state_4, 4ms)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 4 is not responed");
  // }
  // if (serial_5_->wait_read_state_response(5, state_5, 4ms)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 5 is not responed");
  // }
  // if (serial_6_->wait_read_state_response(6, state_6, 4ms)) {
  //   RCLCPP_WARN(this->get_logger(), "Servo id 6 is not responed");
  // }

  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names_;
  joint_state.position = {
    tick_to_rad(state_1.present_position),
    tick_to_rad(state_2.present_position),
    0.0,
    0.0,
    0.0, // TODO
  };
  
  cur_joint_state_pub_->publish(joint_state);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<koma::FeetechJointManager> node = std::make_shared<koma::FeetechJointManager>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}