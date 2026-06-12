#include <uart_bridge/color.hpp>

using namespace std::chrono_literals;

koma::Color::Color(const rclcpp::NodeOptions & options) : Node("color_node", options)
{
  std::string serial_port = this->declare_parameter<std::string>(
    "device_name", "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.2");
  serial_fd_ = open_serial(serial_port.c_str());

  if (serial_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  color_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
    "color", rclcpp::QoS(rclcpp::KeepLast(10)));

  receive_timer_ =
    this->create_wall_timer(100ms, std::bind(&Color::receive_color_from_serial, this));

  color_srv_ = this->create_service<inrof2026_koma_type::srv::Color>(
    "color", std::bind(&Color::color_callback, this, std::placeholders::_1, std::placeholders::_2));
}

int koma::Color::open_serial(const char * device_name)
{
  int fd = ::open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"), "Serial Fail: could not open %s (%s)", device_name,
      std::strerror(errno));
    return -1;
  }
  fcntl(fd, F_SETFL, 0);

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"), "Serial Fail: tcgetattr error (%s)", std::strerror(errno));
    ::close(fd);
    return -1;
  }

  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  cfmakeraw(&tty);

  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= (CS8 | CLOCAL | CREAD);

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("rclcpp"), "Serial Fail: tcsetattr error (%s)", std::strerror(errno));
    ::close(fd);
    return -1;
  }

  return fd;
}

/*
    data_1,data_2\r\n
*/
void koma::Color::receive_color_from_serial()
{
  uint8_t tmp[256];
  ssize_t n = read(serial_fd_, tmp, sizeof(tmp));
  static constexpr uint8_t DELTM[] = {'\r', '\n'};

  if (n > 0) {
    recev_buffer_.insert(recev_buffer_.end(), tmp, tmp + n);

    while (1) {
      std::vector<uint8_t>::iterator it_delim =
        std::search(recev_buffer_.begin(), recev_buffer_.end(), std::begin(DELTM), std::end(DELTM));

      if (it_delim == recev_buffer_.end()) break;

      std::size_t frame_len = std::distance(recev_buffer_.begin(), it_delim);

      if (frame_len == 3) {
        uint8_t data_byte_1 = recev_buffer_[0] - 48;
        uint8_t data_byte_2 = recev_buffer_[2] - 48;

        // publish each ciolor value
        std_msgs::msg::UInt8MultiArray msg_pub;
        msg_pub.data.push_back(data_byte_1);
        msg_pub.data.push_back(data_byte_2);
        color_pub_->publish(msg_pub);

        // decide color value
        // 0: none, 1: red, 2: yellow, 3: blue
        // -> 0: red, 1: yellow, 2: blue, none
        std_msgs::msg::UInt8 msg_srv;
        msg_srv.data = (std::max(data_byte_1, data_byte_2) - 1) % 3;
        color_response_.color = msg_srv.data;

        RCLCPP_INFO(this->get_logger(), "Received color data: %d, %d -> color: %d", data_byte_1, data_byte_2, msg_srv.data);
      }

      recev_buffer_.erase(recev_buffer_.begin(), it_delim + 2);
    }
  }
}

void koma::Color::color_callback(
  const std::shared_ptr<inrof2026_koma_type::srv::Color::Request> request,
  const std::shared_ptr<inrof2026_koma_type::srv::Color::Response> response)
{
  (void)request;
  response->color = color_response_.color;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<koma::Color>());
  rclcpp::shutdown();
  return 0;
}