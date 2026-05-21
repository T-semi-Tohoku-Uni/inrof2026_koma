#include <feetech_servo/sts3125.hpp>

koma::FeetechSerial::FeetechSerial(const char* device_name) {
    serial_fd_ = open_serial(device_name);
}

int koma::FeetechSerial::open_serial(const char* device_name) {
    int fd = ::open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                        "Serial Fail: could not open %s (%s)",
                        device_name, std::strerror(errno));
        return -1;
    }
    fcntl(fd, F_SETFL, 0);

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                        "Serial Fail: tcgetattr error (%s)", std::strerror(errno));
        ::close(fd);
        return -1;
    }

    cfsetispeed(&tty, B1000000);
    cfsetospeed(&tty, B1000000);
    cfmakeraw(&tty);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= (CS8 | CLOCAL | CREAD);

    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                        "Serial Fail: tcsetattr error (%s)", std::strerror(errno));
        ::close(fd);
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Serial opened: %s @ 1000000, 8N1, raw", device_name);
    return fd;
}

koma::FeetechState koma::FeetechSerial::send_read_state_command() {

    // TODO: exclude controle

    // write read command
    uint8_t buf[8];
    memset(buf, 0x00, sizeof(buf));
    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = 0x01; // servo id. TODO
    buf[3] = 0x04;
    buf[4] = 0x02; // read command
    buf[5] = 0x00;
    buf[6] = 0x47;

    uint8_t sum = buf[2] + buf[3] + buf[4] + buf[5] + buf[6];
    buf[7] = ~sum;

    ::write(serial_fd_, buf, sizeof(buf));

    // TODO
    return koma::FeetechState();
}

uint8_t koma::FeetechSerial::make_checksum(uint8_t buf[8]) {}

int main(int argc, char ** argv)
{
    koma::FeetechSerial serial = koma::FeetechSerial("/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.1:1.0");

    while (1) {
        koma::FeetechState state = serial.send_read_state_command();

        std::cout << "send_read_state_command()" << std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}