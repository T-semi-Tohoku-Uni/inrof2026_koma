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

koma::FeetechState koma::FeetechSerial::send_read_state_command()
{
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial port is not open");
        return koma::FeetechState();
    }

    constexpr uint8_t servo_id = 0x01;
    constexpr uint8_t instruction = 0x02;   // READ
    constexpr uint8_t start_address = 0x00;
    constexpr uint8_t read_size = 0x47;     // 71 bytes: addr 0〜70

    uint8_t tx[8];
    memset(tx, 0x00, sizeof(tx));

    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = servo_id;
    tx[3] = 0x04;
    tx[4] = instruction;
    tx[5] = start_address;
    tx[6] = read_size;

    uint8_t sum = tx[2] + tx[3] + tx[4] + tx[5] + tx[6];
    tx[7] = static_cast<uint8_t>(~sum);

    std::cout << "TX: ";
    for (size_t i = 0; i < sizeof(tx); ++i) {
        printf("%02X ", tx[i]);
    }
    std::cout << std::endl;

    tcflush(serial_fd_, TCIFLUSH);  // 古い受信データを捨てる

    ssize_t written = ::write(serial_fd_, tx, sizeof(tx));
    if (written < 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Serial write failed: %s",
            std::strerror(errno)
        );
        return koma::FeetechState();
    }

    if (written != static_cast<ssize_t>(sizeof(tx))) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Serial write incomplete: %ld / %zu bytes",
            written,
            sizeof(tx)
        );
        return koma::FeetechState();
    }

    constexpr size_t expected_rx_size = 6 + read_size;  // header, id, length, error, data, checksum
    uint8_t rx[128];
    memset(rx, 0x00, sizeof(rx));

    size_t received = 0;

    auto start_time = std::chrono::steady_clock::now();
    constexpr auto timeout = std::chrono::milliseconds(10);

    while (received < expected_rx_size) {
        ssize_t n = ::read(
            serial_fd_,
            rx + received,
            expected_rx_size - received
        );

        if (n > 0) {
            received += static_cast<size_t>(n);
        } else if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // まだ来てないだけ
            } else {
                RCLCPP_ERROR(
                    rclcpp::get_logger("rclcpp"),
                    "Serial read failed: %s",
                    std::strerror(errno)
                );
                break;
            }
        }

        auto now = std::chrono::steady_clock::now();
        if (now - start_time > timeout) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "RX(" << received << "): ";
    for (size_t i = 0; i < received; ++i) {
        printf("%02X ", rx[i]);
    }
    std::cout << std::endl;

    if (received < expected_rx_size) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "RX timeout/incomplete: %zu / %zu bytes",
            received,
            expected_rx_size
        );
        return koma::FeetechState();
    }

    // 簡易チェック
    if (rx[0] != 0xFF || rx[1] != 0xFF) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid response header");
        return koma::FeetechState();
    }

    if (rx[2] != servo_id) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Unexpected servo id: %u",
            rx[2]
        );
        return koma::FeetechState();
    }

    uint8_t error = rx[4];
    if (error != 0x00) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Servo returned error: 0x%02X",
            error
        );
    }

    // DATA は rx[5] から始まる
    // address 0 から読んでいるので:
    // rx[5 + addr] がそのアドレスの値

    std::cout << "ID register: " << static_cast<int>(rx[5 + 5]) << std::endl;
    std::cout << "Baudrate: " << static_cast<int>(rx[5 + 6]) << std::endl;

    uint16_t present_position =
        static_cast<uint16_t>(rx[5 + 56]) |
        (static_cast<uint16_t>(rx[5 + 57]) << 8);

    uint16_t present_speed =
        static_cast<uint16_t>(rx[5 + 58]) |
        (static_cast<uint16_t>(rx[5 + 59]) << 8);

    uint16_t present_load =
        static_cast<uint16_t>(rx[5 + 60]) |
        (static_cast<uint16_t>(rx[5 + 61]) << 8);

    uint8_t present_voltage = rx[5 + 62];
    uint8_t present_temperature = rx[5 + 63];
    uint8_t moving = rx[5 + 66];

    uint16_t present_current =
        static_cast<uint16_t>(rx[5 + 69]) |
        (static_cast<uint16_t>(rx[5 + 70]) << 8);

    std::cout << "present_position: " << present_position << std::endl;
    std::cout << "present_speed: " << present_speed << std::endl;
    std::cout << "present_load: " << present_load << std::endl;
    std::cout << "present_voltage: " << static_cast<int>(present_voltage) << std::endl;
    std::cout << "present_temperature: " << static_cast<int>(present_temperature) << std::endl;
    std::cout << "moving: " << static_cast<int>(moving) << std::endl;
    std::cout << "present_current: " << present_current << std::endl;

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