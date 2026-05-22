#include <feetech_servo/sts3125.hpp>

void koma::FeetechState::print() {
    auto logger = rclcpp::get_logger("rclcpp");
    RCLCPP_INFO(logger, "===== FEETECH STATE =====");

    RCLCPP_INFO(logger, "Model L               : %u", static_cast<unsigned>(model_l));
    RCLCPP_INFO(logger, "Model H               : %u", static_cast<unsigned>(model_h));
    RCLCPP_INFO(logger, "Firmware Version      : %u", static_cast<unsigned>(firmware_version));
    RCLCPP_INFO(logger, "ID                    : %u", static_cast<unsigned>(id));
    RCLCPP_INFO(logger, "Baudrate              : %u", static_cast<unsigned>(baudrate));
    RCLCPP_INFO(logger, "Return Delay          : %u", static_cast<unsigned>(return_delay));
    RCLCPP_INFO(logger, "Response Status Level : %u", static_cast<unsigned>(response_status_level));

    RCLCPP_INFO(logger, "Min Angle Limit       : %u", static_cast<unsigned>(min_angle_limit));
    RCLCPP_INFO(logger, "Max Angle Limit       : %u", static_cast<unsigned>(max_angle_limit));

    RCLCPP_INFO(logger, "Max Temperature Limit : %u degC", static_cast<unsigned>(max_temperature_limit));
    RCLCPP_INFO(
        logger,
        "Max Voltage Limit     : %.1f V raw=%u",
        static_cast<double>(max_voltage_limit) / 10.0,
        static_cast<unsigned>(max_voltage_limit)
    );
    RCLCPP_INFO(
        logger,
        "Min Voltage Limit     : %.1f V raw=%u",
        static_cast<double>(min_voltage_limit) / 10.0,
        static_cast<unsigned>(min_voltage_limit)
    );

    RCLCPP_INFO(logger, "Max Torque            : %u", static_cast<unsigned>(max_torque));

    RCLCPP_INFO(logger, "P Coefficient         : %u", static_cast<unsigned>(p_coefficient));
    RCLCPP_INFO(logger, "D Coefficient         : %u", static_cast<unsigned>(d_coefficient));
    RCLCPP_INFO(logger, "I Coefficient         : %u", static_cast<unsigned>(i_coefficient));

    RCLCPP_INFO(logger, "Mode                  : %u", static_cast<unsigned>(mode));

    RCLCPP_INFO(logger, "Torque Enable         : %u", static_cast<unsigned>(torque_enable));
    RCLCPP_INFO(logger, "Acceleration          : %u", static_cast<unsigned>(acceleration));

    RCLCPP_INFO(logger, "Goal Position         : %u", static_cast<unsigned>(goal_position));
    RCLCPP_INFO(logger, "Goal Time             : %u", static_cast<unsigned>(goal_time));
    RCLCPP_INFO(logger, "Goal Speed            : %u", static_cast<unsigned>(goal_speed));
    RCLCPP_INFO(logger, "Torque Limit          : %u", static_cast<unsigned>(torque_limit));

    RCLCPP_INFO(logger, "EPROM Lock            : %u", static_cast<unsigned>(eprom_lock));

    RCLCPP_INFO(logger, "Present Position      : %u", static_cast<unsigned>(present_position));
    RCLCPP_INFO(logger, "Present Speed         : %u", static_cast<unsigned>(present_speed));
    RCLCPP_INFO(logger, "Present Load          : %u", static_cast<unsigned>(present_load));

    RCLCPP_INFO(
        logger,
        "Present Voltage       : %.1f V raw=%u",
        static_cast<double>(present_voltage) / 10.0,
        static_cast<unsigned>(present_voltage)
    );
    RCLCPP_INFO(logger, "Present Temperature   : %u degC", static_cast<unsigned>(present_temperature));
    RCLCPP_INFO(logger, "Async Write Flag      : %u", static_cast<unsigned>(async_write_flag));
    RCLCPP_INFO(logger, "Servo Status          : %u", static_cast<unsigned>(servo_status));
    RCLCPP_INFO(logger, "Moving                : %u", static_cast<unsigned>(moving));

    RCLCPP_INFO(logger, "Present Current       : %u", static_cast<unsigned>(present_current));

    RCLCPP_INFO(logger, "=========================");
}

koma::FeetechSerial::FeetechSerial(const char* device_name, int servo_id) {
    serial_fd_ = open_serial(device_name);
    servo_id_ = servo_id;
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

koma::FeetechState koma::FeetechSerial::send_ping_command() {
    uint8_t servo_id = static_cast<uint8_t>(servo_id_);
    uint8_t instruction = 0x02; // READ
    uint8_t start_address = 0x00;
    uint8_t read_size = 0x4A; // 74 bytes

    uint8_t tx[8];
    std::memset(tx, 0x00, sizeof(tx));

    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = servo_id;
    tx[3] = 0x04; // length = instruction + params + checksum
    tx[4] = instruction;
    tx[5] = start_address;
    tx[6] = read_size;

    uint8_t sum = tx[2] + tx[3] + tx[4] + tx[5] + tx[6];
    tx[7] = static_cast<uint8_t>(~sum);

    tcflush(serial_fd_, TCIFLUSH);

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
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Serial write incomplete: written=%zd expected=%zu",
            written,
            sizeof(tx)
        );
        return koma::FeetechState();
    }

    // ---- read status packet ----
    // Format:
    //   0xFF 0xFF ID LENGTH ERROR PARAMS... CHECKSUM

    uint8_t b = 0;
    bool found_first_ff = false;

    while (true) {
        if (!read_exact(&b, 1)) {
            return koma::FeetechState();
        }

        if (b == 0xFF) {
            if (found_first_ff) {
                break;
            }

            found_first_ff = true;
        } else {
            found_first_ff = false;
        }
    }

    uint8_t rx_id = 0;
    uint8_t length = 0;
    uint8_t error = 0;

    if (!read_exact(&rx_id, 1) ||
        !read_exact(&length, 1) ||
        !read_exact(&error, 1)) {
        return koma::FeetechState();
    }

    if (rx_id != servo_id) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Unexpected servo ID: received=%u expected=%u",
            rx_id,
            servo_id
        );
        return koma::FeetechState();
    }

    if (length < 2) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Invalid status packet length: %u",
            length
        );
        return koma::FeetechState();
    }

    const size_t params_size = static_cast<size_t>(length) - 2;
    std::vector<uint8_t> params(params_size);

    if (params_size > 0) {
        if (!read_exact(params.data(), params_size)) {
            return koma::FeetechState();
        }
    }

    uint8_t received_checksum = 0;
    if (!read_exact(&received_checksum, 1)) {
        return koma::FeetechState();
    }

    uint16_t checksum_sum = 0;
    checksum_sum += rx_id;
    checksum_sum += length;
    checksum_sum += error;

    for (uint8_t p : params) {
        checksum_sum += p;
    }

    uint8_t expected_checksum =
        static_cast<uint8_t>(~static_cast<uint8_t>(checksum_sum & 0xFF));

    if (received_checksum != expected_checksum) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Checksum mismatch: received=0x%02X expected=0x%02X",
            received_checksum,
            expected_checksum
        );
        return koma::FeetechState();
    }

    if (error != 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Servo returned error: 0x%02X",
            error
        );
        return koma::FeetechState();
    }

    if (params.size() != read_size) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Unexpected read size: got=%zu expected=%u",
            params.size(),
            read_size
        );
        return koma::FeetechState();
    }

    // ---- decode params ----
    // params[0] corresponds to address 0x00.

    auto u8 = [&](size_t addr) -> uint8_t {
        return params.at(addr);
    };

    auto u16 = [&](size_t addr) -> uint16_t {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(params.at(addr)) |
            (static_cast<uint16_t>(params.at(addr + 1)) << 8)
        );
    };

    koma::FeetechState state;

    state.model_l = u8(0);
    state.model_h = u8(1);
    state.firmware_version = u8(3);
    state.id = u8(5);
    state.baudrate = u8(6);
    state.return_delay = u8(7);
    state.response_status_level = u8(8);

    state.min_angle_limit = u16(9);
    state.max_angle_limit = u16(11);

    state.max_temperature_limit = u8(13);
    state.max_voltage_limit = u8(14);
    state.min_voltage_limit = u8(15);

    state.max_torque = u16(16);

    state.p_coefficient = u8(21);
    state.d_coefficient = u8(22);
    state.i_coefficient = u8(23);

    state.mode = u8(33);

    state.torque_enable = u8(40);
    state.acceleration = u8(41);

    state.goal_position = u16(42);
    state.goal_time = u16(44);
    state.goal_speed = u16(46);
    state.torque_limit = u16(48);

    state.eprom_lock = u8(55);

    state.present_position = u16(56);
    state.present_speed = u16(58);
    state.present_load = u16(60);

    state.present_voltage = u8(62);
    state.present_temperature = u8(63);
    state.async_write_flag = u8(64);
    state.servo_status = u8(65);
    state.moving = u8(66);

    state.present_current = u16(69);

    return state;
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
    koma::FeetechSerial serial = koma::FeetechSerial("/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.1:1.0", 0);

    while (1) {
        koma::FeetechState state = serial.send_ping_command();
        state.print();
    }
}