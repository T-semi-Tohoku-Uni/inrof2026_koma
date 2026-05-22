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
    fcntl(fd, F_SETFL, O_NONBLOCK);

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

bool koma::FeetechSerial::send_write_state_command(double position) {
    auto logger = rclcpp::get_logger("rclcpp");

    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger, "Serial port is not open");
        return false;
    }

    const uint8_t servo_id = static_cast<uint8_t>(servo_id_);

    constexpr uint8_t instruction = 0x03;      // WRITE
    constexpr uint8_t goal_position_addr = 42; // Goal Position L

    // position は raw tick として扱う
    if (position < 0.0) {
        position = 0.0;
    }

    if (position > 4095.0) {
        position = 4095.0;
    }

    const uint16_t goal_position =
        static_cast<uint16_t>(std::lround(position));

    const uint8_t pos_l = static_cast<uint8_t>(goal_position & 0xFF);
    const uint8_t pos_h = static_cast<uint8_t>((goal_position >> 8) & 0xFF);

    // Packet:
    // FF FF ID LENGTH INSTRUCTION ADDRESS PARAMS... CHECKSUM
    //
    // params = address + pos_l + pos_h = 3 bytes
    // length = instruction + params + checksum = 1 + 3 + 1 = 5
    uint8_t tx[13];
    std::memset(tx, 0x00, sizeof(tx));

    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = servo_id;
    tx[3] = 0x09;              // length
    tx[4] = instruction;       // 0x03 WRITE
    tx[5] = goal_position_addr; // 42
    tx[6] = pos_l;
    tx[7] = pos_h;
    tx[8] = 0x00;              // time low
    tx[9] = 0x00;              // time high
    tx[10] = 0x00;             // speed low
    tx[11] = 0x00;             // speed high
    tx[12] = make_checksum(&tx[2], 10); // IDからspeed highまで

    tcflush(serial_fd_, TCIFLUSH);

    if (!write_packet(tx, sizeof(tx))) return false;

    RCLCPP_INFO(
        logger,
        "WRITE position command sent: id=%u position=%u",
        static_cast<unsigned>(servo_id),
        static_cast<unsigned>(goal_position)
    );

    return true;
}

bool koma::FeetechSerial::send_read_state_command()
{
    auto logger = rclcpp::get_logger("rclcpp");

    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger, "Serial port is not open");
        return false;
    }

    const uint8_t servo_id = static_cast<uint8_t>(servo_id_);
    constexpr uint8_t instruction = 0x02;     // READ
    constexpr uint8_t start_address = 0x38;   // 56: Present Position
    constexpr uint8_t read_size = 0x06;       // position, speed, load

    uint8_t tx[8];
    std::memset(tx, 0x00, sizeof(tx));

    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = servo_id;
    tx[3] = 0x04;
    tx[4] = instruction;
    tx[5] = start_address;
    tx[6] = read_size;
    tx[7] = make_checksum(&tx[2], 5);

    rx_buffer_.clear();
    tcflush(serial_fd_, TCIFLUSH);

    if (!write_packet(tx, 8)) return false;

    return true;
}

bool koma::FeetechSerial::try_read_state_response(koma::FeetechState& state)
{
    auto logger = rclcpp::get_logger("rclcpp");

    if (serial_fd_ < 0) {
        RCLCPP_ERROR(logger, "Serial port is not open");
        return false;
    }

    uint8_t tmp[64];

    while (true) {
        const ssize_t n = ::read(serial_fd_, tmp, sizeof(tmp));

        if (n > 0) {
            rx_buffer_.insert(
                rx_buffer_.end(),
                tmp,
                tmp + n
            );
            continue;
        }

        if (n == 0) {
            break;
        }

        if (errno == EINTR) {
            continue;
        }

        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            break;
        }

        RCLCPP_ERROR(
            logger,
            "Serial read failed: %s",
            std::strerror(errno)
        );
        rx_buffer_.clear();
        return false;
    }

    // header FF FF を探す
    while (rx_buffer_.size() >= 2) {
        if (rx_buffer_[0] == 0xFF && rx_buffer_[1] == 0xFF) {
            break;
        }

        rx_buffer_.erase(rx_buffer_.begin());
    }

    if (rx_buffer_.size() < 4) {
        return false;
    }

    const uint8_t servo_id = static_cast<uint8_t>(servo_id_);
    const uint8_t rx_id = rx_buffer_[2];
    const uint8_t length = rx_buffer_[3];

    // total packet:
    // FF FF ID LENGTH ERROR PARAMS... CHECKSUM
    const size_t packet_size = static_cast<size_t>(length) + 4;

    if (rx_buffer_.size() < packet_size) {
        return false;
    }

    if (rx_id != servo_id) {
        RCLCPP_WARN(
            logger,
            "Unexpected servo id: received=%u expected=%u",
            rx_id,
            servo_id
        );

        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + packet_size);
        return false;
    }

    if (length == 0x02) {
        const uint8_t error = rx_buffer_[4];
        const uint8_t received_checksum = rx_buffer_[packet_size - 1];

        uint16_t checksum_sum = 0;
        for (size_t i = 2; i < packet_size - 1; ++i) {
            checksum_sum += rx_buffer_[i];
        }

        const uint8_t expected_checksum =
            static_cast<uint8_t>(~static_cast<uint8_t>(checksum_sum & 0xFF));

        if (received_checksum != expected_checksum) {
            RCLCPP_WARN(
                logger,
                "WRITE ACK checksum mismatch: received=0x%02X expected=0x%02X",
                received_checksum,
                expected_checksum
            );
        } else if (error != 0x00) {
            RCLCPP_WARN(
                logger,
                "WRITE ACK returned error: 0x%02X",
                error
            );
        } else {
            RCLCPP_DEBUG(logger, "Ignored WRITE ACK packet");
        }
        
        RCLCPP_WARN(
            logger,
            "RX buffer head: %s",
            bytes_to_hex(rx_buffer_, std::min<size_t>(rx_buffer_.size(), 32)).c_str()
        );

        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + packet_size);
        return false;
    }

    constexpr uint8_t read_size = 0x06;

    if (length != read_size + 2) {
        RCLCPP_WARN(
            logger,
            "Unexpected response length: received=%u expected=%u",
            length,
            static_cast<unsigned>(read_size + 2)
        );

        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + packet_size);
        return false;
    }

    const uint8_t error = rx_buffer_[4];

    if (error != 0x00) {
        RCLCPP_WARN(
            logger,
            "Servo returned error: 0x%02X",
            error
        );

        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + packet_size);
        return false;
    }

    const uint8_t received_checksum = rx_buffer_[packet_size - 1];

    uint16_t checksum_sum = 0;
    for (size_t i = 2; i < packet_size - 1; ++i) {
        checksum_sum += rx_buffer_[i];
    }

    const uint8_t expected_checksum =
        static_cast<uint8_t>(~static_cast<uint8_t>(checksum_sum & 0xFF));

    if (received_checksum != expected_checksum) {
        RCLCPP_WARN(
            logger,
            "Checksum mismatch: received=0x%02X expected=0x%02X",
            received_checksum,
            expected_checksum
        );

        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + packet_size);
        return false;
    }

    auto u16_from_data = [&](size_t offset) -> uint16_t {
        constexpr size_t data_start = 5;

        return static_cast<uint16_t>(
            static_cast<uint16_t>(rx_buffer_[data_start + offset]) |
            (static_cast<uint16_t>(rx_buffer_[data_start + offset + 1]) << 8)
        );
    };

    state.id = rx_id;
    state.present_position = u16_from_data(0);
    state.present_speed = u16_from_data(2);
    state.present_load = u16_from_data(4);

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + packet_size);

    return true;
}

bool koma::FeetechSerial::read_all_state(koma::FeetechState& state)
{
    if (serial_fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Serial port is not open");
        return false;
    }

    const uint8_t servo_id = static_cast<uint8_t>(servo_id_);
    constexpr uint8_t instruction = 0x02;   // READ
    constexpr uint8_t start_address = 0x00;
    constexpr uint8_t read_size = 0x47;     // 71 bytes: addr 0〜70

    uint8_t tx[8];
    std::memset(tx, 0x00, sizeof(tx));

    tx[0] = 0xFF;
    tx[1] = 0xFF;
    tx[2] = servo_id;
    tx[3] = 0x04;
    tx[4] = instruction;
    tx[5] = start_address;
    tx[6] = read_size;
    tx[7] = make_checksum(&tx[2], 5);

    RCLCPP_INFO(
        rclcpp::get_logger("rclcpp"),
        "TX: %02X %02X %02X %02X %02X %02X %02X %02X",
        tx[0], tx[1], tx[2], tx[3], tx[4], tx[5], tx[6], tx[7]
    );

    tcflush(serial_fd_, TCIFLUSH);

    if (!write_packet(tx, 8)) return false;

    constexpr size_t expected_rx_size = 6 + read_size;
    uint8_t rx[128];
    std::memset(rx, 0x00, sizeof(rx));

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
            if (errno == EINTR) {
                continue;
            }

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

    if (received < expected_rx_size) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "RX timeout/incomplete: %zu / %zu bytes",
            received,
            expected_rx_size
        );
        return false;
    }

    if (rx[0] != 0xFF || rx[1] != 0xFF) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid response header");
        return false;
    }

    if (rx[2] != servo_id) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Unexpected servo id: received=%u expected=%u",
            rx[2],
            servo_id
        );
        return false;
    }

    uint8_t length = rx[3];
    uint8_t error = rx[4];

    if (length != read_size + 2) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Unexpected response length: received=%u expected=%u",
            length,
            static_cast<unsigned>(read_size + 2)
        );
        return false;
    }

    if (error != 0x00) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Servo returned error: 0x%02X",
            error
        );
        return false;
    }

    const uint8_t received_checksum = rx[expected_rx_size - 1];

    uint16_t checksum_sum = 0;
    for (size_t i = 2; i < expected_rx_size - 1; ++i) {
        checksum_sum += rx[i];
    }

    const uint8_t expected_checksum =
        static_cast<uint8_t>(~static_cast<uint8_t>(checksum_sum & 0xFF));

    if (received_checksum != expected_checksum) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Checksum mismatch: received=0x%02X expected=0x%02X",
            received_checksum,
            expected_checksum
        );
        return false;
    }

    // DATA は rx[5] から始まる。
    // address 0 から読んでいるので rx[5 + addr] がそのアドレスの値。
    auto u8 = [&](size_t addr) -> uint8_t {
        return rx[5 + addr];
    };

    auto u16 = [&](size_t addr) -> uint16_t {
        return static_cast<uint16_t>(
            static_cast<uint16_t>(rx[5 + addr]) |
            (static_cast<uint16_t>(rx[5 + addr + 1]) << 8)
        );
    };

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

    return true;
}

uint8_t koma::FeetechSerial::make_checksum(uint8_t tx[], size_t size) {
    uint16_t sum = 0;

    for (size_t i = 0; i < size; ++i) {
        sum += tx[i];
    }

    return static_cast<uint8_t>(~static_cast<uint8_t>(sum & 0xFF));
}

bool koma::FeetechSerial::write_packet(uint8_t tx[], size_t size) {
    ssize_t written = ::write(serial_fd_, tx, size);

    if (written < 0) {
        RCLCPP_ERROR(
            rclcpp::get_logger("rclcpp"),
            "Serial write failed: %s",
            std::strerror(errno)
        );
        return false;
    }

    if (written != static_cast<ssize_t>(size)) {
        RCLCPP_WARN(
            rclcpp::get_logger("rclcpp"),
            "Serial write incomplete: %zd / %zu bytes",
            written,
            size
        );
        return false;
    }

    return true;
}

std::string koma::FeetechSerial::bytes_to_hex(const std::vector<uint8_t>& buffer, size_t size)
{
    std::ostringstream oss;

    const size_t n = std::min(size, buffer.size());
    for (size_t i = 0; i < n; ++i) {
        oss << std::hex << std::uppercase
            << std::setw(2) << std::setfill('0')
            << static_cast<int>(buffer[i]) << " ";
    }

    return oss.str();
}

int main(int argc, char ** argv)
{
    koma::FeetechSerial serial(
        "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.1.1:1.0",
        1
    );

    koma::FeetechState state;
    if (!serial.read_all_state(state)) {}
    state.print();

    std::vector<double> positions = {
        1000.0,
        2000.0,
        3000.0,
        2000.0
    };

    size_t index = 0;

    while (true) {
        double position = positions[index];
        bool ok = serial.send_write_state_command(position);

        // if (ok) {
        //     std::cout << "sent position: " << position << std::endl;
        // } else {
        //     std::cerr << "failed to send position: " << position << std::endl;
        // }

        if (!serial.send_read_state_command()){}
        while (!serial.try_read_state_response(state)) {}
        state.print();

        index = (index + 1) % positions.size();

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}