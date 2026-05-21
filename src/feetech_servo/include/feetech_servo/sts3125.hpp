#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <error.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <iterator>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>

namespace koma {
    struct FeetechAddr {
        static constexpr uint8_t MODEL_L = 0;
        static constexpr uint8_t MODEL_H = 1;
        static constexpr uint8_t FIRMWARE_VERSION = 3;
        static constexpr uint8_t ID = 5;
        static constexpr uint8_t BAUDRATE = 6;
        static constexpr uint8_t RETURN_DELAY = 7;
        static constexpr uint8_t RESPONSE_STATUS_LEVEL = 8;

        static constexpr uint8_t MIN_ANGLE_LIMIT = 9;
        static constexpr uint8_t MAX_ANGLE_LIMIT = 11;

        static constexpr uint8_t MAX_TEMPERATURE_LIMIT = 13;
        static constexpr uint8_t MAX_VOLTAGE_LIMIT = 14;
        static constexpr uint8_t MIN_VOLTAGE_LIMIT = 15;

        static constexpr uint8_t MAX_TORQUE = 16;

        static constexpr uint8_t P_COEFFICIENT = 21;
        static constexpr uint8_t D_COEFFICIENT = 22;
        static constexpr uint8_t I_COEFFICIENT = 23;

        static constexpr uint8_t MODE = 33;

        static constexpr uint8_t TORQUE_ENABLE = 40;
        static constexpr uint8_t ACCELERATION = 41;

        static constexpr uint8_t GOAL_POSITION = 42;
        static constexpr uint8_t GOAL_TIME = 44;
        static constexpr uint8_t GOAL_SPEED = 46;
        static constexpr uint8_t TORQUE_LIMIT = 48;

        static constexpr uint8_t EPROM_LOCK = 55;

        static constexpr uint8_t PRESENT_POSITION = 56;
        static constexpr uint8_t PRESENT_SPEED = 58;
        static constexpr uint8_t PRESENT_LOAD = 60;
        static constexpr uint8_t PRESENT_VOLTAGE = 62;
        static constexpr uint8_t PRESENT_TEMPERATURE = 63;
        static constexpr uint8_t ASYNC_WRITE_FLAG = 64;
        static constexpr uint8_t SERVO_STATUS = 65;
        static constexpr uint8_t MOVING = 66;
        static constexpr uint8_t PRESENT_CURRENT = 69;
    };

    class FeetechState {
    public:
        FeetechState() = default;

        uint8_t model_l = 0;
        uint8_t model_h = 0;
        uint8_t firmware_version = 0;
        uint8_t id = 0;
        uint8_t baudrate = 0;
        uint8_t return_delay = 0;
        uint8_t response_status_level = 0;

        uint16_t min_angle_limit = 0;
        uint16_t max_angle_limit = 0;

        uint8_t max_temperature_limit = 0;
        uint8_t max_voltage_limit = 0;
        uint8_t min_voltage_limit = 0;

        uint16_t max_torque = 0;

        uint8_t p_coefficient = 0;
        uint8_t d_coefficient = 0;
        uint8_t i_coefficient = 0;

        uint8_t mode = 0;

        uint8_t torque_enable = 0;
        uint8_t acceleration = 0;

        uint16_t goal_position = 0;
        uint16_t goal_time = 0;
        uint16_t goal_speed = 0;
        uint16_t torque_limit = 0;

        uint8_t eprom_lock = 0;

        uint16_t present_position = 0;
        uint16_t present_speed = 0;
        uint16_t present_load = 0;

        uint8_t present_voltage = 0;
        uint8_t present_temperature = 0;
        uint8_t async_write_flag = 0;
        uint8_t servo_status = 0;
        uint8_t moving = 0;

        uint16_t present_current = 0;
    };

    class FeetechSerial {
        public:
            FeetechSerial(const char* device_name);
            koma::FeetechState send_read_state_command();
        private:
            int serial_fd_;

            int open_serial(const char* device_name);
            uint8_t make_checksum(uint8_t buf[]);
            uint16_t make_u16(uint8_t low, uint8_t high) {
                return static_cast<uint16_t>(low) | (static_cast<uint16_t>(high) << 8);
            }
            
            // bool write();
            // FeetechJointState read();
    };
}