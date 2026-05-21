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
    class FeetechJointState {
        public:
            FeetechJointState(int id);
        private:
            int id_;
            double position_;
            double velocity_;
            double effort_;
    };

    class FeetechSerial {
        public:
            FeetechSerial(const char* device_name);
        private:
            int serial_fd_;
            int open_serial(const char* device_name);
            // bool write();
            // FeetechJointState read();
    };
}