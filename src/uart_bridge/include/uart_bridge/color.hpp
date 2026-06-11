#include <rclcpp/rclcpp.hpp>
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
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <inrof2026_koma_type/srv/color.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

namespace koma {
    class Color: public rclcpp::Node {
        public: 
            Color(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        private:
            void color_callback(
                const std::shared_ptr<inrof2026_koma_type::srv::Color::Request> request,
                const std::shared_ptr<inrof2026_koma_type::srv::Color::Response> response
            );
            int open_serial(const char * device_name);
            void receive_color_from_serial();

            int serial_fd_;

            // color
            rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr color_pub_;
            rclcpp::TimerBase::SharedPtr receive_timer_;
            std::vector<uint8_t> recev_buffer_;

            // vacume
            rclcpp::Service<inrof2026_koma_type::srv::Color>::SharedPtr color_srv_;
            inrof2026_koma_type::srv::Color::Response color_response_;

    };
}