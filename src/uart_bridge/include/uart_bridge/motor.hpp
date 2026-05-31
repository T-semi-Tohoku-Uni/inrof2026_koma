#include <rclcpp/rclcpp.hpp>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <error.h>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace koma {
    typedef union {
        uint8_t byte[4];
        float value;
    } U32Bytes;

    typedef struct MotorVel {
        float v1;
        float v2;
        float v3;
    } MotorVel;

    class TwistManager: public rclcpp::Node {
        public:
            TwistManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
        private:
            void acceleration_control(geometry_msgs::msg::Twist::SharedPtr msg); 
            void cascade_control();
            void send_vel(geometry_msgs::msg::Twist msg);
            void receive_vel_callback();
            int open_serial(const char *device_name);
            MotorVel forwardKinematics(float vx, float vy, float vtheta);
            geometry_msgs::msg::Twist inverseKinematics(float v1, float v2, float v3);

            int fd_vel_;
            float r_;
            double Kp_linear_, Kp_angular_;
            double max_linear_acceleration_;
            double max_angular_acceleration_;
            double dt_;
            std::vector<uint8_t> recev_buffer_;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
            rclcpp::TimerBase::SharedPtr receive_timer_;
            rclcpp::TimerBase::SharedPtr control_timer_;
            geometry_msgs::msg::Twist cmd_vel_;
            geometry_msgs::msg::Twist cmd_vel_prime_;
            geometry_msgs::msg::Twist cmd_vel_feedback_;
    };
}