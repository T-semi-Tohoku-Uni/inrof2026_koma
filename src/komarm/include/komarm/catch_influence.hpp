#pragma once
#include <rclcpp/rclcpp.hpp>
#include <torch/torch.h>
#include <torch/script.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>

using namespace std::chrono_literals;


namespace koma{
    class CatchInfluence : public rclcpp::Node{
        public:
            CatchInfluence(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        private:
        
            //variables
            torch::jit::script::Module module_;
            std::string model_path_;
            sensor_msgs::msg::JointState cur_joint_state_;
            geometry_msgs::msg::PoseStamped cur_hand_pose_;
            sensor_msgs::msg::JointState pre_action_;
            rclcpp::TimerBase::SharedPtr control_timer_;

            //functions
            torch::jit::script::Module load_model(const std::string& model_path);
            torch::Tensor inference(const torch::Tensor& obs);
            void control_loop();
            

            //publishers
            rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr target_joint_pub_;

            //subscribers
            rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr hand_sub_;

            //callback function
            void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
            void hand_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    };

}