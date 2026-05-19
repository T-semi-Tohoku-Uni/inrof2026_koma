#pragma once
#include <rclcpp/rclcpp.hpp>
#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>


namespace koma{
    class CatchInfluence : public rclcpp::Node{
        public:
            CatchInfluence(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

        private:
            torch::jit::script::Module module_;
            std::string model_path_;
            torch::jit::script::Module load_model(const std::string& model_path);
            torch::Tensor inference(const torch::Tensor& obs);
        


    };

}