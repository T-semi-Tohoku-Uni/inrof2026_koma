#include "komarm/catch_influence.hpp"

// 入力22次元　ジョイントの位置、速度１０次元、ハンドの先端の位置７次元、１つ前のアクション５次元
// 出力5次元　アクション５次元　

namespace koma{

    //define the constructor of the node
    CatchInfluence::CatchInfluence(const rclcpp::NodeOptions & options) : Node("catch_influence", options){
        RCLCPP_INFO(this->get_logger(), "CatchInfluence node has been started.");
        
        // Load the model
        model_path_ = "/home/daikou/komarm/logs/rsl_rl/reach/2026-05-16_16-41-15/exported/policy.pt";
        module_ = load_model(model_path_);
        //inference mode
        module_.eval();
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully.");


        //test
        //一旦obsに0を入れる
        torch::Tensor obs = torch::zeros({1, 22});
        torch::Tensor action = inference(obs);
        std::cout << "Action: " << action << std::endl;
        std::cout << action.sizes() << std::endl;

    }

    //define the function that loads the parameters of the model
    torch::jit::script::Module CatchInfluence::load_model(const std::string& model_path) {
        torch::jit::script::Module module;

        try{
            module = torch::jit::load(model_path);
        }
        catch (const c10::Error& e) {
            RCLCPP_ERROR(rclcpp::get_logger("CatchInfluence"), "Error loading the model: %s", e.what());
            throw;
        }

        return module;
    }

    //define the function that inferes the action from the observation
    torch::Tensor CatchInfluence::inference(const torch::Tensor& obs) {

        // Disable gradient calculation for inference
        torch::NoGradGuard no_grad; 

        // forward the observation
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(obs);
        torch::Tensor action = module_.forward(inputs).toTensor();

        return action;
    }




 
    
}





int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<koma::CatchInfluence>());
    rclcpp::shutdown();

    return 0;
}