#include "komarm/catch_influence.hpp"

// 入力22次元　ジョイントの位置、速度１０次元、ハンドの先端の位置７次元、１つ前のアクション５次元
// 出力5次元　アクション５次元　

namespace koma{

    //define the constructor of the node
    CatchInfluence::CatchInfluence(const rclcpp::NodeOptions & options) : Node("catch_influence", options){
        RCLCPP_INFO(this->get_logger(), "CatchInfluence node has been started.");
        
        // create a timer for the control loop
        control_timer_ = this->create_wall_timer(100ms, std::bind(&CatchInfluence::control_loop, this));

        // Initialize publishers and subscribers
        target_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "target_joint_states", rclcpp::SensorDataQoS()
        );

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", rclcpp::SensorDataQoS(), std::bind(&CatchInfluence::joint_callback, this, std::placeholders::_1)
        );

        hand_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "hand_pose", rclcpp::SensorDataQoS(), std::bind(&CatchInfluence::hand_callback, this, std::placeholders::_1)
        );

        // Initialize previous action
        pre_action_ = sensor_msgs::msg::JointState();
        pre_action_.name = {"Revolute 12", "Revolute 11", "Revolute 7", "Revolute 8", "Revolute 9"};
        pre_action_.position = {0.0, 0.0, 0.0, 0.0, 0.0};

        // Load the model
        model_path_ = "/home/daikou/komarm/logs/rsl_rl/reach/2026-05-16_16-41-15/exported/policy.pt";
        module_ = load_model(model_path_);
        //inference mode
        module_.eval();
        RCLCPP_INFO(this->get_logger(), "Model loaded successfully.");
        
    }

    void CatchInfluence::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        cur_joint_state_ = *msg;
    }

    void CatchInfluence::hand_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        cur_hand_pose_ = *msg;
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

    //main control loop
    void CatchInfluence::control_loop() {
        
        //create states
        //influence 
        //publish action
        //pre action = action


         
        //create states
        torch::Tensor obs = torch::zeros({22}); 

        //influence
        torch::Tensor action = inference(obs);

        //publish action
        pre_action_.header.stamp = this->get_clock()->now();
        pre_action_.position = {action[0].item<double>(), action[1].item<double>(), action[2].item<double>(), action[3].item<double>(), action[4].item<double>()};
        target_joint_pub_->publish(pre_action_);
        
        
        

    }
}



int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<koma::CatchInfluence>());
    rclcpp::shutdown();

    return 0;
}