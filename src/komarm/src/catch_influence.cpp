#include "komarm/catch_influence.hpp"

// 入力22次元　ジョイントの位置、速度１０次元、ハンドの先端の位置７次元、１つ前のアクション５次元
// 出力5次元　アクション５次元　

namespace koma
{

//define the constructor of the node
CatchInfluence::CatchInfluence(const rclcpp::NodeOptions & options)
: Node("catch_influence", options)
{
  RCLCPP_INFO(this->get_logger(), "CatchInfluence node has been started.");

  // create a timer for the control loop
  control_timer_ = this->create_wall_timer(33ms, std::bind(&CatchInfluence::control_loop, this));

  // Initialize publishers and subscribers
  target_joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "target_joint_states", rclcpp::SensorDataQoS());

  joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", rclcpp::SensorDataQoS(),
    std::bind(&CatchInfluence::joint_callback, this, std::placeholders::_1));

  // hand_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
  //     "hand_pose", rclcpp::SensorDataQoS(), std::bind(&CatchInfluence::hand_callback, this, std::placeholders::_1)
  // );
  hand_srv_ = this->create_service<inrof2026_koma_type::srv::PoseStamped>(
    "hand_pose",
    std::bind(&CatchInfluence::hand_callback, this, std::placeholders::_1, std::placeholders::_2));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize previous action
  pre_action_ = sensor_msgs::msg::JointState();
  pre_action_.name = {"Revolute_1", "Revolute_2", "Revolute_3",
                      "Revolute_4", "Revolute_5", "Revolute_6"};
  pre_action_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // Load the model
  const std::string model_path_ = this->declare_parameter<std::string>(
    "model_path", "/home/keigo/komarm/logs/rsl_rl/reach/2026-05-20_05-43-29/exported/policy.pt");

  module_ = load_model(model_path_);
  //inference mode
  module_.eval();
  RCLCPP_INFO(this->get_logger(), "Model loaded successfully.");
}

void CatchInfluence::joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  cur_joint_state_ = *msg;
  has_cur_joint_state_ = true;

  geometry_msgs::msg::TransformStamped base_to_hand_tf;
  try {
    base_to_hand_tf = tf_buffer_->lookupTransform(
      "base_link",        
      "hand_unit_v1_1",   
      tf2::TimePointZero  
    );
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(),
      "Could not transform base_link -> hand_unit_v1_1: %s",
      ex.what()
    );
    return;
  }

  cur_hand_pose_.pose.position.x = base_to_hand_tf.transform.translation.x;
  cur_hand_pose_.pose.position.y = base_to_hand_tf.transform.translation.y;
  cur_hand_pose_.pose.position.z = base_to_hand_tf.transform.translation.z;
  cur_hand_pose_.pose.orientation = base_to_hand_tf.transform.rotation;
}

void CatchInfluence::hand_callback(
  const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Request> request,
  const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Response> Response)
{
  RCLCPP_INFO(this->get_logger(), "Change hand pose");
  target_hand_pose_ = request->pose_stamped;
  has_target_hand_pose_ = true;
}

//define the function that loads the parameters of the model
torch::jit::script::Module CatchInfluence::load_model(const std::string & model_path)
{
  torch::jit::script::Module module;

  try {
    module = torch::jit::load(model_path);
  } catch (const c10::Error & e) {
    RCLCPP_ERROR(rclcpp::get_logger("CatchInfluence"), "Error loading the model: %s", e.what());
    throw;
  }

  return module;
}

//define the function that inferes the action from the observation
torch::Tensor CatchInfluence::inference(const torch::Tensor & obs)
{
  // Disable gradient calculation for inference
  torch::NoGradGuard no_grad;

  // forward the observation
  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(obs);
  torch::Tensor action = module_.forward(inputs).toTensor();

  return action;
}

//main control loop
void CatchInfluence::control_loop()
{
  //create states
  //influence
  //publish action
  //pre action = action

  if (!has_cur_joint_state_) {
    RCLCPP_WARN(this->get_logger(), "cur_joint_state is empty");
    return;
  }
  if (!has_target_hand_pose_) {
    RCLCPP_WARN(this->get_logger(), "cur_hand_pose is empty");
    return;
  }

  //create states
  torch::Tensor obs = torch::tensor(
                        {
                          /* joint position */
                          cur_joint_state_.position[0],
                          cur_joint_state_.position[1],
                          cur_joint_state_.position[2],
                          cur_joint_state_.position[3],
                          cur_joint_state_.position[4],
                          cur_joint_state_.position[5],

                          /* target arm position */
                          target_hand_pose_.pose.position.x - cur_hand_pose_.pose.position.x,
                          target_hand_pose_.pose.position.y - cur_hand_pose_.pose.position.y,
                          target_hand_pose_.pose.position.z - cur_hand_pose_.pose.position.z,

                          /* pre action */
                          pre_action_.position[0],
                          pre_action_.position[1],
                          pre_action_.position[2],
                          pre_action_.position[3],
                          pre_action_.position[4],
                          pre_action_.position[5],
                        },
                        torch::TensorOptions().dtype(torch::kFloat32))
                        .unsqueeze(0);

  //influence
  torch::Tensor action = inference(obs).squeeze(0);

  // post compute
  sensor_msgs::msg::JointState target_joint;
  target_joint.header.stamp = this->get_clock()->now();
  target_joint.name = {"Revolute_1", "Revolute_2", "Revolute_3",
                       "Revolute_4", "Revolute_5", "Revolute_6"};
  target_joint.position.resize(target_joint.name.size());
  // TODO
  for (size_t i = 0; i < target_joint.name.size(); i++) {
    double raw = action[i].item<double>();
    target_joint.position[i] = 0.5 * raw;
    pre_action_.position[i] = raw;
  }

  //publish action
  target_joint_pub_->publish(target_joint);
}
}  // namespace koma

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<koma::CatchInfluence>());
  rclcpp::shutdown();

  return 0;
}