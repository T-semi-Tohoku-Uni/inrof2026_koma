#include <komarm/hand_pose.hpp>

koma::HandPose::HandPose(const rclcpp::NodeOptions & options) : Node("hand_pose")
{
  dist_x = std::uniform_real_distribution<double>(0.30, 0.30);
  dist_y = std::uniform_real_distribution<double>(0.0, 0.0);
  dist_z = std::uniform_real_distribution<double>(0.10, 0.10);

  hand_pose_client_ = this->create_client<inrof2026_koma_type::srv::PoseStamped>("hand_pose");
  while (!this->hand_pose_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "srvHandPose not available");
  }
  hand_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::Marker>("hand_pose_marker", 10);

  hand_pose_timer_ =
    this->create_wall_timer(10s, std::bind(&koma::HandPose::handPoseCallback, this));
}

void koma::HandPose::handPoseCallback()
{
  while (!this->hand_pose_client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) break;
    RCLCPP_WARN(this->get_logger(), "srvHandPose not available");
  }

  inrof2026_koma_type::srv::PoseStamped_Request::SharedPtr request =
    std::make_shared<inrof2026_koma_type::srv::PoseStamped::Request>();
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = "base_link";
  pose_msg.header.stamp = this->get_clock()->now();

  pose_msg.pose.position.x = dist_x(gen_);
  pose_msg.pose.position.y = dist_y(gen_);
  pose_msg.pose.position.z = dist_z(gen_);

  pose_msg.pose.orientation.x = 0.0;
  pose_msg.pose.orientation.y = 0.0;
  pose_msg.pose.orientation.z = 0.0;
  pose_msg.pose.orientation.w = 1.0;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "base_link";
  marker.header.stamp = this->get_clock()->now();

  marker.ns = "hand_pose";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = pose_msg.pose.position.x;
  marker.pose.position.y = pose_msg.pose.position.y;
  marker.pose.position.z = pose_msg.pose.position.z;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = rclcpp::Duration::from_seconds(0.0);

  hand_pose_marker_pub_->publish(marker);
  request->pose_stamped = pose_msg;
  hand_pose_client_->async_send_request(
    request, [this](rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::SharedFuture future) {
      const auto response = future.get();
    });
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<koma::HandPose> node = std::make_shared<koma::HandPose>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}