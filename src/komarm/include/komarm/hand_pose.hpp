#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <inrof2026_koma_type/srv/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <random>

using namespace std::chrono_literals;

namespace koma
{
class HandPose : public rclcpp::Node
{
public:
  HandPose(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::uniform_real_distribution<double> dist_x;
  std::uniform_real_distribution<double> dist_y;
  std::uniform_real_distribution<double> dist_z;
  std::random_device rd_;
  std::mt19937 gen_{rd_()};

  rclcpp::TimerBase::SharedPtr hand_pose_timer_;
  rclcpp::Client<inrof2026_koma_type::srv::PoseStamped>::SharedPtr hand_pose_client_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr hand_pose_marker_pub_;
  void handPoseCallback();
};
}  // namespace koma