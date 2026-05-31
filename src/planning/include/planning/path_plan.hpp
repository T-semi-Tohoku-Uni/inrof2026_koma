#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <inrof2026_koma_type/srv/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace koma
{
struct Cell
{
  int u, v;
  double cost;

  bool operator>(const Cell & other) const { return cost > other.cost; }
};

class PathPlanner : public rclcpp::Node
{
public:
  PathPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // map settings
  int map_width_, map_height_;
  double map_resolution_;
  cv::Mat dist_field_;

  // robot pose
  geometry_msgs::msg::Pose robot_pose_;

  // waypoints
  // TODO: change to vector of Pose
  std::vector<std::pair<double, double>> waypoint_array_;

  // subscriber
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr robot_pose_sub_;
  void robot_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

  // publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

  // service server
  rclcpp::Service<inrof2026_koma_type::srv::PoseStamped>::SharedPtr goal_pose_srv_;
  rclcpp::Service<inrof2026_koma_type::srv::PoseStamped>::SharedPtr waypoint_srv_;
  void goal_pose_callback(
    const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Request> request,
    std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Response> response);
  void waypoint_callback(
    const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Request> request,
    std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Response> response);

  void read_map(std::filesystem::path & map_path);
  std::vector<std::pair<double, double>> generator(
    std::pair<double, double> start_point, std::pair<double, double> goal_point);
  void xy2uv(std::double_t x, std::double_t y, std::int32_t * u, std::int32_t * v);
  // std::vector<std::pair<double, double>> spline_smooth_eigen(const std::vector<std::pair<double, double>> &original_path);
};
}  // namespace koma