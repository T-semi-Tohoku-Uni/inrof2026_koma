#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

using namespace std::chrono_literals;

namespace koma
{
class PoseWithLikelihood
{
public:
  geometry_msgs::msg::Pose position;
  double likelihood;
};

class MCL : public rclcpp::Node
{
public:
  MCL(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // field var (TODO: merge in field class)
  int32_t map_width_, map_height_;
  double map_resolution_;
  cv::Mat dist_field_;

  // mcl constant
  int particle_num_;
  int scan_step_;

  double effective_sample_size_;
  double resample_th_;
  double z_hit_, z_short_, z_max_, z_rand_;
  double lfm_sigma_;
  double odom_noise_1_, odom_noise_2_, odom_noise_3_, odom_noise_4_;

  // mcl var
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  std::vector<PoseWithLikelihood> particles_;
  geometry_msgs::msg::Twist cmd_vel_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  double control_loop_cycle_;

  // estimated robot position
  geometry_msgs::msg::Pose robot_pose_;

  // save trajectory and publish to rviz
  nav_msgs::msg::Path trajectory_;

  // only use real world robot
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // publisher
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particle_marker_;

  // subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

  // function
  void contol_loop();
  void read_map(std::filesystem::path & map_path);
  void resetParticlesDistribution(double noise_x, double noise_y, double noise_theta);
  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msgs);
  void laser_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msgs);
  void update_particles();
  void caculate_measurement_model();
  void estimate_pose();
  void resample_particles();
  std::vector<double> caculate_likelihood_field_model(geometry_msgs::msg::Pose particle_pose);
  void lidarpose2uv(
    double range, double theta, geometry_msgs::msg::Pose pose, double * x_odom, double * y_odom,
    int * u, int * v);
  void xy2uv(std::double_t x, std::double_t y, int32_t * u, int32_t * v);
  double randNormal(double n)
  {
    return (n * sqrt(-2.0 * log((double)rand() / RAND_MAX)) * cos(2.0 * M_PI * rand() / RAND_MAX));
  }
  void print_particles_maker_on_rviz2();
  void print_trajectory_on_rviz2();
};
}  // namespace koma