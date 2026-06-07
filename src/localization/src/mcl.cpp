#include <localization/mcl.hpp>

koma::MCL::MCL(const rclcpp::NodeOptions & options) : Node("mcl", options)
{
  // map settings
  std::filesystem::path map_path =
    std::filesystem::path(this->declare_parameter<std::string>("map_path"));

  // mcl var
  particle_num_ = this->declare_parameter<int>("particle_num", 100);
  control_loop_cycle_ = this->declare_parameter<double>("control_period_s", 0.1);
  resample_th_ = this->declare_parameter<double>("resample_th", 0.5);
  scan_step_ = this->declare_parameter<int>("scan_step", 100);
  lfm_sigma_ = this->declare_parameter<double>("lfm_sigma", 0.03);
  z_hit_ = this->declare_parameter<double>("z_hit", 1.0);
  z_max_ = this->declare_parameter<double>("z_max", 0.0);
  z_rand_ = this->declare_parameter<double>("z_rand", 1.0);

  odom_noise_1_ = this->declare_parameter<double>("odom_noise_1", 1.0);
  odom_noise_2_ = this->declare_parameter<double>("odom_noise_2", 1.0);
  odom_noise_3_ = this->declare_parameter<double>("odom_noise_3", 1.0);
  odom_noise_4_ = this->declare_parameter<double>("odom_noise_4", 1.0);

  double initial_x = this->declare_parameter<double>("initial_x", 0.25);
  double initial_y = this->declare_parameter<double>("initial_y", 0.25);
  double initial_z = this->declare_parameter<double>("initial_z", 0.3);
  double initial_theta = this->declare_parameter<double>("initial_theta", M_PI / 2);

  double initial_noise_x = this->declare_parameter<double>("initial_noise_x", 0.07);
  double initial_noise_y = this->declare_parameter<double>("initial_noise_y", 0.07);
  double initial_noise_theta = this->declare_parameter<double>("initial_noise_theta", M_PI / 180.0);

  // read field map
  read_map(map_path);

  // publisher
  pose_pub_ =
    this->create_publisher<geometry_msgs::msg::Pose>("pose", rclcpp::QoS(rclcpp::KeepLast(10)));
  trajectory_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("trajectory", rclcpp::QoS(rclcpp::KeepLast(10)));
  particle_marker_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", rclcpp::SensorDataQoS());

  // subscriber
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&koma::MCL::odom_callback, this, std::placeholders::_1));
  laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/ldlidar_node/scan", rclcpp::SensorDataQoS(),
    std::bind(&koma::MCL::laser_scan_callback, this, std::placeholders::_1));

  // tf broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  // initialize mcl
  particles_.resize(particle_num_);
  robot_pose_.position.x = initial_x;
  robot_pose_.position.y = initial_y;
  robot_pose_.position.z = initial_z;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, initial_theta);
  robot_pose_.orientation = tf2::toMsg(q);

  resetParticlesDistribution(initial_noise_x, initial_noise_y, initial_noise_theta);

  control_loop_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(control_loop_cycle_)),
    std::bind(&koma::MCL::contol_loop, this));
}

void koma::MCL::contol_loop()
{
  if (!scan_) {
    RCLCPP_WARN(this->get_logger(), "scan_ is empty");
    return;
  }

  update_particles();
  print_particles_maker_on_rviz2();
  caculate_measurement_model();
  estimate_pose();
  resample_particles();
  print_trajectory_on_rviz2();
}

void koma::MCL::update_particles()
{
  // caculate delta ( = v * dt)
  geometry_msgs::msg::Twist delta;
  delta.linear.x = cmd_vel_.linear.x * control_loop_cycle_;
  delta.linear.y = cmd_vel_.linear.y * control_loop_cycle_;
  delta.angular.z = cmd_vel_.angular.z * control_loop_cycle_;

  // L2 error
  std::double_t dd2 = delta.linear.x * delta.linear.x + delta.linear.y * delta.linear.y;
  std::double_t dy2 = delta.angular.z * delta.angular.z;

  for (size_t i = 0; i < this->particles_.size(); i++) {
    std::double_t dx =
      delta.linear.x +
      randNormal(
        std::sqrt(odom_noise_1_ * dd2 + odom_noise_2_ * dy2));  //ルート取らないといけなくない？
    std::double_t dy =
      delta.linear.y + randNormal(std::sqrt(odom_noise_1_ * dd2 + odom_noise_2_ * dy2));
    std::double_t dtheta =
      delta.angular.z + +randNormal(std::sqrt(odom_noise_3_ * dd2 + odom_noise_4_ * dy2));

    geometry_msgs::msg::Pose update_particle_pose;
    geometry_msgs::msg::Pose particle_pose = particles_[i].position;
    double theta = tf2::getYaw(particle_pose.orientation);

    // update position
    update_particle_pose.position.x =
      particle_pose.position.x + std::cos(theta) * dx - std::sin(theta) * dy;
    update_particle_pose.position.y =
      particle_pose.position.y + std::sin(theta) * dx + std::cos(theta) * dy;
    update_particle_pose.position.z = particle_pose.position.z;

    // update orientation
    tf2::Quaternion q;
    theta += dtheta;
    q.setRPY(0.0, 0.0, theta);
    update_particle_pose.orientation = tf2::toMsg(q);

    particles_[i].position = update_particle_pose;
  }
}

void koma::MCL::caculate_measurement_model()
{
  std::vector<std::vector<double>> likelihood_table;
  likelihood_table.reserve(particle_num_);

  for (std::size_t i = 0; i < particles_.size(); i++) {
    likelihood_table.push_back(std::move(caculate_likelihood_field_model(particles_[i].position)));
  }

  std::vector<double> total_log_likelihood(particle_num_, 0.0);
  for (std::size_t i = 0; i < particle_num_; i++) {
    for (std::size_t k = 0; k < likelihood_table[i].size(); k++) {
      // 1e-10などで床打ちして log(0) を防ぐ
      total_log_likelihood[i] += std::log(std::max(likelihood_table[i][k], 1e-10));
    }
  }

  double max_log_likelihood = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < particle_num_; i++) {
    if (total_log_likelihood[i] > max_log_likelihood) {
      max_log_likelihood = total_log_likelihood[i];
    }
  }

  std::vector<double> linear_weights(particle_num_, 0.0);
  double w_sum = 0.0;

  for (std::size_t i = 0; i < particle_num_; i++) {
    linear_weights[i] = std::exp(total_log_likelihood[i] - max_log_likelihood);
    w_sum += linear_weights[i];
  }

  double w_sq_sum = 0.0;
  for (std::size_t i = 0; i < particle_num_; i++) {
    double normalized_w = linear_weights[i] / w_sum;
    particles_[i].likelihood = normalized_w;
    w_sq_sum += normalized_w * normalized_w;
  }

  effective_sample_size_ = 1.0 / w_sq_sum;
}

void koma::MCL::estimate_pose()
{
  std::double_t tmpTheta = tf2::getYaw(robot_pose_.orientation);
  std::double_t x = 0.0, y = 0.0, theta = 0.0;
  for (size_t i = 0; i < particles_.size(); i++) {
    std::double_t w = particles_[i].likelihood;
    x += particles_[i].position.position.x * w;
    y += particles_[i].position.position.y * w;
    std::double_t dTheta = tmpTheta - tf2::getYaw(particles_[i].position.orientation);
    // RCLCPP_INFO(this->get_logger(), "%.4f %.4f %.4f", w, particles_[i].getX(), particles_[i].getY());
    while (dTheta < -M_PI) dTheta += 2.0 * M_PI;
    while (dTheta > M_PI) dTheta -= 2.0 * M_PI;
    theta += dTheta * w;
  }
  theta = tmpTheta - theta;

  robot_pose_.position.x = x;
  robot_pose_.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  robot_pose_.orientation = tf2::toMsg(q);

  pose_pub_->publish(robot_pose_);

  // if (!is_sim_) {
  //     geometry_msgs::msg::TransformStamped tf_msg;
  //     tf_msg.header.stamp = this->get_clock()->now();
  //     tf_msg.header.frame_id = "odom";
  //     tf_msg.child_frame_id = "base_footprint";

  //     tf_msg.transform.translation.x = x;
  //     tf_msg.transform.translation.y = y;
  //     tf_msg.transform.translation.z = 0.3;

  //     tf2::Quaternion q;
  //     q.setRPY(0.0, 0.0, theta);
  //     tf_msg.transform.rotation = tf2::toMsg(q);

  //     tf_broadcaster_->sendTransform(tf_msg);
  // }
}

void koma::MCL::resample_particles(void)
{
  double threshold = ((double)particles_.size()) * resample_th_;
  if (effective_sample_size_ > threshold) return;

  std::vector<double> wBuffer((int)particles_.size());
  wBuffer[0] = particles_[0].likelihood;
  for (size_t i = 1; i < particles_.size(); i++) {
    wBuffer[i] = particles_[i].likelihood + wBuffer[i - 1];
  }

  std::vector<PoseWithLikelihood> tmpParticles = particles_;
  double wo = 1.0 / (double)particles_.size();
  for (size_t i = 0; i < particles_.size(); i++) {
    double darts = (double)rand() / ((double)RAND_MAX + 1.0);
    for (size_t j = 0; j < particles_.size(); j++) {
      if (darts < wBuffer[j]) {
        geometry_msgs::msg::Pose tmpPose = tmpParticles[j].position;
        particles_[i].position = tmpPose;
        particles_[i].likelihood = wo;
        break;
      }
    }
  }
}

void koma::MCL::xy2uv(std::double_t x, std::double_t y, std::int32_t * u, std::int32_t * v)
{
  *u = static_cast<int32_t>(x / map_resolution_);
  *v = map_height_ - 1 - static_cast<int32_t>(y / map_resolution_);
}

void koma::MCL::lidarpose2uv(
  double range, double theta, geometry_msgs::msg::Pose pose, double * x_odom, double * y_odom,
  int * u, int * v)
{
  double x_lidar = range * cos(theta) + 0.0765;
  double y_lidar = range * sin(theta);

  double pose_theta = tf2::getYaw(pose.orientation);
  double x = x_lidar * cos(pose_theta) - y_lidar * sin(pose_theta) + pose.position.x;
  double y = x_lidar * sin(pose_theta) + y_lidar * cos(pose_theta) + pose.position.y;

  *x_odom = x;
  *y_odom = y;

  xy2uv(x, y, u, v);
}

std::vector<double> koma::MCL::caculate_likelihood_field_model(
  geometry_msgs::msg::Pose particle_pose)
{
  double var = lfm_sigma_ * lfm_sigma_;
  double normConst = 1.0 / (sqrt(2.0 * M_PI * var));
  double pMax = 1.0 / map_resolution_;  // <- mapResolution_で割る必要なくない？
  double p_rand = 1.0 / scan_->range_max * map_resolution_;
  double w = 0.0;

  std::vector<double> p_vector;

  for (std::size_t i = 0; i < scan_->ranges.size(); i += scan_step_) {
    std::double_t r = scan_->ranges[i];
    if (std::isnan(r) || r < scan_->range_min || scan_->range_max < r) {
      // p_vector.push_back(zRand_*pRand); // TODO: add pMax
      p_vector.push_back(z_rand_ * p_rand);
      continue;
    }

    std::double_t theta_lidar;
    theta_lidar =
      scan_->angle_min + (static_cast<double>(i)) * scan_->angle_increment - 3.0 * M_PI / 2.0;
    // theta_lidar = scan_->angle_min + (static_cast<double>(i)) * scan_->angle_increment;

    // } else {
    //     theta_lidar = scan.angle_min + ((std::double_t)(i))*scan.angle_increment - 3.0*M_PI/2.0;
    // }

    double x_odom, y_odom;
    int u, v;
    lidarpose2uv(r, theta_lidar, particle_pose, &x_odom, &y_odom, &u, &v);

    // TODO: isInMap
    if (0 <= u && u < map_width_ && 0 <= v && v < map_height_) {
      double d = (double)dist_field_.at<double>(v, u);
      double p_hit = normConst * exp(-(d * d) / (2.0 * var)) * map_resolution_;
      std::double_t p = z_hit_ * p_hit + z_rand_ * p_rand;

      if (p > 1.0) p = 1.0;
      p_vector.push_back(p);
    } else {
      p_vector.push_back(z_rand_ * p_rand);
    }
  }

  return p_vector;
}

void koma::MCL::odom_callback(nav_msgs::msg::Odometry::SharedPtr msgs)
{
  cmd_vel_.linear.x = msgs->twist.twist.linear.x;
  cmd_vel_.linear.y = msgs->twist.twist.linear.y;
  cmd_vel_.angular.z = msgs->twist.twist.angular.z;
}

void koma::MCL::laser_scan_callback(sensor_msgs::msg::LaserScan::SharedPtr msgs) { scan_ = msgs; }

void koma::MCL::print_particles_maker_on_rviz2()
{
  sensor_msgs::msg::PointCloud2 cloud_;
  cloud_.header.stamp = this->get_clock()->now();
  cloud_.header.frame_id = "map";
  cloud_.height = 1;
  cloud_.width = particle_num_;
  cloud_.is_dense = false;
  cloud_.is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(particle_num_);

  sensor_msgs::PointCloud2Iterator<std::float_t> iter_x(cloud_, "x");
  sensor_msgs::PointCloud2Iterator<std::float_t> iter_y(cloud_, "y");
  sensor_msgs::PointCloud2Iterator<std::float_t> iter_z(cloud_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_, "b");

  for (const PoseWithLikelihood & p : particles_) {
    *iter_x = p.position.position.x;
    *iter_y = p.position.position.y;
    *iter_z = p.position.position.z;

    *iter_r = 0;
    *iter_g = 0;
    *iter_b = int(p.likelihood * 255);

    ++iter_x, ++iter_y, ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }

  particle_marker_->publish(cloud_);
}

void koma::MCL::print_trajectory_on_rviz2()
{
  geometry_msgs::msg::PoseStamped stamped;
  stamped.header.stamp = this->now();
  stamped.header.frame_id = "map";
  stamped.pose.position.x = robot_pose_.position.x;
  stamped.pose.position.y = robot_pose_.position.y;
  stamped.pose.position.z = robot_pose_.position.z;
  stamped.pose.orientation = robot_pose_.orientation;

  trajectory_.poses.push_back(stamped);
  trajectory_.header.stamp = stamped.header.stamp;
  trajectory_.header.frame_id = stamped.header.frame_id;

  trajectory_pub_->publish(trajectory_);
}

void koma::MCL::resetParticlesDistribution(double noise_x, double noise_y, double noise_theta)
{
  std::double_t wo = 1.0 / (std::double_t)particles_.size();
  for (std::size_t i = 0; i < particles_.size(); i++) {
    // TODO: フィールドの中に入っていない場合はリサンプリングする
    std::double_t x = robot_pose_.position.x + randNormal(noise_x);
    std::double_t y = robot_pose_.position.y + randNormal(noise_y);
    particles_[i].position.position.x = x;
    particles_[i].position.position.y = y;
    particles_[i].position.position.z = robot_pose_.position.z;
    particles_[i].position.orientation = robot_pose_.orientation;
    particles_[i].likelihood = wo;
  }
}

void koma::MCL::read_map(std::filesystem::path & map_path)
{
  try {
    RCLCPP_INFO(this->get_logger(), "map path is %s", map_path.string().c_str());

    // load setting from map.yaml
    YAML::Node lconf = YAML::LoadFile((map_path / "map.yaml").string());
    map_resolution_ = lconf["resolution"].as<double>();

    cv::Mat map_img = cv::imread((map_path / "map.pgm").string(), 0);
    map_width_ = map_img.cols;
    map_height_ = map_img.rows;

    cv::Mat binary_map_img(map_height_, map_width_, CV_8UC1);
    for (int v = 0; v < map_height_; v++) {
      for (int u = 0; u < map_width_; u++) {
        uchar val = map_img.at<uchar>(v, u);

        if (val < 230) {
          binary_map_img.at<uchar>(v, u) = 0;  // 障害物
        } else {
          binary_map_img.at<uchar>(v, u) = 255;  // 自由空間
        }
      }
    }

    cv::Mat dist_field_out(map_height_, map_width_, CV_32FC1);
    cv::distanceTransform(binary_map_img, dist_field_out, cv::DIST_L2, 5);

    cv::Mat inverted_map_img;
    cv::bitwise_not(binary_map_img, inverted_map_img);
    cv::Mat dist_field_in(map_height_, map_width_, CV_32FC1);
    cv::distanceTransform(inverted_map_img, dist_field_in, cv::DIST_L2, 5);

    dist_field_ = cv::Mat(map_height_, map_width_, CV_64FC1);
    for (int v = 0; v < map_height_; v++) {
      for (int u = 0; u < map_width_; u++) {
        float d_out = dist_field_out.at<float>(v, u);
        float d_in = dist_field_in.at<float>(v, u);

        // 外側ならプラス、内側ならマイナス
        double sdf_pixel = static_cast<double>(d_out - d_in);
        dist_field_.at<double>(v, u) = sdf_pixel * map_resolution_;
      }
    }

    double min_val, max_val;
    cv::minMaxLoc(dist_field_, &min_val, &max_val);
    double max_abs_val = std::max(std::abs(min_val), std::abs(max_val)) + 1e-6;

    cv::Mat sdf_color_img(map_height_, map_width_, CV_8UC3);
    for (int v = 0; v < map_height_; v++) {
      for (int u = 0; u < map_width_; u++) {
        double d = dist_field_.at<double>(v, u);
        double ratio = std::abs(d) / max_abs_val;
        int intensity = 255 - static_cast<int>(ratio * 255.0);
        intensity = std::max(0, std::min(255, intensity));

        cv::Vec3b color;
        if (d > 1e-9) {
          color = cv::Vec3b(255, intensity, intensity);
        } else if (d < -1e-9) {
          color = cv::Vec3b(intensity, intensity, 255);
        } else {
          color = cv::Vec3b(255, 255, 255);
        }
        sdf_color_img.at<cv::Vec3b>(v, u) = color;
      }
    }
    cv::imwrite("distField_highlight.png", sdf_color_img);
    RCLCPP_INFO(this->get_logger(), "Saved colored SDF image to distField_highlight.png");

  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    throw std::runtime_error("Failed to koma::MCL::read_map");
  }
}

// void koma::MCL::read_map(std::filesystem::path & map_path)
// {
//   try {
//     RCLCPP_INFO(this->get_logger(), "map path is %s", map_path.string().c_str());

//     // load setting from map.yaml
//     YAML::Node lconf = YAML::LoadFile((map_path / "map.yaml").string());
//     map_resolution_ = lconf["resolution"].as<double>();

//     cv::Mat map_img = cv::imread((map_path / "map.pgm").string(), 0);
//     map_width_ = map_img.cols;
//     map_height_ = map_img.rows;

//     cv::Mat binary_map_img = map_img.clone();
//     for (int v = 0; v < map_height_; v++) {
//       for (int u = 0; u < map_width_; u++) {
//         uchar val = binary_map_img.at<uchar>(v, u);
//         if (val == 0) {
//           binary_map_img.at<uchar>(v, u) = 0;
//         } else {
//           binary_map_img.at<uchar>(v, u) = 1;
//         }
//       }
//     }

//     cv::Mat dist_field_f(map_height_, map_width_, CV_32FC1);
//     cv::Mat dist_field_d(map_height_, map_width_, CV_64FC1);
//     cv::distanceTransform(binary_map_img, dist_field_f, cv::DIST_L2, 5);

//     for (int v = 0; v < map_height_; v++) {
//       for (int u = 0; u < map_width_; u++) {
//         std::float_t d = dist_field_f.at<std::float_t>(v, u);
//         dist_field_d.at<std::double_t>(v, u) = (std::double_t)d * map_resolution_;
//       }
//     }

//     dist_field_ = dist_field_d.clone();

//     // save distField_
//     cv::Mat normDist;
//     cv::normalize(dist_field_d, normDist, 0.0, 255.0, cv::NORM_MINMAX);
//     cv::Mat dist8U;
//     normDist.convertTo(dist8U, CV_8U);
//     cv::Mat colorImg;
//     cv::cvtColor(dist8U, colorImg, cv::COLOR_GRAY2BGR);
//     cv::imwrite("distField_highlight.png", colorImg);

//   } catch (const YAML::Exception & e) {
//     RCLCPP_ERROR(this->get_logger(), "%s", e.what());
//     throw std::runtime_error("Failed to koma::MCL::read_map");
//   }
// }

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<koma::MCL>());
  rclcpp::shutdown();
  return 0;
}
