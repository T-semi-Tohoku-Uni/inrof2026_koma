#include <planning/path_plan.hpp>

koma::PathPlanner::PathPlanner(const rclcpp::NodeOptions & options) : Node("path_planner", options)
{
  std::filesystem::path map_path = this->declare_parameter<std::string>("map_path");

  double initial_x = this->declare_parameter<double>("initial_x", 0.25);
  double initial_y = this->declare_parameter<double>("initial_y", 0.25);
  double initial_z = this->declare_parameter<double>("initial_z", 0.3);
  double initial_theta = this->declare_parameter<double>("initial_theta", 0.0);
  robot_pose_.position.x = initial_x;
  robot_pose_.position.y = initial_y;
  robot_pose_.position.z = initial_z;
  tf2::Quaternion q;
  q.setRPY(0, 0, initial_theta);
  robot_pose_.orientation = tf2::toMsg(q);

  read_map(map_path);

  // publisher
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>(
    "path", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local());
  // subscriber
  robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "pose", rclcpp::QoS(rclcpp::KeepLast(10)),
    std::bind(&PathPlanner::robot_pose_callback, this, std::placeholders::_1));
  // service server
  goal_pose_srv_ = this->create_service<inrof2026_koma_type::srv::PoseStamped>(
    "goal_pose",
    std::bind(
      &PathPlanner::goal_pose_callback, this, std::placeholders::_1, std::placeholders::_2));
  waypoint_srv_ = this->create_service<inrof2026_koma_type::srv::PoseStamped>(
    "waypoint",
    std::bind(&PathPlanner::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void koma::PathPlanner::goal_pose_callback(
  const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Request> request,
  std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Response> response)
{
  RCLCPP_INFO(
    this->get_logger(), "Received goal pose: x=%f, y=%f", request->pose_stamped.pose.position.x,
    request->pose_stamped.pose.position.y);

  waypoint_array_.push_back(
    std::make_pair(request->pose_stamped.pose.position.x, request->pose_stamped.pose.position.y));

  std::vector<std::pair<double, double>> path;
  std::pair<double, double> start_point =
    std::make_pair(robot_pose_.position.x, robot_pose_.position.y);
  for (size_t i = 0; i < waypoint_array_.size(); i++) {
    std::vector<std::pair<double, double>> partial_pass =
      generator(start_point, waypoint_array_[i]);
    path.insert(path.end(), partial_pass.begin(), partial_pass.end());
    start_point = waypoint_array_[i];
  }

  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->get_clock()->now();
  path_msg.header.frame_id = "map";
  for (size_t i = 0; i < path.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;

    pose.pose.position.x = path[i].first;
    pose.pose.position.y = path[i].second;
    pose.pose.position.z = robot_pose_.position.z;

    tf2::Quaternion q;
    if (i + 30 > path.size()) {
      pose.pose.orientation = request->pose_stamped.pose.orientation;
    } else {
      pose.pose.orientation = robot_pose_.orientation;
    }
    path_msg.poses.push_back(pose);
  }

  pub_path_->publish(path_msg);
  response->success = true;
}

void koma::PathPlanner::xy2uv(std::double_t x, std::double_t y, std::int32_t * u, std::int32_t * v)
{
  *u = (std::int32_t)(x / map_resolution_);
  *v = map_height_ - 1 - (std::int32_t)(y / map_resolution_);
}

std::vector<std::pair<double, double>> koma::PathPlanner::generator(
  std::pair<double, double> start_point, std::pair<double, double> goal_point)
{
  double sx = start_point.first;
  double sy = start_point.second;
  double gx = goal_point.first;
  double gy = goal_point.second;

  std::priority_queue<Cell, std::vector<Cell>, std::greater<Cell>> q;
  std::vector<std::vector<double>> distances(
    this->map_height_,
    std::vector<double>(this->map_width_, std::numeric_limits<double>::infinity()));
  std::vector<std::vector<std::pair<int, int>>> previous(
    this->map_height_, std::vector<std::pair<int, int>>(this->map_width_, {-1, -1}));

  int su, sv, gu, gv;
  xy2uv(sx, sy, &su, &sv);
  xy2uv(gx, gy, &gu, &gv);

  // RCLCPP_INFO(this->get_logger(), "start %d %d", su, sv);

  distances[sv][su] = 0;
  q.push({su, sv, 0});

  const int du[4] = {-1, 1, 0, 0};
  const int dv[4] = {0, 0, -1, 1};

  while (!q.empty()) {
    Cell cur = q.top();
    q.pop();
    if (cur.u == gu && cur.v == gv) break;

    for (int dir = 0; dir < 4; dir++) {
      int nu = cur.u + du[dir];
      int nv = cur.v + dv[dir];

      if (nu >= 0 && nu < this->map_width_ && nv >= 0 && nv < this->map_height_) {
        double cost = cur.cost + dist_field_.at<double>(nv, nu);
        if (cost < distances[nv][nu]) {
          distances[nv][nu] = cost;
          previous[nv][nu] = {cur.u, cur.v};
          q.push({nu, nv, cost});
        }
      }
    }
  }

  // 経路再構築
  std::vector<std::pair<int, int>> path_g;
  for (int u = gu, v = gv; u != -1 && v != -1;) {
    path_g.push_back({u, v});
    std::tie(u, v) = previous[v][u];
  }

  std::reverse(path_g.begin(), path_g.end());

  // convert grid -> field
  std::vector<std::pair<double, double>> path_f;
  for (std::pair<int, int> & p : path_g) {
    path_f.push_back(std::make_pair(
      (p.first + 0.5) * map_resolution_,
      (static_cast<double>(map_height_ - p.second - 1) + 0.5) * map_resolution_));
  }

  return path_f;
}

void koma::PathPlanner::waypoint_callback(
  const std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Request> request,
  std::shared_ptr<inrof2026_koma_type::srv::PoseStamped::Response> response)
{
  waypoint_array_.push_back(
    std::make_pair(request->pose_stamped.pose.position.x, request->pose_stamped.pose.position.y));
  RCLCPP_INFO(
    this->get_logger(), "Received waypoint: x=%f, y=%f", request->pose_stamped.pose.position.x,
    request->pose_stamped.pose.position.y);
  response->success = true;
}

void koma::PathPlanner::robot_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  robot_pose_ = *msg;
}

void koma::PathPlanner::read_map(std::filesystem::path & map_path)
{
  try {
    RCLCPP_INFO(this->get_logger(), "map path is %s", map_path.string().c_str());

    // load setting from map.yaml
    YAML::Node lconf = YAML::LoadFile((map_path / "map.yaml").string());
    map_resolution_ = lconf["resolution"].as<double>();

    cv::Mat map_img = cv::imread((map_path / "map.pgm").string(), 0);
    map_width_ = map_img.cols;
    map_height_ = map_img.rows;

    cv::Mat binary_map_img = map_img.clone();
    for (int v = 0; v < map_height_; v++) {
      for (int u = 0; u < map_width_; u++) {
        uchar val = binary_map_img.at<uchar>(v, u);
        if (val == 0) {
          binary_map_img.at<uchar>(v, u) = 0;
        } else {
          binary_map_img.at<uchar>(v, u) = 1;
        }
      }
    }

    cv::Mat dist_field_f(map_height_, map_width_, CV_32FC1);
    cv::Mat dist_field_d(map_height_, map_width_, CV_64FC1);
    cv::distanceTransform(binary_map_img, dist_field_f, cv::DIST_L2, 5);

    for (int v = 0; v < map_height_; v++) {
      for (int u = 0; u < map_width_; u++) {
        std::float_t d = dist_field_f.at<std::float_t>(v, u);
        dist_field_d.at<std::double_t>(v, u) = (std::double_t)d * map_resolution_;
      }
    }

    double max_val;
    cv::minMaxLoc(dist_field_d, nullptr, &max_val, nullptr, nullptr);
    cv::Mat diff = dist_field_d - max_val;  // 要素ごとに引き算
    cv::Mat absDiff = cv::abs(diff);        // 要素ごとの絶対値

    dist_field_ = absDiff.clone();
  } catch (const YAML::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "%s", e.what());
    throw std::runtime_error("Failed to koma::MCL::read_map");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<koma::PathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}