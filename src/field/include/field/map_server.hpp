#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace koma
{
class FieldMeshMarkerPublisher : public rclcpp::Node
{
public:
  FieldMeshMarkerPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  std::string mesh_resource_;
  std::string frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_publisher_;

  void publish_map();
};
}  // namespace koma