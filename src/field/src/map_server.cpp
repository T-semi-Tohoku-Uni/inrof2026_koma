#include <field/map_server.hpp>

koma::FieldMeshMarkerPublisher::FieldMeshMarkerPublisher(const rclcpp::NodeOptions & options): Node("map_server", options) {
    mesh_resource_ = this->declare_parameter<std::string>("mesh_resource", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

    if (mesh_resource_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "mesh_resource parameter is required");
        throw std::runtime_error("mesh_resource parameter is required");
    }

    map_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "field_marker",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()
    );

    timer_ = this->create_wall_timer(
        10s,
        std::bind(&koma::FieldMeshMarkerPublisher::publish_map, this)
    );
}

void koma::FieldMeshMarkerPublisher::publish_map() {
    visualization_msgs::msg::Marker marker;

    marker.header.stamp = this->now();
    marker.header.frame_id = frame_id_;

    marker.ns = "field";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.mesh_resource = mesh_resource_;

    marker.mesh_use_embedded_materials = true;

    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
        
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.8f;
    marker.color.g = 0.8f;
    marker.color.b = 0.8f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration(0, 0);

    map_publisher_->publish(marker);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<koma::FieldMeshMarkerPublisher>());
  rclcpp::shutdown();
  return 0;
}