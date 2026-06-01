#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include <nanoflann.hpp>
#include <random>
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <optional>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <inrof2026_koma_type/srv/ball_position.hpp>
#include <filesystem>

namespace koma {
    enum ClusterID {
        UNVISITED=-1,
        NOISE=-2,
    };

    class UnitVector {
        public:
            UnitVector(double x, double y, double eps);
            double getX() const;
            double getY() const;
        private:
            double x_, y_;
    };

    class Point {
        public:
            Point(float x, float y, int pointID, int clusterID=koma::ClusterID::UNVISITED);
            float getX() const;
            float getY() const;
            int getID() const;
            int getPointID() const;
            void setID(int id);
        private:
            float x_, y_;
            int clusterID_;
            int pointID_;
    };

    class PointCloud {
        public:
            std::vector<Point> points;
            inline size_t kdtree_get_point_count() const;
            inline double kdtree_get_pt(const size_t idx, const size_t dim) const;

            template <class BBOX>
            bool kdtree_get_bbox(BBOX&) const;
    };

    class Circle {
        public:
            Circle();
            Circle(std::vector<koma::Point> &cluster);
            void markClosest();
            double getX();
            double getY();
            double getR();
            bool isClosest();
        private:
            double is_closest_;
            double x_, y_, r_;
    };

    class Field {
        public:
            Field();
            Field(std::string map_dir);
            bool isBallOnField(koma::Circle &c);
            void xy2uv(std::double_t x, std::double_t y, std::int32_t *u, std::int32_t *v);
            double mapResolution_;
            int mapWidth_, mapHeight_;
            std::vector<double> mapOrigin_;
            cv::Mat mapImg_;
    };

    using KdTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, koma::PointCloud>,
        koma::PointCloud,
        2>;

    class BallDetect: public rclcpp::Node {
        public:
            BallDetect(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
            std::optional<geometry_msgs::msg::Pose> detect();

        private:
            // callback
            void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
            void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);

            // DBSCAN algorithm
            std::unordered_map<int, std::vector<koma::Point>> dbscan(
                std::vector<koma::Point> &points, 
                koma::KdTree &tree
            );
            void expandCluster(
                koma::Point &p, 
                std::vector<koma::Point> &points,
                std::vector<size_t> &neighbors, 
                koma::KdTree& tree,
                const int cluster_id
            );
            std::vector<size_t> regionQuery(
                koma::Point &p, 
                koma::KdTree& tree
            );

            // delete wall
            double median(std::vector<double>& v);
            std::vector<int> deleteWall(
                std::unordered_map<int, std::vector<koma::Point>>& clusters
            );
            // <cluster_id, cluster_points>
            std::vector<std::pair<int, std::vector<koma::Point>>> collectBallPoints(
                const std::unordered_map<int, std::vector<koma::Point>>& clusters,
                const std::vector<int>& ball_cluster_ids
            );

            // search
            std::optional<geometry_msgs::msg::Pose> findClosestBall(
                std::vector<koma::Circle> &ball
            );

            // convert LaserScan to Point
            koma::PointCloud scan2Point(const sensor_msgs::msg::LaserScan scan);

            sensor_msgs::msg::PointCloud2 point2PointCloud2(
                const std::vector<std::pair<int, std::vector<koma::Point>>> &points
            );

            bool isBallOnField(koma::Field &f, koma::Circle &c);

            // service server callback
            void ballPoseCallback(
                const std::shared_ptr<inrof2026_koma_type::srv::BallPosition::Request> request,
                const std::shared_ptr<inrof2026_koma_type::srv::BallPosition::Response> response
            );

            // Lidar
            sensor_msgs::msg::LaserScan::SharedPtr scan_;
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLider_;

            // pose
            std::unique_ptr<geometry_msgs::msg::Pose> pose_;

            // DBSCAN parameter
            double EPS_;
            int MIN_PTS_;
            
            // delete wall parameter
            double DIAGONAL_THTRSHOLD_;
            double DIFF_THTRSHOLD_;
            double WALL_THTRSHOLD_;
            double LIDAR_THTRSHOLD_;
            double RADIUS_THTRSHOLD_;

            // ball 
            std::vector<koma::Circle> ball_;
            sensor_msgs::msg::PointCloud2 circle2PointCloud2(std::vector<koma::Circle> ball_position);

            // env
            bool is_sim_;

            // tf
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;
            rclcpp::TimerBase::SharedPtr timer_;

            // lidar frame
            std::string frame_id_;

            // field
            koma::Field field_;

            // publisher
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubClusters_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubBallShape_;

            // subscriber
            rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subPose_;

            // detact action server
            rclcpp::Service<inrof2026_koma_type::srv::BallPosition>::SharedPtr srv_ball_pose_;
    };
}
