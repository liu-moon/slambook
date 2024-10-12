#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <string>

using namespace std;
using namespace Eigen;

string trajectory_file = "src/ch3/src/trajectory.txt";

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("robot_path", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 10);

        vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses = ReadTrajectory(trajectory_file);
        if (poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory file is empty or cannot be found.");
            return;
        }

        PublishPath(poses);
        PublishMarkers(poses);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;

    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> ReadTrajectory(const string &file_path) {
        vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
        ifstream fin(file_path);
        if (!fin) {
            RCLCPP_ERROR(this->get_logger(), "Cannot find trajectory file at %s", file_path.c_str());
            return poses;
        }

        while (!fin.eof()) {
            double time, tx, ty, tz, qx, qy, qz, qw;
            fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
            Twr.pretranslate(Vector3d(tx, ty, tz));
            poses.push_back(Twr);
        }
        RCLCPP_INFO(this->get_logger(), "Read total %lu pose entries", poses.size());
        return poses;
    }

    void PublishPath(const vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> &poses) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->get_clock()->now();
        path_msg.header.frame_id = "map";

        for (const auto &pose : poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->get_clock()->now();
            pose_stamped.header.frame_id = "map";

            pose_stamped.pose.position.x = pose.translation().x();
            pose_stamped.pose.position.y = pose.translation().y();
            pose_stamped.pose.position.z = pose.translation().z();

            Quaterniond q(pose.rotation());
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);
        }

        path_publisher_->publish(path_msg);
    }

    void PublishMarkers(const vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> &poses) {
        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t i = 0; i < poses.size(); i++) {
            Vector3d Ow = poses[i].translation();
            Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
            Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
            Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));

            // 创建 X 轴
            marker_array.markers.push_back(CreateAxisMarker(Ow, Xw, i * 3, {1.0, 0.0, 0.0}));

            // 创建 Y 轴
            marker_array.markers.push_back(CreateAxisMarker(Ow, Yw, i * 3 + 1, {0.0, 1.0, 0.0}));

            // 创建 Z 轴
            marker_array.markers.push_back(CreateAxisMarker(Ow, Zw, i * 3 + 2, {0.0, 0.0, 1.0}));
        }

        marker_publisher_->publish(marker_array);
    }

    visualization_msgs::msg::Marker CreateAxisMarker(const Vector3d &start, const Vector3d &end, int id, const std::array<float, 3> &color) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "axis";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;

        geometry_msgs::msg::Point p_start, p_end;
        p_start.x = start.x();
        p_start.y = start.y();
        p_start.z = start.z();
        p_end.x = end.x();
        p_end.y = end.y();
        p_end.z = end.z();

        marker.points.push_back(p_start);
        marker.points.push_back(p_end);

        marker.scale.x = 0.02;  // 箭头的杆的粗细
        marker.scale.y = 0.04;  // 箭头的头的宽度
        marker.scale.z = 0.04;  // 箭头的头的高度

        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;  // 不透明

        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);  // 永久显示

        return marker;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
