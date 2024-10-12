#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <string>

using namespace std;
using namespace Eigen;

// 轨迹文件路径
string trajectory_file = "src/ch3/src/trajectory.txt";

class TrajectoryPublisher : public rclcpp::Node {
public:
    TrajectoryPublisher() : Node("trajectory_publisher") {
        // 创建发布器，发布轨迹信息
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("robot_path", 10);

        // 读取轨迹文件
        vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses = ReadTrajectory(trajectory_file);
        if (poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory file is empty or cannot be found.");
            return;
        }

        // 发布轨迹
        PublishPath(poses);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;

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
        path_msg.header.frame_id = "map";  // 假设我们在“map”坐标系中

        for (const auto &pose : poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->get_clock()->now();
            pose_stamped.header.frame_id = "map";

            // 设置位置
            pose_stamped.pose.position.x = pose.translation().x();
            pose_stamped.pose.position.y = pose.translation().y();
            pose_stamped.pose.position.z = pose.translation().z();

            // 设置姿态
            Quaterniond q(pose.rotation());
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);
        }

        // 发布路径
        path_publisher_->publish(path_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}
