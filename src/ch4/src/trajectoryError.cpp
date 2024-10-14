#include <rclcpp/rclcpp.hpp>
#include <sophus/se3.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <vector>
#include <string>

using namespace std;
using namespace Eigen;

string groundtruth_file = "src/ch4/src/estimated.txt";
string estimated_file = "src/ch4/src/groundtruth.txt";


class SophusExampleNode : public rclcpp::Node {
public:
    SophusExampleNode() : Node("sophus_example_node") {

        // 创建发布器，分别发布groundtruth和estimated轨迹信息
        groundtruth_publisher_ = this->create_publisher<nav_msgs::msg::Path>("groundtruth_path", 10);
        estimated_publisher_ = this->create_publisher<nav_msgs::msg::Path>("estimated_path", 10);

        cout << "*******************************" << endl;
        // 读取groundtruth轨迹文件
        vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> groundtruth = ReadTrajectory(groundtruth_file);
        if (groundtruth.empty()) {
            RCLCPP_ERROR(this->get_logger(), "groundtruth_file file is empty or cannot be found.");
            return;
        }

        // 读取estimated轨迹文件
        vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> estimated = ReadTrajectory(estimated_file);
        if (estimated.empty()) {
            RCLCPP_ERROR(this->get_logger(), "estimated_file file is empty or cannot be found.");
            return;
        }

        // compute rmse
        rmse = 0;
        for (size_t i = 0; i < estimated.size(); i++) {
            Sophus::SE3d p1 = estimated[i], p2 = groundtruth[i];
            double error = (p2.inverse() * p1).log().norm();
            rmse += error * error;
        }
        rmse = rmse / double(estimated.size());
        rmse = sqrt(rmse);
        cout << "RMSE = " << rmse << endl;

        // 分别发布groundtruth和estimated轨迹
        PublishPath(groundtruth, groundtruth_publisher_);
        PublishPath(estimated, estimated_publisher_);
    }
    double rmse;
private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr groundtruth_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr estimated_publisher_;

    vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> ReadTrajectory(const string &file_path) {
        vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> poses;
        ifstream fin(file_path);
        if (!fin) {
            RCLCPP_ERROR(this->get_logger(), "Cannot find trajectory file at %s", file_path.c_str());
            return poses;
        }

        while (!fin.eof()) {
            double time, tx, ty, tz, qx, qy, qz, qw;
            fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Sophus::SE3d Twr(Quaterniond(qw, qx, qy, qz),Vector3d(tx,ty,tz));
            poses.push_back(Twr);
        }
        RCLCPP_INFO(this->get_logger(), "Read total %lu pose entries", poses.size());
        return poses;
    }

    void PublishPath(const vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> &poses, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher) {
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
            Quaterniond q(pose.so3().unit_quaternion());
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose_stamped);
        }

        // 发布路径
        publisher->publish(path_msg);
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SophusExampleNode>());
    rclcpp::shutdown();
    return 0;
}
