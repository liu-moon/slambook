#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <fstream>
#include <vector>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

using namespace std;
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;

class PointCloudPublisher : public rclcpp::Node {
public:
    PointCloudPublisher()
        : Node("pointcloud_publisher") {
        // 创建发布器，发布到 "colored_pointcloud" 话题
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("colored_pointcloud", 10);

        compute_point_cloud();
    }

private:
    void compute_point_cloud() {
        
        vector<cv::Mat> colorImgs, depthImgs;
        TrajectoryType poses;

        // 读取位姿文件
        ifstream fin("src/ch5/src/pose.txt");
        if (!fin) {
            RCLCPP_ERROR(this->get_logger(), "请在有pose.txt的目录下运行此程序");
            return;
        }

        // 读取图像和位姿
        for (int i = 0; i < 5; i++) {
            boost::format fmt("src/ch5/src/%s/%d.%s"); 
            colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
            depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));

            if (colorImgs.back().empty() || depthImgs.back().empty()) {
                RCLCPP_ERROR(this->get_logger(), "无法加载图像 %d", i + 1);
                return;
            }

            double data[7] = {0};
            for (auto &d : data)
                fin >> d;
            Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                              Eigen::Vector3d(data[0], data[1], data[2]));
            poses.push_back(pose);
        }

        // 相机内参
        double cx = 325.5;
        double cy = 253.5;
        double fx = 518.0;
        double fy = 519.0;
        double depthScale = 1000.0;
        vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
        pointcloud.reserve(1000000);

        // 处理每张图像
        for (int i = 0; i < 5; i++) {
            RCLCPP_INFO(this->get_logger(), "转换图像中: %d", i + 1);
            cv::Mat color = colorImgs[i];
            cv::Mat depth = depthImgs[i];
            Sophus::SE3d T = poses[i];

            for (int v = 0; v < color.rows; v++) {
                for (int u = 0; u < color.cols; u++) {
                    unsigned int d = depth.ptr<unsigned short>(v)[u]; // 深度值
                    if (d == 0) continue; // 跳过无效的深度值

                    Eigen::Vector3d point;
                    point[2] = double(d) / depthScale;
                    point[0] = (u - cx) * point[2] / fx;
                    point[1] = (v - cy) * point[2] / fy;
                    Eigen::Vector3d pointWorld = T * point;

                    Vector6d p;
                    p.head<3>() = pointWorld;
                    p[5] = color.data[v * color.step + u * color.channels()];   // blue
                    p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
                    p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
                    pointcloud.push_back(p);
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "点云共有 %ld 个点.", pointcloud.size());
        publish_pointcloud(pointcloud);
    }

    void publish_pointcloud(vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud)
    {
        
        auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();

        // 设置点云的帧 ID 和时间戳
        pointcloud_msg->header.frame_id = "map";
        pointcloud_msg->header.stamp = this->now();
        pointcloud_msg->height = 1;  // 单行点云
        pointcloud_msg->width = pointcloud.size();

        // 设置字段 (x, y, z, rgb)
        sensor_msgs::PointCloud2Modifier modifier(*pointcloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(pointcloud_msg->width * pointcloud_msg->height);

        // 使用迭代器设置点云数据
        sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*pointcloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*pointcloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*pointcloud_msg, "b");

        for (size_t i = 0; i < pointcloud_msg->width; i++)
        {
            *iter_x = pointcloud[i](0);
            *iter_y = pointcloud[i](1);
            *iter_z = pointcloud[i](2);
            *iter_r = pointcloud[i](3);
            *iter_g = pointcloud[i](4);
            *iter_b = pointcloud[i](5);

            ++iter_x;
            ++iter_y;
            ++iter_z;
            ++iter_r;
            ++iter_g;
            ++iter_b;
        }

        // 发布点云并输出日志信息
        publisher_->publish(*pointcloud_msg);
        RCLCPP_INFO(this->get_logger(), "Published colored pointcloud with %d points.", pointcloud_msg->width);
        
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPublisher>();
    rclcpp::spin(node);  // 使用spin处理节点事件，保持程序运行直到用户关闭
    rclcpp::shutdown();  // 程序终止后关闭节点
    return 0;
}
