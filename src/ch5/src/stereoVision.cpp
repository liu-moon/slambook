#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <unistd.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using namespace std;
using namespace Eigen;

// 文件路径
string left_file = "src/ch5/src/left.png";
string right_file = "src/ch5/src/right.png";

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("MyNode")
  {
    // 初始化点云发布器
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    // 内参
    double fx = 718.856, fy = 718.856, cx = 607.1928, cy = 185.2157;
    // 基线
    double b = 0.573;

    // 读取图像
    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        0, 96, 9, 8 * 9 * 9, 32 * 9 * 9, 1, 63, 10, 100, 32);    // 神奇的参数
    cv::Mat disparity_sgbm, disparity;
    sgbm->compute(left, right, disparity_sgbm);
    disparity_sgbm.convertTo(disparity, CV_32F, 1.0 / 16.0f);

    // 生成点云
    vector<Vector4d, Eigen::aligned_allocator<Vector4d>> pointcloud;

    // 如果你的机器慢，请把后面的v++和u++改成v+=2, u+=2
    for (int v = 0; v < left.rows; v++)
        for (int u = 0; u < left.cols; u++) {
            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0) continue;

            Vector4d point(0, 0, 0, left.at<uchar>(v, u) / 255.0); // 前三维为xyz,第四维为颜色

            // 根据双目模型计算 point 的位置
            double x = (u - cx) / fx;
            double y = (v - cy) / fy;
            double depth = fx * b / (disparity.at<float>(v, u));
            point[0] = x * depth;
            point[1] = y * depth;
            point[2] = depth;

            pointcloud.push_back(point);
        }

    // todo: 窗口显示bug
    // cv::imshow("disparity", disparity / 96.0);
    // cv::waitKey(0);
    // 画出点云
    // 生成点云并发布到 ROS 2
    publishPointCloud(disparity, left, fx, fy, cx, cy, b);
  }

  void publishPointCloud(const cv::Mat &disparity, const cv::Mat &left_image, double fx, double fy, double cx, double cy, double b)
    {
        // 创建 PointCloud2 消息
        auto pointcloud_msg = sensor_msgs::msg::PointCloud2();
        pointcloud_msg.header.frame_id = "map";
        pointcloud_msg.header.stamp = this->now();
        pointcloud_msg.height = 1; // 点云的高度
        pointcloud_msg.width = left_image.rows * left_image.cols; // 点云的宽度
        pointcloud_msg.is_dense = false;
        pointcloud_msg.is_bigendian = false;

        // 设置字段
        sensor_msgs::PointCloud2Modifier modifier(pointcloud_msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

        sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pointcloud_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pointcloud_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pointcloud_msg, "b");

        int valid_points = 0;

        // 遍历每个像素并计算 3D 点云
        for (int v = 0; v < disparity.rows; v++)
        {
            for (int u = 0; u < disparity.cols; u++)
            {
                float disp = disparity.at<float>(v, u);
                if (disp <= 0.0 || disp >= 96.0)
                {
                    // 如果视差无效，则跳过
                    continue;
                }

                double x = (u - cx) / fx;
                double y = (v - cy) / fy;
                double depth = fx * b / disp;

                // 填充点云数据
                *iter_x = static_cast<float>(x * depth);
                *iter_y = static_cast<float>(y * depth);
                *iter_z = static_cast<float>(depth);

                // 获取灰度值并转换为RGB
                uint8_t intensity = left_image.at<uchar>(v, u);
                *iter_r = intensity;
                *iter_g = intensity;
                *iter_b = intensity;

                ++iter_x; ++iter_y; ++iter_z;
                ++iter_r; ++iter_g; ++iter_b;
                
                valid_points++;  // 计数有效点数
            }
        }

        RCLCPP_INFO(this->get_logger(), "Point cloud generated with %d valid points. Publishing to /pointcloud...", valid_points);

        // 发布点云消息
        pointcloud_publisher_->publish(pointcloud_msg);

        RCLCPP_INFO(this->get_logger(), "Point cloud published successfully.");
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}