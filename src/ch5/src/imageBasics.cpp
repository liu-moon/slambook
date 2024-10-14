#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class ImagePublisherNode : public rclcpp::Node {
public:
  ImagePublisherNode() : Node("image_publisher_node") {
    // 创建一个发布器，发布sensor_msgs/Image类型的消息
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 10);
    // 启动定时器，每500ms发布一次图像
    timer_ = this->create_wall_timer(500ms, std::bind(&ImagePublisherNode::timerCallback, this));
  }

  void loadAndProcessImage(const std::string& file_path) {
    // 读取图像
    image_ = cv::imread(file_path);
    if (image_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "文件 %s 不存在或无法读取。", file_path.c_str());
      return;
    }

    RCLCPP_INFO(this->get_logger(), "图像宽为 %d, 高为 %d, 通道数为 %d", image_.cols, image_.rows, image_.channels());

    if (image_.type() != CV_8UC1 && image_.type() != CV_8UC3) {
      RCLCPP_ERROR(this->get_logger(), "请输入一张彩色图或灰度图。");
      return;
    }

    // 遍历图像并计时
    auto t1 = std::chrono::steady_clock::now();
    for (int y = 0; y < image_.rows; y++) {
      unsigned char* row_ptr = image_.ptr<unsigned char>(y);
      for (int x = 0; x < image_.cols; x++) {
        unsigned char* data_ptr = &row_ptr[x * image_.channels()];
        for (int c = 0; c != image_.channels(); c++) {
          unsigned char data = data_ptr[c];
        }
      }
    }
    auto t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    RCLCPP_INFO(this->get_logger(), "遍历图像用时：%.2f 秒", time_used.count());

    // 关于 cv::Mat 的拷贝
  // 直接赋值并不会拷贝数据
  cv::Mat image_another = image_;
  // 修改 image_another 会导致 image 发生变化
  image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 将左上角100*100的块置零
  cv::imshow("image", image_);
  cv::waitKey(0);

  // 使用clone函数来拷贝数据
  cv::Mat image_clone = image_.clone();
  image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
  cv::imshow("image", image_);
  cv::imshow("image_clone", image_clone);
  cv::waitKey(0);

  // 对于图像还有很多基本的操作,如剪切,旋转,缩放等,限于篇幅就不一一介绍了,请参看OpenCV官方文档查询每个函数的调用方法.
  cv::destroyAllWindows();
  }

private:
  void timerCallback() {
    if (!image_.empty()) {
      // 将cv::Mat转换为ROS的图像消息
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
      image_publisher_->publish(*msg);
      RCLCPP_INFO(this->get_logger(), "发布图像消息。");
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::Mat image_;
};

int main(int argc, char** argv) {
  // 初始化ROS 2
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: ros2 run <package_name> <executable> <image_file_path>" << std::endl;
    return 1;
  }

  auto node = std::make_shared<ImagePublisherNode>();
  node->loadAndProcessImage(argv[1]);

  // 开始处理ROS 2的事件循环
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
