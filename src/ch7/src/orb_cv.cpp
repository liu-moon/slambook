#include <rclcpp/rclcpp.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std;
using namespace cv;

class FeatureExtractionNode : public rclcpp::Node {
public:
  FeatureExtractionNode(const std::string& img1_path, const std::string& img2_path) 
  : Node("feature_extraction_node"), img1_path_(img1_path), img2_path_(img2_path) {

    if (img1_path_.empty() || img2_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "请提供img1_path和img2_path参数。");
      rclcpp::shutdown();
    }

    process_images();
  }

private:
  void process_images() {
    // 读取图像
    Mat img_1 = imread(img1_path_, IMREAD_COLOR);
    Mat img_2 = imread(img2_path_, IMREAD_COLOR);

    // 初始化
    vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

    // 第一步:检测 Oriented FAST 角点位置
    auto t1 = chrono::steady_clock::now();
    detector->detect(img_1, keypoints_1);
    detector->detect(img_2, keypoints_2);

    // 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, keypoints_1, descriptors_1);
    descriptor->compute(img_2, keypoints_2, descriptors_2);
    auto t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    RCLCPP_INFO(this->get_logger(), "ORB特征提取耗时 = %f 秒", time_used.count());

    Mat outimg1;
    drawKeypoints(img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT); // Scalar::all(-1)（关键点颜色:表示自动选择颜色） DrawMatchesFlags::DEFAULT（绘制标志:默认圆形）
    imshow("ORB features", outimg1);

    // 第三步:匹配 BRIEF 描述子，使用 Hamming 距离
    vector<DMatch> matches;
    t1 = chrono::steady_clock::now();
    matcher->match(descriptors_1, descriptors_2, matches);
    t2 = chrono::steady_clock::now();
    time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    RCLCPP_INFO(this->get_logger(), "ORB匹配耗时 = %f 秒", time_used.count());

    // 第四步:筛选匹配点对
    auto min_max = minmax_element(matches.begin(), matches.end(),
                                  [](const DMatch &m1, const DMatch &m2) { return m1.distance < m2.distance; });
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;

    RCLCPP_INFO(this->get_logger(), "最大距离: %f", max_dist);
    RCLCPP_INFO(this->get_logger(), "最小距离: %f", min_dist);

    vector<DMatch> good_matches;
    for (int i = 0; i < descriptors_1.rows; i++) {
      if (matches[i].distance <= max(2 * min_dist, 30.0)) {
        good_matches.push_back(matches[i]);
      }
    }

    // 第五步:绘制匹配结果
    Mat img_match, img_goodmatch;
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, matches, img_match);
    drawMatches(img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch);
    imshow("all matches", img_match);
    imshow("good matches", img_goodmatch);
    waitKey(0);
  }

  string img1_path_;
  string img2_path_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  if (argc < 3)
  {
    std::cerr << "Usage: ros2 run <package_name> <executable> <img1_path> <img2_path>" << std::endl;
    return 1;
  }

  auto node = std::make_shared<FeatureExtractionNode>(std::string(argv[1]),std::string(argv[2]));
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
