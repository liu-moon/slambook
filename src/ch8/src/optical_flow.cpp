#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

/// Optical flow tracker and interface
class OpticalFlowTracker
{
public:
    OpticalFlowTracker(
        const Mat &img1_,
        const Mat &img2_,
        const vector<KeyPoint> &kp1_,
        vector<KeyPoint> &kp2_,
        vector<bool> &success_,
        bool inverse_ = true, bool has_initial_ = false) : img1(img1_), img2(img2_), kp1(kp1_), kp2(kp2_), success(success_), inverse(inverse_),
                                                           has_initial(has_initial_) {}

    void calculateOpticalFlow(const Range &range);

private:
    const Mat &img1;
    const Mat &img2;
    const vector<KeyPoint> &kp1;
    vector<KeyPoint> &kp2;
    vector<bool> &success;
    bool inverse = true;
    bool has_initial = false;
};

/**
 * single level optical flow
 * @param [in] img1 the first image
 * @param [in] img2 the second image
 * @param [in] kp1 keypoints in img1
 * @param [in|out] kp2 keypoints in img2, if empty, use initial guess in kp1
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse use inverse formulation?
 */
void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false,
    bool has_initial_guess = false);

/**
 * multi level optical flow, scale of pyramid is set to 2 by default
 * the image pyramid will be create inside the function
 * @param [in] img1 the first pyramid
 * @param [in] img2 the second pyramid
 * @param [in] kp1 keypoints in img1
 * @param [out] kp2 keypoints in img2
 * @param [out] success true if a keypoint is tracked successfully
 * @param [in] inverse set true to enable inverse formulation
 */
void OpticalFlowMultiLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse = false);

/**
 * get a gray scale value from reference image (bi-linear interpolated)
 * @param img
 * @param x
 * @param y
 * @return the interpolated value of this pixel
 */

inline float GetPixelValue(const cv::Mat &img, float x, float y)
{
    // boundary check
    if (x < 0)
        x = 0;
    if (y < 0)
        y = 0;
    if (x >= img.cols - 1)
        x = img.cols - 2;
    if (y >= img.rows - 1)
        y = img.rows - 2;

    float xx = x - floor(x);
    float yy = y - floor(y);
    int x_a1 = std::min(img.cols - 1, int(x) + 1);
    int y_a1 = std::min(img.rows - 1, int(y) + 1);

    return (1 - xx) * (1 - yy) * img.at<uchar>(y, x) + xx * (1 - yy) * img.at<uchar>(y, x_a1) + (1 - xx) * yy * img.at<uchar>(y_a1, x) + xx * yy * img.at<uchar>(y_a1, x_a1);
}

class optical_flow : public rclcpp::Node
{
public:
    optical_flow(const std::string &img1_path, const std::string &img2_path)
        : Node("optical_flow_node"), img1_path_(img1_path), img2_path_(img2_path)
    {
        // images, note they are CV_8UC1, not CV_8UC3
        Mat img1 = imread(img1_path, 0);
        Mat img2 = imread(img2_path, 0);

        // key points, using GFTT here.
        vector<KeyPoint> kp1;
        // Good Features to Track 检测器
        // 500 最大特征点数量
        // 0.01 质量等级参数
        // 20 最小距离。该参数表示特征点之间的最小间距，防止特征点过于密集。
        Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints

        detector->detect(img1, kp1);

        // now lets track these key points in the second image
        // first use single level LK in the validation picture
        vector<KeyPoint> kp2_single; // 在图像 img2 中追踪到的特征点的向量
        vector<bool> success_single; // 记录每个关键点的追踪状态
        OpticalFlowSingleLevel(img1, img2, kp1, kp2_single, success_single); // 通过光流计算将 img1 中的特征点 kp1 追踪到 img2 中的位置，并将这些位置存储在 kp2_single 中

        // then test multi-level LK
        vector<KeyPoint> kp2_multi;
        vector<bool> success_multi;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        OpticalFlowMultiLevel(img1, img2, kp1, kp2_multi, success_multi, true); // 多层光流
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "optical flow by gauss-newton: " << time_used.count() << endl;

        // use opencv's flow for validation
        vector<Point2f> pt1, pt2;
        for (auto &kp : kp1)
            pt1.push_back(kp.pt);
        vector<uchar> status;
        vector<float> error;
        t1 = chrono::steady_clock::now();
        cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
        t2 = chrono::steady_clock::now();
        time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "optical flow by opencv: " << time_used.count() << endl;

        // plot the differences of those functions
        Mat img2_single;
        cv::cvtColor(img2, img2_single, COLOR_GRAY2RGB);
        for (int i = 0; i < kp2_single.size(); i++)
        {
            if (success_single[i])
            {
                cv::circle(img2_single, kp2_single[i].pt, 2, cv::Scalar(0, 250, 0), 2);
                cv::line(img2_single, kp1[i].pt, kp2_single[i].pt, cv::Scalar(0, 250, 0));
            }
        }

        Mat img2_multi;
        cv::cvtColor(img2, img2_multi, COLOR_GRAY2RGB);
        for (int i = 0; i < kp2_multi.size(); i++)
        {
            if (success_multi[i])
            {
                cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
                cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
            }
        }

        Mat img2_CV;
        cv::cvtColor(img2, img2_CV, COLOR_GRAY2RGB);
        for (int i = 0; i < pt2.size(); i++)
        {
            if (status[i])
            {
                cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
                cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
            }
        }

        cv::imshow("tracked single level", img2_single);
        cv::imshow("tracked multi level", img2_multi);
        cv::imshow("tracked by opencv", img2_CV);
        cv::waitKey(0);
    }

private:
    string img1_path_;
    string img2_path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3)
    {
        std::cerr << "Usage: ros2 run <package_name> <executable> <img1_path> <img2_path>" << std::endl;
        return 1;
    }

    auto node = std::make_shared<optical_flow>(std::string(argv[1]), std::string(argv[2]));

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

void OpticalFlowSingleLevel(
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse, bool has_initial)
{
    kp2.resize(kp1.size());
    success.resize(kp1.size());
    OpticalFlowTracker tracker(img1, img2, kp1, kp2, success, inverse, has_initial);
    // 并行计算光流 parallel_for_ 会将每个分段的特征点范围分配给不同的线程来处理
    // parallel_for_(Range(0, kp1.size()),
    //               std::bind(&OpticalFlowTracker::calculateOpticalFlow, &tracker, placeholders::_1));
    for (int i = 0; i < kp1.size(); i++)
    {
        tracker.calculateOpticalFlow(Range(i, i + 1));
    }
}

void OpticalFlowTracker::calculateOpticalFlow(const Range &range)
{
    // parameters
    int half_patch_size = 4;    // 光流跟踪中每个特征点周围的搜索窗口大小
    int iterations = 10;    // 迭代次数
    for (size_t i = range.start; i < range.end; i++)
    {
        auto kp = kp1[i];
        double dx = 0, dy = 0; // dx,dy need to be estimated
        if (has_initial)
        {
            dx = kp2[i].pt.x - kp.pt.x;
            dy = kp2[i].pt.y - kp.pt.y;
        }

        double cost = 0, lastCost = 0;
        bool succ = true; // indicate if this point succeeded

        // Gauss-Newton iterations
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero(); // hessian
        Eigen::Vector2d b = Eigen::Vector2d::Zero(); // bias
        Eigen::Vector2d J;                           // jacobian
        for (int iter = 0; iter < iterations; iter++)
        {
            if (inverse == false)
            {
                H = Eigen::Matrix2d::Zero();
                b = Eigen::Vector2d::Zero();
            }
            else
            {
                // only reset b
                b = Eigen::Vector2d::Zero();
            }

            cost = 0;

            // compute cost and jacobian
            for (int x = -half_patch_size; x < half_patch_size; x++)    // 遍历特征点周围的像素8x8块
                for (int y = -half_patch_size; y < half_patch_size; y++)
                {
                    double error = GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y) -
                                   GetPixelValue(img2, kp.pt.x + x + dx, kp.pt.y + y + dy); // GetPixelValue 使用双线性插值得到亚像素坐标的灰度值 error 表示 img1 和 img2 中对应位置的像素差异
                    ; // Jacobian
                    if (inverse == false)
                    {
                        J = -1.0 * Eigen::Vector2d(
                                       0.5 * (GetPixelValue(img2, kp.pt.x + dx + x + 1, kp.pt.y + dy + y) -
                                              GetPixelValue(img2, kp.pt.x + dx + x - 1, kp.pt.y + dy + y)), // x方向梯度
                                       0.5 * (GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y + 1) -
                                              GetPixelValue(img2, kp.pt.x + dx + x, kp.pt.y + dy + y - 1))); // y方向梯度
                    }
                    else if (iter == 0)
                    {
                        // in inverse mode, J keeps same for all iterations
                        // NOTE this J does not change when dx, dy is updated, so we can store it and only compute error
                        J = -1.0 * Eigen::Vector2d(
                                       0.5 * (GetPixelValue(img1, kp.pt.x + x + 1, kp.pt.y + y) -
                                              GetPixelValue(img1, kp.pt.x + x - 1, kp.pt.y + y)),
                                       0.5 * (GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y + 1) -
                                              GetPixelValue(img1, kp.pt.x + x, kp.pt.y + y - 1)));
                    }
                    // compute H, b and set cost;
                    b += -error * J;
                    cost += error * error;
                    if (inverse == false || iter == 0)
                    {
                        // also update H
                        H += J * J.transpose();
                    }
                }

            // compute update
            Eigen::Vector2d update = H.ldlt().solve(b);

            if (std::isnan(update[0]))
            {
                // sometimes occurred when we have a black or white patch and H is irreversible
                cout << "update is nan" << endl;
                succ = false;
                break;
            }

            if (iter > 0 && cost > lastCost)
            {
                break;
            }

            // update dx, dy
            dx += update[0];
            dy += update[1];
            lastCost = cost;
            succ = true;

            if (update.norm() < 1e-2)
            {
                // converge
                break;
            }
        }

        success[i] = succ;

        // set kp2
        kp2[i].pt = kp.pt + Point2f(dx, dy); // 更新了特征点 kp2[i] 在图像 img2 中的坐标
    }
}

void OpticalFlowMultiLevel( // 多层光流
    const Mat &img1,
    const Mat &img2,
    const vector<KeyPoint> &kp1,
    vector<KeyPoint> &kp2,
    vector<bool> &success,
    bool inverse)
{

    // parameters
    int pyramids = 4;   // 图像金字塔层数
    double pyramid_scale = 0.5; // 每层的缩放比例
    double scales[] = {1.0, 0.5, 0.25, 0.125};  // 缩放比例数组

    // create pyramids
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    vector<Mat> pyr1, pyr2; // image pyramids 用于存储图像 img1 和 img2 的各层金字塔图像。
    for (int i = 0; i < pyramids; i++)  // 遍历每层金字塔
    {
        if (i == 0)
        {
            pyr1.push_back(img1);
            pyr2.push_back(img2);
        }
        else
        {
            Mat img1_pyr, img2_pyr;
            cv::resize(pyr1[i - 1], img1_pyr,
                       cv::Size(pyr1[i - 1].cols * pyramid_scale, pyr1[i - 1].rows * pyramid_scale));   // 缩放
            cv::resize(pyr2[i - 1], img2_pyr,
                       cv::Size(pyr2[i - 1].cols * pyramid_scale, pyr2[i - 1].rows * pyramid_scale));
            pyr1.push_back(img1_pyr);
            pyr2.push_back(img2_pyr);
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "build pyramid time: " << time_used.count() << endl;

    // coarse-to-fine LK tracking in pyramids
    vector<KeyPoint> kp1_pyr, kp2_pyr;  // 将原始图像中 kp1 的特征点坐标进行缩放 适应图像金字塔的最顶层（最低分辨率层）
    for (auto &kp : kp1)    // 遍历 kp1 中的每个特征点 kp
    {
        auto kp_top = kp;
        kp_top.pt *= scales[pyramids - 1];
        kp1_pyr.push_back(kp_top);
        kp2_pyr.push_back(kp_top);
    }

    for (int level = pyramids - 1; level >= 0; level--)
    {
        // from coarse to fine
        success.clear();
        t1 = chrono::steady_clock::now();
        OpticalFlowSingleLevel(pyr1[level], pyr2[level], kp1_pyr, kp2_pyr, success, inverse, true); // kp2_pyr为估计的光流像素
        t2 = chrono::steady_clock::now();
        auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "track pyr " << level << " cost time: " << time_used.count() << endl;

        if (level > 0)     // 将结果放大到下一层
        {
            for (auto &kp : kp1_pyr)
                kp.pt /= pyramid_scale;
            for (auto &kp : kp2_pyr)
                kp.pt /= pyramid_scale;
        }
    }

    for (auto &kp : kp2_pyr)
        kp2.push_back(kp);
}